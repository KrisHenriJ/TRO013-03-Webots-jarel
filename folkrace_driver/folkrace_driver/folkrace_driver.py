import math
import signal
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

DRIVING_SPEED = 0.18 * 2
TURN_SPEED    = 4.0 / 5.0

DANGER_LIMIT_FRONT = 1.10 / 2
DANGER_LIMIT_SIDE  = 3.0 / 10.0

REVERSE_THRESHOLD = 1.0 / 4.0
REVERSE_SPEED     = -(3.0 / 20.0)
CREEP_SPEED       = 6.0 / 50.0

SECTORS_COUNT      = 3 * 6
REVERSE_PERSISTENCE = 2 + 2
CLOSE_PERSISTENCE   = 1 + 2

ROBOT_HALF_WIDTH       = 15.0 / 100.0
DISPARITY_THRESHOLD    = 0.15 * 2
WHEEL_RADIUS           = 33.0 / 1000.0
HALF_TRACK             = 83.0 / 1000.0
MAX_WHEEL_ANGULAR_SPEED = 22.0 / 2.0

LIDAR_MIN_RANGE  = 0.12
LIDAR_MAX_RANGE  = 8.0
SCAN_TOPIC       = '/scan'
CMD_VEL_TOPIC    = '/cmd_vel'
STATE_TOPIC      = '/folkrace_state'
LOG_INTERVAL     = 32

def _valid_range(r: float) -> bool:
    return (
        not math.isnan(r)
        and not math.isinf(r)
        and LIDAR_MIN_RANGE <= r <= LIDAR_MAX_RANGE
    )

def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

class FolkraceDriver(Node):

    def __init__(self):
        super().__init__('folkrace_driver')
        self._setup_comms()
        self._reset_state()
        self.get_logger().info('FolkraceDriver started')

    def _setup_comms(self):
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_callback, 10)
        self._vel_pub   = self.create_publisher(Twist,  CMD_VEL_TOPIC, 10)
        self._state_pub = self.create_publisher(String, STATE_TOPIC,   10)

    def _reset_state(self):
        self._previous_angle  = 0.0
        self._log_counter     = 0
        self._reverse_streak  = 0
        self._close_streak    = 0

    def _publish_state(self, text: str):
        self._state_pub.publish(String(data=text))

    def _publish_cmd(self, speed: float, turn: float):
        cmd = Twist()
        cmd.linear.x  = float(_clamp(speed, -DRIVING_SPEED, DRIVING_SPEED))
        cmd.angular.z = float(_clamp(turn,  -2.0,           2.0))
        self._vel_pub.publish(cmd)

    def _log_scan(self, front: float, left: float, right: float,
                  speed: float, turn: float):
        self._log_counter += 1
        if self._log_counter % LOG_INTERVAL == 0:
            self.get_logger().info(
                f'front={front:.2f}m  left={left:.2f}m  right={right:.2f}m  '
                f'→ v={speed:.2f}m/s  ω={turn:.2f}rad/s'
            )

    def _deg_to_index(self, deg: float, n: int) -> int:
        return int((deg + 180.0) / 360.0 * n) % n

    def _get_sector_min(self, ranges: list, start_deg: float,
                        end_deg: float) -> float:
        n = len(ranges)
        if n == 0:
            return float('inf')

        a = self._deg_to_index(start_deg, n)
        b = self._deg_to_index(end_deg,   n)

        sector = ranges[a:b + 1] if a <= b else list(ranges[a:]) + list(ranges[:b + 1])
        valid   = [r for r in sector if _valid_range(r)]
        return statistics.median(valid) if valid else float('inf')

    def _inflate_disparities(self, distances: list) -> list:
        n          = len(distances)
        sector_rad = math.radians(180.0 / n)
        inflated   = list(distances)

        for i in range(n - 1):
            d_left  = distances[i]
            d_right = distances[i + 1]

            if abs(d_left - d_right) < DISPARITY_THRESHOLD:
                continue

            if d_left < d_right:
                self._inflate_towards_right(inflated, distances, i, d_left, sector_rad)
            else:
                self._inflate_towards_left(inflated, distances, i, d_right, sector_rad)

        return inflated

    def _inflation_steps(self, d: float, sector_rad: float) -> int:
        half_angle = math.asin(min(1.0, ROBOT_HALF_WIDTH / d))
        return int(half_angle / sector_rad) + 1

    def _inflate_towards_right(self, inflated: list, distances: list,
                                i: int, d: float, sector_rad: float):
        if d < 0.15:
            return
        n       = len(distances)
        steps   = self._inflation_steps(d, sector_rad)
        for k in range(i + 1, min(i + 1 + steps, n)):
            if inflated[k] > d:
                inflated[k] = d

    def _inflate_towards_left(self, inflated: list, distances: list,
                               i: int, d: float, sector_rad: float):
        if d < 0.15:
            return
        steps = self._inflation_steps(d, sector_rad)
        for k in range(max(0, i + 1 - steps), i + 1):
            if inflated[k] > d:
                inflated[k] = d

    def _kinematic_limit(self, speed: float, turn: float):
        output      = (abs(speed) + abs(turn) * HALF_TRACK) / WHEEL_RADIUS
        if output > MAX_WHEEL_ANGULAR_SPEED:
            scale   = MAX_WHEEL_ANGULAR_SPEED / output
            speed  *= scale
            turn   *= scale
        return speed, turn

    def _scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)

        front = self._get_sector_min(ranges,  -20,   20)
        left  = self._get_sector_min(ranges,   60,  120)
        right = self._get_sector_min(ranges, -120,  -60)

        speed, turn = self._calculate_speed(ranges, front, left, right)
        speed, turn = self._kinematic_limit(speed, turn)

        self._publish_cmd(speed, turn)
        self._log_scan(front, left, right, speed, turn)

    def _calculate_speed(self, ranges: list, front: float,
                         left: float, right: float):
        n = len(ranges)
        if n == 0:
            return DRIVING_SPEED, 0.0

        self._update_reverse_streak(front)

        distances = self._build_sector_distances(ranges, n)
        if distances is None:
            return DRIVING_SPEED, 0.0

        distances = self._inflate_disparities(distances)

        best_idx, best_distance = self._find_best_sector(distances)
        self._update_close_streak(best_distance)

        best_angle = self._sector_to_angle(best_idx)
        best_angle = self._apply_angle_hysteresis(best_angle, best_distance, distances)
        self._previous_angle = best_angle

        return self._decide_motion(best_angle, best_distance, left, right)

    def _update_reverse_streak(self, front: float):
        if front < REVERSE_THRESHOLD:
            self._reverse_streak += 1
        else:
            self._reverse_streak = 0

    def _update_close_streak(self, best_distance: float):
        if best_distance < 0.4:
            self._close_streak += 1
        else:
            self._close_streak = 0

    def _build_sector_distances(self, ranges: list, n: int):
        start = n // 4
        end   = 3 * n // 4
        front_ranges = ranges[start:end]
        m = len(front_ranges)

        sector_size = m // SECTORS_COUNT
        if sector_size == 0:
            return None

        distances = []
        for i in range(SECTORS_COUNT):
            a      = i * sector_size
            b      = a + sector_size
            sector = front_ranges[a:b]
            valid  = [r for r in sector if not (math.isinf(r) or math.isnan(r)) and r > LIDAR_MIN_RANGE]
            distances.append(statistics.median(valid) if valid else LIDAR_MAX_RANGE)
        return distances

    def _find_best_sector(self, distances: list):
        best_distance = 0.0
        best_idx      = SECTORS_COUNT // 2

        for i in range(SECTORS_COUNT):
            nb_left  = distances[i - 1] if i > 0               else distances[i]
            nb_right = distances[i + 1] if i < SECTORS_COUNT - 1 else distances[i]
            if nb_left < 0.25 and nb_right < 0.25:
                continue
            if distances[i] > best_distance:
                best_distance = distances[i]
                best_idx      = i

        if best_distance == 0.0:
            best_idx      = max(range(SECTORS_COUNT), key=lambda i: distances[i])
            best_distance = distances[best_idx]

        return best_idx, best_distance

    def _sector_to_angle(self, idx: int) -> float:
        return -((idx / SECTORS_COUNT) - 0.5) * 180.0

    def _apply_angle_hysteresis(self, best_angle: float,
                                best_distance: float,
                                distances: list) -> float:
        previous_idx = int(((-self._previous_angle / 180.0) + 0.5) * SECTORS_COUNT)
        previous_idx = _clamp(previous_idx, 0, SECTORS_COUNT - 1)
        previous_distance = distances[int(previous_idx)]

        if best_distance < previous_distance * 1.2 and previous_distance > 0.5:
            return self._previous_angle
        return best_angle

    def _decide_motion(self, best_angle: float, best_distance: float,
                       left: float, right: float):

        if self._reverse_streak >= REVERSE_PERSISTENCE:
            direction = 1.0 if best_angle >= 0 else -1.0
            label     = 'L' if direction > 0 else 'R'
            self._publish_state(f'REVERSING (persistent) → {label}')
            return REVERSE_SPEED, direction * TURN_SPEED

        angle_rad = math.radians(best_angle)
        direction = 1.0 if best_angle >= 0 else -1.0

        if self._close_streak >= CLOSE_PERSISTENCE and best_distance < 0.4:
            label = 'L' if direction > 0 else 'R'
            self._publish_state(f'STUCK → turning {label}')
            speed = 0.0
            turn  = direction * TURN_SPEED

        elif best_distance < 0.4:
            self._publish_state(f'CREEPING (temporary) → {best_angle:+.0f}°')
            speed = CREEP_SPEED
            turn  = direction * TURN_SPEED * 0.5

        elif abs(best_angle) > 40:
            speed = DRIVING_SPEED * 0.2
            turn  = TURN_SPEED * 0.8 * direction

        elif abs(best_angle) > 15:
            speed = DRIVING_SPEED * 0.5
            turn  = _clamp(angle_rad * 2.0, -TURN_SPEED, TURN_SPEED)

        else:
            speed = DRIVING_SPEED
            turn  = _clamp(angle_rad * 1.0, -TURN_SPEED * 0.3, TURN_SPEED * 0.3)

        turn = self._apply_side_corrections(turn, left, right)
        turn = _clamp(turn, -TURN_SPEED, TURN_SPEED)

        self._publish_state(
            f'BEST={best_angle:+.0f}° d={best_distance:.1f}m '
            f'v={speed:.2f} ω={turn:+.2f}'
        )
        return speed, turn

    def _apply_side_corrections(self, turn: float,
                                left: float, right: float) -> float:
        if left < DANGER_LIMIT_SIDE:
            correction = (DANGER_LIMIT_SIDE - left) / DANGER_LIMIT_SIDE
            turn -= correction * TURN_SPEED * 0.5
        if right < DANGER_LIMIT_SIDE:
            correction = (DANGER_LIMIT_SIDE - right) / DANGER_LIMIT_SIDE
            turn += correction * TURN_SPEED * 0.5
        return turn

def _make_stop_handler(flag: list):
    def _handler(sig, frame):
        flag[0] = False
    return _handler

def main():
    rclpy.init()
    node = FolkraceDriver()

    running = [True]
    signal.signal(signal.SIGINT, _make_stop_handler(running))

    while running[0]:
        rclpy.spin_once(node, timeout_sec=0.1)

    stop = Twist()
    for _ in range(10):
        node._vel_pub.publish(stop)
        time.sleep(0.05)

    node.destroy_node()
    rclpy.shutdown()
    print('\nStopped.')

if __name__ == '__main__':
    main()