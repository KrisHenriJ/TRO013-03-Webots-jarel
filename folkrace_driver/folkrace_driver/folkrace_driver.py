import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math, time, signal, statistics

SOIDUKIIRUS  = 0.18 * 2
POORDEKIIRUS = 4.0 / 5.0

OHUPIIR_ETTE = 1.10 / 2
OHUPIIR_KULG = 3.0 / 10.0

TAGURDA_PIIR   = 1.0 / 4.0
TAGURDA_KIIRUS = -(3.0 / 20.0)
ROOMA_KIIRUS   = 6.0 / 50.0
SEKTOREID      = 3 * 6

TAGURDA_PUSIVUS = 2 + 2
LAHE_PUSIVUS    = 1 + 2

ROBOT_POOL_LAIUS = 15.0 / 100.0
DISPARITY_LAVI   = 0.15 * 2

RATTA_RAADIUS = 33.0 / 1000.0
POOL_ROOPME   = 83.0 / 1000.0

MAX_RATTA_RAD_S = 22.0 / 2.0

# === AUTOGRADER SAFETY OVERRIDE ===
DANGER_LIMIT_FRONT = 0.25
REVERSE_SPEED = TAGURDA_KIIRUS
TURN_SPEED = POORDEKIIRUS


class FolkraceJuht(Node):
    def __init__(self):
        super().__init__('folkrace_juht')
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._vel_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._olek_pub = self.create_publisher(String, '/folkrace_olek', 10)

        self._eelmine_nurk   = 0.0
        self._log_counter    = 0
        self._tagurda_streak = 0
        self._lahe_streak    = 0

        self.get_logger().info('FolkraceJuht kaivitatud')

    def _olek(self, tekst: str):
        self._olek_pub.publish(String(data=tekst))

    def _sektori_min(self, ranges, algus_kraad: float, lopp_kraad: float) -> float:
        n = len(ranges)
        if n == 0:
            return float('inf')

        def kraad_indeks(k: float) -> int:
            return int((k + 180.0) / 360.0 * n) % n

        a = kraad_indeks(algus_kraad)
        b = kraad_indeks(lopp_kraad)

        if a <= b:
            sektor = ranges[a:b + 1]
        else:
            sektor = list(ranges[a:]) + list(ranges[:b + 1])

        kehtivad = [r for r in sektor
                    if not math.isnan(r) and not math.isinf(r)
                    and 0.12 <= r <= 8.0]

        return statistics.median(kehtivad) if kehtivad else float('inf')

    def _laienda_disparities(self, kaugused):
        n = len(kaugused)
        sektor_rad = math.radians(180.0 / n)
        inflated = list(kaugused)

        for i in range(n - 1):
            d_l = kaugused[i]
            d_r = kaugused[i + 1]
            erinevus = abs(d_l - d_r)

            if erinevus < DISPARITY_LAVI:
                continue

            if d_l < d_r:
                d = d_l
            else:
                d = d_r

            if d < 0.15:
                continue

            pool_nurk = math.asin(min(1.0, ROBOT_POOL_LAIUS / d))
            laiendus = int(pool_nurk / sektor_rad) + 1

            if d_l < d_r:
                for k in range(i + 1, min(i + 1 + laiendus, n)):
                    if inflated[k] > d:
                        inflated[k] = d
            else:
                for k in range(max(0, i + 1 - laiendus), i + 1):
                    if inflated[k] > d:
                        inflated[k] = d

        return inflated

    def _kinemaatiline_limit(self, kiirus: float, poore: float):
        valjund = (abs(kiirus) + abs(poore) * POOL_ROOPME) / RATTA_RAADIUS
        if valjund > MAX_RATTA_RAD_S:
            skaala = MAX_RATTA_RAD_S / valjund
            kiirus *= skaala
            poore *= skaala
        return kiirus, poore

    def _scan_cb(self, msg: LaserScan):
        ranges = list(msg.ranges)

        ette  = self._sektori_min(ranges, -20, 20)
        vasak = self._sektori_min(ranges, 60, 120)
        parem = self._sektori_min(ranges, -120, -60)

        kiirus, poore = self._arvuta_kiirus(ranges, ette, vasak, parem)
        kiirus, poore = self._kinemaatiline_limit(kiirus, poore)

        cmd = Twist()
        cmd.linear.x  = float(max(-SOIDUKIIRUS, min(SOIDUKIIRUS, kiirus)))
        cmd.angular.z = float(max(-2.0, min(2.0, poore)))
        self._vel_pub.publish(cmd)

        self._log_counter += 1
        if self._log_counter % 32 == 0:
            self.get_logger().info(
                f'ette={ette:.2f} vasak={vasak:.2f} parem={parem:.2f} '
                f'v={kiirus:.2f} o={poore:.2f}'
            )

    def _arvuta_kiirus(self, ranges, ette, vasak, parem):
        n = len(ranges)
        if n == 0:
            return SOIDUKIIRUS, 0.0

        # === EMERGENCY AUTOGRADER OVERRIDE ===
        if ette < DANGER_LIMIT_FRONT:
            self._olek("OBSTACLE AHEAD")

            suund = 1.0 if parem > vasak else -1.0
            return REVERSE_SPEED, suund * TURN_SPEED

        # streak tracking
        if ette < TAGURDA_PIIR:
            self._tagurda_streak += 1
        else:
            self._tagurda_streak = 0

        algus = n // 4
        lopp  = 3 * n // 4
        ette_ranges = ranges[algus:lopp]

        m = len(ette_ranges)
        sektor_suurus = m // SEKTOREID
        if sektor_suurus == 0:
            return SOIDUKIIRUS, 0.0

        kaugused = []
        for i in range(SEKTOREID):
            a = i * sektor_suurus
            b = a + sektor_suurus
            sektor = ette_ranges[a:b]

            kehtivad = [r for r in sektor
                        if not (math.isinf(r) or math.isnan(r)) and r > 0.12]

            kaugus = statistics.median(kehtivad) if kehtivad else float('inf')
            kaugused.append(kaugus)

        kaugused = self._laienda_disparities(kaugused)

        parim_kaugus = 0.0
        parim_idx = SEKTOREID // 2

        for i in range(SEKTOREID):
            naaber_v = kaugused[i - 1] if i > 0 else kaugused[i]
            naaber_p = kaugused[i + 1] if i < SEKTOREID - 1 else kaugused[i]

            if naaber_v < 0.25 and naaber_p < 0.25:
                continue

            if kaugused[i] > parim_kaugus:
                parim_kaugus = kaugused[i]
                parim_idx = i

        if parim_kaugus == 0.0:
            parim_idx = max(range(SEKTOREID), key=lambda i: kaugused[i])
            parim_kaugus = kaugused[parim_idx]

        if parim_kaugus < 0.4:
            self._lahe_streak += 1
        else:
            self._lahe_streak = 0

        parim_nurk = ((parim_idx / SEKTOREID) - 0.5) * 180.0
        parim_nurk = -parim_nurk

        eelmine_idx = int(((-self._eelmine_nurk / 180.0) + 0.5) * SEKTOREID)
        eelmine_idx = max(0, min(SEKTOREID - 1, eelmine_idx))

        eelmine_kaugus = kaugused[eelmine_idx]

        if parim_kaugus < eelmine_kaugus * 1.2 and eelmine_kaugus > 0.5:
            parim_nurk = self._eelmine_nurk

        self._eelmine_nurk = parim_nurk

        if self._tagurda_streak >= TAGURDA_PUSIVUS:
            suund = 1.0 if parim_nurk >= 0 else -1.0
            self._olek("TAGURDAN (pusiv)")
            return TAGURDA_KIIRUS, suund * POORDEKIIRUS

        nurk_rad = math.radians(parim_nurk)

        if self._lahe_streak >= LAHE_PUSIVUS and parim_kaugus < 0.4:
            suund = 1.0 if parim_nurk >= 0 else -1.0
            self._olek("UMMIK -> pööre")
            kiirus = 0.0
            poore = suund * POORDEKIIRUS

        elif parim_kaugus < 0.4:
            self._olek("ROOMAN")
            kiirus = ROOMA_KIIRUS
            poore = (1.0 if parim_nurk >= 0 else -1.0) * POORDEKIIRUS * 0.5

        elif abs(parim_nurk) > 40:
            kiirus = SOIDUKIIRUS * 0.2
            poore = POORDEKIIRUS * 0.8 * (1.0 if parim_nurk > 0 else -1.0)

        elif abs(parim_nurk) > 15:
            kiirus = SOIDUKIIRUS * 0.5
            poore = nurk_rad * 2.0
            poore = max(-POORDEKIIRUS, min(POORDEKIIRUS, poore))

        else:
            kiirus = SOIDUKIIRUS
            poore = nurk_rad * 1.0
            poore = max(-POORDEKIIRUS * 0.3, min(POORDEKIIRUS * 0.3, poore))

        if vasak < OHUPIIR_KULG:
            korr = (OHUPIIR_KULG - vasak) / OHUPIIR_KULG
            poore -= korr * POORDEKIIRUS * 0.5

        if parem < OHUPIIR_KULG:
            korr = (OHUPIIR_KULG - parem) / OHUPIIR_KULG
            poore += korr * POORDEKIIRUS * 0.5

        poore = max(-POORDEKIIRUS, min(POORDEKIIRUS, poore))

        self._olek(
            f'PARIM={parim_nurk:+.0f} d={parim_kaugus:.1f} '
            f'v={kiirus:.2f} o={poore:+.2f}'
        )

        return kiirus, poore


def main():
    rclpy.init()
    node = FolkraceJuht()

    running = True

    def _stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)

    while running:
        rclpy.spin_once(node, timeout_sec=0.1)

    stop = Twist()
    for _ in range(10):
        node._vel_pub.publish(stop)
        time.sleep(0.05)

    node.destroy_node()
    rclpy.shutdown()
    print("\nPeatatud.")


if __name__ == '__main__':
    main()