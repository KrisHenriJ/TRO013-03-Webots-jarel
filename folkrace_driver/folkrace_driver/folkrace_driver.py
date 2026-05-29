import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math, time, signal, statistics

SOIDUKIIRUS  = 0.18 * 2
PÖÖRDEKIIRUS = 4.0 / 5.0

OHUPIIR_ETTE = 1.10 / 2
OHUPIIR_KÜLG = 3.0 / 10.0

TAGURDA_PIIR   = 1.0 / 4.0
TAGURDA_KIIRUS = -(3.0 / 20.0)
ROOMA_KIIRUS   = 6.0 / 50.0
SEKTOREID      = 3 * 6

TAGURDA_PÜSIVUS = 2 + 2
LÄHE_PÜSIVUS    = 1 + 2

ROBOT_POOL_LAIUS = 15.0 / 100.0
DISPARITY_LÄVI   = 0.15 * 2

RATTA_RAADIUS = 33.0 / 1000.0
POOL_RÖÖPME   = 83.0 / 1000.0

MAX_RATTA_RAD_S = 22.0 / 2.0


class FolkraceJuht(Node):
    def __init__(self):
        super().__init__('folkrace_juht')
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._vel_pub  = self.create_publisher(Twist,  '/cmd_vel',       10)
        self._olek_pub = self.create_publisher(String, '/folkrace_olek', 10)

        self._eelmine_nurk   = 0.0
        self._log_counter    = 0
        self._tagurda_streak = 0
        self._lähe_streak    = 0

        self.get_logger().info('FolkraceJuht käivitatud')

   
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
            
            if erinevus < DISPARITY_LÄVI:
                continue

            if d_l < d_r:
                d = d_l
                if d < 0.15:
                    continue
                pool_nurk = math.asin(min(1.0, ROBOT_POOL_LAIUS / d))
                laiendus = int(pool_nurk / sektor_rad) + 1
                for k in range(i + 1, min(i + 1 + laiendus, n)):
                    if inflated[k] > d:
                        inflated[k] = d
            else:
                d = d_r
                if d < 0.15:
                    continue
                pool_nurk = math.asin(min(1.0, ROBOT_POOL_LAIUS / d))
                laiendus = int(pool_nurk / sektor_rad) + 1
                for k in range(max(0, i + 1 - laiendus), i + 1):
                    if inflated[k] > d:
                        inflated[k] = d

        return inflated

    def _kinemaatiline_limit(self, kiirus: float, pööre: float):
        
        väljund = (abs(kiirus) + abs(pööre) * POOL_RÖÖPME) / RATTA_RAADIUS
        max_lubatud = MAX_RATTA_RAD_S
        if väljund > max_lubatud:
            skaala = max_lubatud / väljund
            kiirus *= skaala
            pööre *= skaala
        return kiirus, pööre

    def _scan_cb(self, msg: LaserScan):
        ranges = list(msg.ranges)

        ette  = self._sektori_min(ranges, -20,   20)
        vasak = self._sektori_min(ranges,  60,  120)
        parem = self._sektori_min(ranges, -120, -60)

        kiirus, pööre = self._arvuta_kiirus(ranges, ette, vasak, parem)

        kiirus, pööre = self._kinemaatiline_limit(kiirus, pööre)

        cmd = Twist()
        cmd.linear.x  = float(max(-SOIDUKIIRUS, min(SOIDUKIIRUS, kiirus)))
        cmd.angular.z = float(max(-2.0, min(2.0, pööre)))
        self._vel_pub.publish(cmd)

        self._log_counter += 1
        if self._log_counter % 32 == 0:
            self.get_logger().info(
                f'ette={ette:.2f}m  vasak={vasak:.2f}m  parem={parem:.2f}m  '
                f'→ v={kiirus:.2f}m/s  ω={pööre:.2f}rad/s'
            )

    
    def _arvuta_kiirus(self, ranges, ette, vasak, parem):
        n = len(ranges)
        if n == 0:
            return SOIDUKIIRUS, 0.0

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
            kaugus = statistics.median(kehtivad) if kehtivad else 8.0
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
            self._lähe_streak += 1
        else:
            self._lähe_streak = 0

        parim_nurk = ((parim_idx / SEKTOREID) - 0.5) * 180.0
        parim_nurk = -parim_nurk

        eelmine_idx = int(((-self._eelmine_nurk / 180.0) + 0.5) * SEKTOREID)
        eelmine_idx = max(0, min(SEKTOREID - 1, eelmine_idx))
        eelmine_kaugus = kaugused[eelmine_idx]

        if parim_kaugus < eelmine_kaugus * 1.2 and eelmine_kaugus > 0.5:
            parim_nurk = self._eelmine_nurk

        self._eelmine_nurk = parim_nurk

        if self._tagurda_streak >= TAGURDA_PÜSIVUS:
            suund = 1.0 if parim_nurk >= 0 else -1.0
            self._olek(f'TAGURDAN (püsiv) → {"V" if suund > 0 else "P"}')
            return TAGURDA_KIIRUS, suund * PÖÖRDEKIIRUS

        nurk_rad = math.radians(parim_nurk)

        if self._lähe_streak >= LÄHE_PÜSIVUS and parim_kaugus < 0.4:
            suund = 1.0 if parim_nurk >= 0 else -1.0
            self._olek(f'UMMIK → pööran {"V" if suund > 0 else "P"}')
            kiirus = 0.0
            pööre = suund * PÖÖRDEKIIRUS
        elif parim_kaugus < 0.4:
            self._olek(f'ROOMAN (ajutine) → {parim_nurk:+.0f}°')
            kiirus = ROOMA_KIIRUS
            pööre = (1.0 if parim_nurk >= 0 else -1.0) * PÖÖRDEKIIRUS * 0.5
        elif abs(parim_nurk) > 40:
            kiirus = SOIDUKIIRUS * 0.2
            pööre = PÖÖRDEKIIRUS * 0.8 * (1.0 if parim_nurk > 0 else -1.0)
        elif abs(parim_nurk) > 15:
            kiirus = SOIDUKIIRUS * 0.5
            pööre = nurk_rad * 2.0
            pööre = max(-PÖÖRDEKIIRUS, min(PÖÖRDEKIIRUS, pööre))
        else:
            kiirus = SOIDUKIIRUS
            pööre = nurk_rad * 1.0
            pööre = max(-PÖÖRDEKIIRUS * 0.3, min(PÖÖRDEKIIRUS * 0.3, pööre))

        if vasak < OHUPIIR_KÜLG:
            korr = (OHUPIIR_KÜLG - vasak) / OHUPIIR_KÜLG
            pööre -= korr * PÖÖRDEKIIRUS * 0.5
        if parem < OHUPIIR_KÜLG:
            korr = (OHUPIIR_KÜLG - parem) / OHUPIIR_KÜLG
            pööre += korr * PÖÖRDEKIIRUS * 0.5

        pööre = max(-PÖÖRDEKIIRUS, min(PÖÖRDEKIIRUS, pööre))

        self._olek(
            f'PARIM={parim_nurk:+.0f}° d={parim_kaugus:.1f}m '
            f'v={kiirus:.2f} ω={pööre:+.2f}'
        )
        return kiirus, pööre


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
    print('\nPeatatud.')


if __name__ == '__main__':
    main()