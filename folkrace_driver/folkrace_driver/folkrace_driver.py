"""Moodul 03: Automaatne folkrace sõitmine.

Ülesanne:
  Kirjuta reaktiivne sõlm, mis sõidab folkrace rajal automaatselt
  ilma seintesse põrkamata. Robot loeb lidarit ja otsustab iga
  mõõtmise põhjal kuhu sõita.

Nõuded:
  - Robot teeb vähemalt ÜHE TÄISRINGI
  - Robot ei põrka seintesse
  - Robot suudab üle SILLA minna

Käivita:
  Terminal 1: ros2 launch yahboom_webots webots.launch.py
  Terminal 2: ros2 run folkrace_driver folkrace_driver

Lidar indeksid (720 kiirt, 360°):
  ranges[0]   = -180° = otse TAGA
  ranges[180] =  -90° = PAREMALE
  ranges[360] =    0° = otse ETTE
  ranges[540] =  +90° = VASAKULE
  ranges[719] = +180° = otse TAGA

  Ette ±15° = indeksid 330..390 (ümber 360)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FolkraceDriver(Node):

    def __init__(self):
        super().__init__('folkrace_driver')

        # Publisher liikumiskäskude jaoks
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber lidari andmete jaoks
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info('Folkrace driver käivitatud!')

    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        # ----------------------------------------------------------
        # 1. Loe lidar andmeid: ees, vasakul, paremal
        # ----------------------------------------------------------

        # TODO: arvuta kaugus ETTE (±15° ehk ~30 kiirt ette suunas)
        #   Indeksid: ranges[n//2-15:n//2+15] (ümber keskpunkti)
        #   Kasuta min() et leida lähim takistus
        #   Filtreeri välja inf ja liiga lähedad (< 0.12m)
        #
        # Vihje:
        #   mid = n // 2
        #   front_ranges = ranges[mid-15:mid+15]
        #   front = min((r for r in front_ranges if 0.12 < r < 8.0), default=8.0)
        front = 8.0  # TODO: asenda seda üleval oleva koodiga

        # TODO: arvuta kaugus VASAKULE (~90° ehk indeksid 3*n//4 - 10 .. 3*n//4 + 10)
        # Vihje: left = min((r for r in ranges[530:550] if 0.12 < r < 8.0), default=8.0)
        left = 8.0  # TODO: asenda

        # TODO: arvuta kaugus PAREMALE (~-90° ehk indeksid n//4 - 10 .. n//4 + 10)
        # Vihje: right = min((r for r in ranges[170:190] if 0.12 < r < 8.0), default=8.0)
        right = 8.0  # TODO: asenda

        # ----------------------------------------------------------
        # 2. Otsusta kuhu sõita
        # ----------------------------------------------------------
        cmd = Twist()

        # TODO: kirjuta reaktiivne loogika
        #
        # Põhimõte:
        #   - Kui ees on vaba (front > 0.5m): sõida edasi
        #     * cmd.linear.x = 0.3  (edasi kiirus m/s)
        #     * Hoia raja keskel: kui left > right, kalluta vasakule
        #       cmd.angular.z = 0.2 (positiivne = vasakule)
        #
        #   - Kui ees on takistus (front <= 0.5m): peatu ja pöördu
        #     * cmd.linear.x = 0.0
        #     * Pöördu vabama külje poole:
        #       cmd.angular.z = 0.8 kui left > right, muidu -0.8
        #
        # Vihje: alusta lihtsa if/else'iga, siis täiusta

        # ----------------------------------------------------------
        # 3. Saada käsk robotile
        # ----------------------------------------------------------
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FolkraceDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
