import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class MavrosInterface(Node):
    """MAVROS ile iletişimi basitleştiren arayüz sınıfı."""

    def __init__(self):
        super().__init__('mavros_interface')

        # Güvenilir bir QoS profili tanımla (MAVROS için önemli)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.local_pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)

        # Publishers
        self.setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)

        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Sınıf değişkenleri
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        self.get_logger().info("MavrosInterface düğümü başlatıldı.")
        self.wait_for_connection()

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def wait_for_connection(self):
        """FCU bağlantısının kurulmasını bekler."""
        self.get_logger().info("Uçuş kontrolcüsü bağlantısı bekleniyor...")
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Uçuş kontrolcüsüne başarıyla bağlanıldı.")

    def set_mode(self, mode_name):
        """İHA'nın uçuş modunu ayarlar."""
        # Servisin aktif olmasını bekle
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SetMode servisi bekleniyor...')
        
        req = SetMode.Request()
        req.custom_mode = mode_name
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().mode_sent:
            self.get_logger().info(f"Mod başarıyla '{mode_name}' olarak ayarlandı.")
            return True
        self.get_logger().error(f"Mod '{mode_name}' olarak ayarlanamadı.")
        return False

    def arm(self):
        """İHA'yı arm eder."""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arming servisi bekleniyor...')
            
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("İHA başarıyla arm edildi.")
            return True
        self.get_logger().error("İHA arm edilemedi.")
        return False

    def disarm(self):
        """İHA'yı disarm eder."""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arming servisi bekleniyor...')
            
        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("İHA başarıyla disarm edildi.")
            return True
        self.get_logger().error("İHA disarm edilemedi.")
        return False

    def takeoff(self, altitude):
        """İHA'yı belirtilen irtifaya otonom olarak kaldırır."""
        self.get_logger().info(f"{altitude} metreye otonom kalkış yapılıyor...")
        
        # Hazırlık: İHA'ya birkaç setpoint göndererek OFFBOARD moduna hazırla
        prep_target = PositionTarget()
        prep_target.type_mask = PositionTarget.IGNORE_ALL_BUT_POSITION
        for _ in range(50):
            self.setpoint_pub.publish(prep_target)
            time.sleep(0.02)
        
        # OFFBOARD moduna geç
        if not self.set_mode("OFFBOARD"):
            return False

        # Arm et
        if not self.arm():
            return False

        # Hedef pozisyonu gönder
        target = PositionTarget()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                           PositionTarget.IGNORE_YAW_RATE
        target.position.z = float(altitude)
        
        # İrtifaya ulaşana kadar setpoint gönder
        start_time = self.get_clock().now()
        timeout = 30.0 # 30 saniye timeout

        while rclpy.ok():
            self.setpoint_pub.publish(target)
            rclpy.spin_once(self, timeout_sec=0.1)

            # Zaman aşımı kontrolü
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("Kalkış zaman aşımına uğradı.")
                return False

            # Yüksekliğe ulaşıldı mı kontrolü
            if abs(self.current_pose.pose.position.z - altitude) < 0.5:
                self.get_logger().info(f"Hedef irtifa olan {altitude} metreye ulaşıldı.")
                return True
            
            time.sleep(0.1)
        return False
