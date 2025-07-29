import os
import unittest
import rclpy
import time
from rclpy.node import Node
from mavros_msgs.msg import State

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory

# Test edilecek görevi belirle. Bu, adayın main.py'sine argüman olarak gidecek.
TASK_NAME = 'task1'

@launch_testing.markers.keep_alive
def generate_test_description():
    # Artık simülasyonu kendimiz başlatacağımız için, bu fonksiyon
    # SADECE adayın kodunu çalıştıracak.

    candidate_node_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'candidate_package', 'run_task', TASK_NAME],
        output='screen',
        name='candidate_node'
    )

    return LaunchDescription([
        candidate_node_cmd,
        # Testlerin başlamaya hazır olduğunu bildir.
        launch_testing.actions.ReadyToTest(),
    ])

# Bu sınıf, test senaryolarını içerir.
class ArmDisarmTest(unittest.TestCase):
    def setUp(self):
        # Her testten önce bir ROS 2 düğümü başlat.
        rclpy.init()
        self.node = Node('test_arm_disarm_monitor')
        self.state = None
        # İHA'nın durumunu dinlemek için bir subscriber oluştur.
        self.state_sub = self.node.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        
    def tearDown(self):
        # Her testten sonra düğümü kapat.
        self.node.destroy_node()
        rclpy.shutdown()

    def state_callback(self, msg):
        # Gelen her durum mesajını kaydet.
        self.state = msg

    def test_arm_disarm_sequence(self):
        """Adayın kodunun İHA'yı doğru sırada arm edip sonra disarm ettiğini doğrular."""
        # Testin başarılı olması için geçmesi gereken adımlar:
        # 1. İHA'nın 'armed' durumu 'True' olmalı.
        # 2. Ardından 'armed' durumu tekrar 'False' olmalı.
        
        timeout_arm = 20.0  # Adayın kodu 20 saniye içinde arm etmeli
        timeout_disarm = 15.0 # Arm ettikten sonra 15 saniye içinde disarm etmeli

        # Adım 1: Arm durumunu bekle
        self.node.get_logger().info("İHA'nın 'ARMED' durumuna geçmesi bekleniyor...")
        start_time = time.time()
        armed_success = False
        while time.time() - start_time < timeout_arm:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.state and self.state.armed:
                self.node.get_logger().info("BAŞARILI: İHA 'ARMED' durumuna geçti!")
                armed_success = True
                break
        
        self.assertTrue(armed_success, "ZAMAN AŞIMI: İHA 'ARMED' durumuna geçmedi.")

        # Adım 2: Disarm durumunu bekle
        self.node.get_logger().info("İHA'nın 'DISARMED' durumuna geçmesi bekleniyor...")
        start_time = time.time()
        disarmed_success = False
        while time.time() - start_time < timeout_disarm:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.state and not self.state.armed:
                self.node.get_logger().info("BAŞARILI: İHA 'DISARMED' durumuna geçti!")
                disarmed_success = True
                break
        
        self.assertTrue(disarmed_success, "ZAMAN AŞIMI: İHA 'DISARMED' durumuna geçmedi.")
