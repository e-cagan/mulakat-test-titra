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
import launch_testing.markers
from ament_index_python.packages import get_package_share_directory

raise RuntimeError("Test dosyası gerçekten güncellendi mi?")

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
        rclpy.init()
        self.node = Node("test_arm_disarm_monitor")
        self.state = None
        self.node.create_subscription(State, "/mavros/state",
                                      lambda m: setattr(self, "state", m), 10)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    # helper
    def wait_until(self, pred, timeout, desc):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if pred():
                return True
        self.node.get_logger().error(f"TIMEOUT: {desc}")
        return False

    def test_arm_disarm_sequence(self):
        start_armed = self.state.armed if self.state else False

        self.assertTrue(
            self.wait_until(
                lambda: self.state and self.state.armed and not start_armed,
                20.0, "Arming transition"),
            "İHA aktif olarak ARM olmadı."
        )

        self.assertTrue(
            self.wait_until(
                lambda: self.state and not self.state.armed,
                15.0, "Disarming transition"),
            "İHA belirtilen sürede DISARM olmadı."
        )
