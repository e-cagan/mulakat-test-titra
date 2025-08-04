"""
Görev 1 – Arm ➜ Disarm Kenar Testi.

PX4 SITL + MAVROS ➜ candidate node ‘task1’ çağrısı
1) ARM    2) 5 s    3) DISARM  
Kenarlar ≤ 20 s içinde gözlenmeli.
"""

import os, time, unittest
from pathlib import Path
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.markers import keep_alive, launch_test
import launch_testing_ros
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

ARM_TIMEOUT = float(os.getenv("ARM_TIMEOUT", "20.0"))
this_dir = Path(__file__).parent

@keep_alive
@launch_test
def generate_test_description() -> LaunchDescription:
    try:
        launch_path = Path(
            get_package_share_directory("test_package"),
            "launch",
            "px4_sim.launch.py",
        )
    except PackageNotFoundError:
        launch_path = this_dir.parent / "launch" / "px4_sim.launch.py"
    sim_launch = PythonLaunchDescriptionSource(str(launch_path))
    return LaunchDescription([
        IncludeLaunchDescription(sim_launch),
        ExecuteProcess(
            cmd=["ros2", "run", "candidate_package", "run_task", "task1"],
            output="screen",
        ),
        launch_testing_ros.actions.ReadyToTest(),
    ])


class ArmDisarmTest(unittest.TestCase):
    """ARM → DISARM kenarlarını doğrular."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_arm_disarm_monitor")
        cls.armed_edge = False
        cls.disarmed_edge = False
        cls.node.create_subscription(State, "/mavros/state", cls._cb, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _cb(cls, msg: State):
        if msg.armed and not cls.armed_edge:
            cls.armed_edge = True
        elif cls.armed_edge and not msg.armed:
            cls.disarmed_edge = True

    def _wait(self, pred, timeout):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if pred():
                return True
        return False

    def test_arm_disarm_edge(self):
        ok = self._wait(lambda: self.armed_edge and self.disarmed_edge, ARM_TIMEOUT)
        self.assertTrue(ok, "ARM veya DISARM kenarı zaman aşımına uğradı")
