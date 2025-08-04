import os
import time
import unittest
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.markers import keep_alive, launch_test
import launch_testing_ros
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from pathlib import Path

# parameters
TARGET_ALT = float(os.getenv("TARGET_ALT", "2.0"))
TOLERANCE = 0.3
TIMEOUT_ARM = 20.0
TIMEOUT_OFF = 10.0
TIMEOUT_TAKE = 30.0
TIMEOUT_LAND = 30.0

this_dir = Path(__file__).parent

@keep_alive
@launch_test
def generate_test_description() -> LaunchDescription:
    # find your px4_sim.launch.py exactly the same as in task1
    try:
        launch_path = Path(
            get_package_share_directory("test_package"),
            "launch",
            "px4_sim.launch.py",
        )
    except PackageNotFoundError:
        launch_path = this_dir.parent / "launch" / "px4_sim.launch.py"
    
    sim_launch = PythonLaunchDescriptionSource(str(launch_path))
    
    # this is the only difference vs task1:
    candidate = ExecuteProcess(
        cmd=["ros2", "run", "candidate_package", "run_task", "task2"],
        output="screen",
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(sim_launch),
        candidate,
        launch_testing_ros.actions.ReadyToTest(),
    ])

class TakeoffLandTest(unittest.TestCase):
    """Verify: ARM → OFFBOARD → rise above TARGET_ALT → LAND + DISARM."""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("takeoff_land_monitor")
        cls.state = None
        cls.alt = None
        cls.node.create_subscription(State, "/mavros/state", cls._state_cb, 10)
        cls.node.create_subscription(PoseStamped, "/mavros/local_position/pose", cls._pose_cb, 10)
    
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def _state_cb(cls, msg: State):  # Düzeltildi: asterisk kaldırıldı
        cls.state = msg
    
    @classmethod
    def _pose_cb(cls, msg: PoseStamped):  # Düzeltildi: asterisk kaldırıldı
        cls.alt = msg.pose.position.z
    
    def _wait(self, predicate, timeout, desc):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        self.node.get_logger().error(f"TIMEOUT {desc}")
        return False
    
    def test_sequence(self):
        # 1) ARM
        self.assertTrue(
            self._wait(lambda: self.state and self.state.armed, TIMEOUT_ARM, "arming"),
            "Never armed"
        )
        
        # 2) OFFBOARD
        self.assertTrue(
            self._wait(lambda: self.state and self.state.mode == "OFFBOARD", TIMEOUT_OFF, "offboard"),
            "Never entered OFFBOARD"
        )
        
        # 3) TAKEOFF
        self.assertTrue(
            self._wait(
                lambda: self.alt is not None and self.alt >= TARGET_ALT - TOLERANCE,
                TIMEOUT_TAKE,
                "takeoff"
            ),
            f"Did not reach {TARGET_ALT}m"
        )
        
        # 4) LAND + DISARM
        self.assertTrue(
            self._wait(
                lambda: (self.alt is not None and self.alt <= TOLERANCE)
                and self.state and not self.state.armed,
                TIMEOUT_LAND,
                "landing+disarm"
            ),
            "Did not land & disarm"
        )