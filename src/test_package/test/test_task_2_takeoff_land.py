"""
Görev 2 – Otonom Kalkış + İniş Testi.

Aday `ros2 run candidate_package run_task task2` çağrısında:
1) PX4 → ARM  
2) ~2 m yüksekliğe otonom kalkış (OFFBOARD)  
3) Güvenli şekilde iniş + DISARM  
Bu test, MAVROS üzerinden /mavros/state ve
/mavros/local_position/pose topic’lerini dinleyerek sıralamanın
başarıyla gerçekleştiğini doğrular.
"""

from __future__ import annotations

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
from launch_testing.markers import keep_alive
from pathlib import Path
import launch_testing_ros

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

this_dir = Path(__file__).parent

# -------- Parametreler --------
TASK_NAME = "task2"
TARGET_ALT = float(os.getenv("TARGET_ALT", "2.0"))    # m
ALT_TOLERANCE = 0.3                                   # m
TIMEOUT_ARM = 20.0                                    # s
TIMEOUT_TAKEOFF = 30.0                                # s
TIMEOUT_LAND = 30.0                                   # s
# ------------------------------


@keep_alive
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

    candidate = ExecuteProcess(
        cmd=["ros2", "run", "candidate_package", "run_task", "task2"],
        output="screen",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(sim_launch),
            candidate,
            launch_testing_ros.actions.ReadyToTest(),
        ]
    )


class TakeoffLandTest(unittest.TestCase):
    """ARM → kalkış → iniş → DISARM sırasını doğrular."""

    # ---------- ROS 2 setup / teardown ---------- #
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node: Node = Node("test_takeoff_land_monitor")
        cls.state: State | None = None
        cls.altitude: float | None = None

        cls.node.create_subscription(State, "/mavros/state", cls._state_cb, 10)
        cls.node.create_subscription(
            PoseStamped, "/mavros/local_position/pose", cls._pose_cb, 10
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()
    # -------------------------------------------- #

    # ---------- Callbacks ---------- #
    @classmethod
    def _state_cb(cls, msg: State) -> None:
        cls.state = msg

    @classmethod
    def _pose_cb(cls, msg: PoseStamped) -> None:
        # ENU frame → Z yukarı
        cls.altitude = msg.pose.position.z
    # -------------------------------- #

    # ---------- Helper ---------- #
    def _wait(self, predicate, timeout: float, desc: str) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        self.node.get_logger().error(f"TIMEOUT: {desc}")
        return False
    # ----------------------------- #

    # ---------- Test ---------- #
    def test_takeoff_and_land_sequence(self) -> None:
        """ARM → OFFBOARD → ≥ TARGET_ALT → LAND + DISARM."""
        # 1) ARM
        self.assertTrue(
            self._wait(
                lambda: self.state and self.state.armed,
                TIMEOUT_ARM,
                "Arming",
            ),
            "İHA belirtilen sürede ARM olmadı.",
        )

        # 2) OFFBOARD moda geçiş
        self.assertTrue(
            self._wait(
                lambda: self.state and self.state.mode == "OFFBOARD",
                10.0,
                "OFFBOARD mode",
            ),
            "Offboard mode hiç ayarlanmadı.",
        )

        # 3) TAKE-OFF (hedef irtifa)
        self.assertTrue(
            self._wait(
                lambda: self.altitude is not None
                and self.altitude >= TARGET_ALT - ALT_TOLERANCE,
                TIMEOUT_TAKEOFF,
                "Take-off",
            ),
            f"İHA {TARGET_ALT} m irtifaya ulaşamadı.",
        )

        # 4) LAND + DISARM
        self.assertTrue(
            self._wait(
                lambda: self.altitude is not None
                and self.altitude <= ALT_TOLERANCE
                and self.state
                and not self.state.armed,
                TIMEOUT_LAND,
                "Landing + Disarm",
            ),
            "İHA başarıyla iniş / disarm yapamadı.",
        )
    # -------------------------------- #
