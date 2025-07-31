"""
Görev 2 – Otonom Kalkış + İniş Testi
------------------------------------
Aday, 'run_task task2' çağrısıyla:
  1) İHA’yı arm etmeli
  2) ~2 m yüksekliğe otonom kalkış
  3) Ardından güvenli şekilde iniş + disarm
Bu test, mavros üzerinden state ve pozisyon verilerini dinleyerek
sıralamanın başarıyla gerçekleştiğini doğrular.
"""

import os
import time
import unittest
import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.markers

# ------------ Parametreler ------------
TASK_NAME          = "task2"          # candidate_package arg
TARGET_ALT         = float(os.getenv("TARGET_ALT", 2.0))   # metre
ALT_TOLERANCE      = 0.3              # ± 0.3 m kabul
TIMEOUT_ARM        = 20.0             # s
TIMEOUT_TAKEOFF    = 30.0             # s
TIMEOUT_LAND       = 30.0             # s
# --------------------------------------


@launch_testing.markers.keep_alive
def generate_test_description():
    candidate_node_cmd = ExecuteProcess(
        cmd=["ros2", "run", "candidate_package", "run_task", TASK_NAME],
        output="screen",
        name="candidate_node",
    )

    return LaunchDescription([
        candidate_node_cmd,
        launch_testing.actions.ReadyToTest(),
    ])


class TakeoffLandTest(unittest.TestCase):

    # ---------- ROS 2 setup / teardown ---------- #
    def setUp(self):
        rclpy.init()
        self.node = Node("test_takeoff_land_monitor")

        self.state = None             # mavros State msg
        self.altitude = None          # Z in ENU frame (metre)

        self.node.create_subscription(
            State, "/mavros/state", self.state_cb, 10)

        self.node.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, 10)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    # -------------------------------------------- #

    # ---------- Callbacks ---------- #
    def state_cb(self, msg: State):
        self.state = msg

    def pose_cb(self, msg: PoseStamped):
        # ENU koordinat sisteminde Z yukarıdır
        self.altitude = msg.pose.position.z
    # -------------------------------- #

    # ---------- Helper ---------- #
    def wait_until(self, predicate, timeout, desc):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        self.node.get_logger().error(f"TIMEOUT: {desc}")
        return False
    # ----------------------------- #

    # ---------- Testin kendisi ---------- #
    def test_takeoff_and_land_sequence(self):
        """İHA arm → kalkış → iniş → disarm sırasını doğrular."""

        # 1) ARM
        start_armed = self.state.armed if self.state else False
        self.assertTrue(
            self.wait_until(
                lambda: self.state and self.state.armed and not start_armed,
                TIMEOUT_ARM,
                "Arming transition"),
           "İHA belirtilen sürede aktif olarak ARM olmadı.")

        # 2) OFFBOARD
        self.assertTrue(
            self.wait_until(
                lambda: self.state and self.state.mode == "OFFBOARD",
                10, "OFFBOARD mode"),
            "Offboard mode never set (candidate kodu çağırmadı?)")

        # 3) TAKE‑OFF (hedef irtifa)
        self.assertTrue(
            self.wait_until(
                lambda: self.altitude is not None and
                        self.altitude >= TARGET_ALT - ALT_TOLERANCE,
                TIMEOUT_TAKEOFF,
                f"Takeoff to {TARGET_ALT}m"),
            f"İHA {TARGET_ALT} m irtifaya ulaşamadı.")

        # 3) LAND + DISARM
        landed = self.wait_until(
            lambda: (self.altitude is not None and
                     self.altitude <= ALT_TOLERANCE) and
                    (self.state and not self.state.armed) and
                    (self.state.mode in ["AUTO.LAND", "AUTO.RTL", "MANUAL"]),
            TIMEOUT_LAND,
           "Landing + Disarm")
    # ------------------------------------- #
