from launch import LaunchDescription, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """PX4 SITL + MAVROS köprüsünü başlatır (UDP :14540)."""
    px4 = ExecuteProcess(
        cmd=[
            "/root/PX4-Autopilot/build/px4_sitl_default/bin/px4",
            "/root/PX4-Autopilot/ROMFS/px4fmu_common",
            "-i",
            "0",
        ],
        output="screen",
    )

    mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("mavros") + "/launch/mavros.launch.py"
        ),
        launch_arguments={
            "fcu_url": "udp://:14540@localhost:14557",
            "respawn": "false",
        }.items(),
    )

    return LaunchDescription([px4, mavros])
