import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # NOT: Bu launch dosyası, PX4-Autopilot'un '~/PX4-Autopilot' dizininde
    # kurulu olduğunu varsayar. GitHub Actions bu şekilde çalışacaktır.
    # Eğer lokalde farklı bir yere kurduysanız, bu yolu değiştirmeniz gerekebilir.
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
        
    px4_sitl_cmd = os.path.join(px4_dir, "build/px4_sitl_default/bin/px4")
    
    # Ortam değişkenlerini PX4 SITL için ayarla
    # Hız faktörü testlerin daha hızlı bitmesini sağlar.
    os.environ['PX4_SIM_MODEL'] = 'iris'
    os.environ['PX4_SIM_SPEED_FACTOR'] = '10'

    return LaunchDescription([
        # Gazebo Simülatörünü Başlat
        ExecuteProcess(
            cmd=[
                'gazebo', 
                '--verbose', 
                '-s', 'libgazebo_ros_init.so', 
                '-s', 'libgazebo_ros_factory.so',
            ],
            shell=True,
            output='screen'
        ),
        
        # PX4 SITL'ı Başlat
        ExecuteProcess(
            cmd=[
                px4_sitl_cmd, 
                os.path.join(px4_dir, "ROMFS/px4fmu_common"), 
                "-s", "etc/init.d-posix/rcS",
            ],
            shell=True,
            output='screen'
        ),
        
        # ROS 2 ve PX4 arasında köprü kuran Micro-RTPS Agent'ı başlat
        ExecuteProcess(
            cmd=['micrortps_agent', '-t', 'UDP'],
            shell=True,
            output='screen'
        ),
    ])
