from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ydlidar driver launch (ของเขา)
    drv_share = get_package_share_directory('ydlidar_ros2_driver')
    drv_launch = os.path.join(drv_share, 'launch', 'ydlidar_launch.py')

    # params ของคุณ (อยู่ใน bringup)
    my_share = get_package_share_directory('delivery_bringup')
    my_params = os.path.join(my_share, 'config', 'ydlidar_x3.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drv_launch),
            launch_arguments={'params_file': my_params}.items()
        ),
    ])
