from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    my_share = get_package_share_directory('delivery_bringup')
    my_params = os.path.join(my_share, 'config', 'ydlidar_x3.yaml')

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',          # ✅ เพิ่มบรรทัดนี้ (Jazzy ต้องมี)
        output='screen',
        emulate_tty=True,       # ✅ แนะนำให้ใส่เหมือนของเดิม จะได้ log สวย
        parameters=[my_params]
    )

    return LaunchDescription([driver_node])
