from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    use_gui  = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('delivery_description'), 'urdf', 'delivery.urdf.xacro']
    )

    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # GUI version (slider)
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description}],
        output='screen',
        condition=IfCondition(use_gui)
    )

    # Non-GUI version (publish joint_states at 0.0)
    joint_state_no_gui = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
        condition=UnlessCondition(use_gui)
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='false', # เป็นการเปิด joint_state_publisher_gui โดยค่าเริ่มต้น เพื่อดูการหมุ่นของข้อต่อ
            description='Start joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false', # เป็นการเปิด RViz2 โดยค่าเริ่มต้น เพื่อดูโมเดลหุ่นยนต์
            description='Start RViz2'
        ),
        robot_state_publisher,
        joint_state_gui,
        joint_state_no_gui,
        rviz2
    ])
