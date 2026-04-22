from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz     = LaunchConfiguration('use_rviz')
    use_lidar    = LaunchConfiguration('use_lidar')
    use_esp32    = LaunchConfiguration('use_esp32')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # -------- paths --------
    startup_share = FindPackageShare('delivery_startup')

    # ✅ EKF ใช้ของ delivery_localization
    ekf_yaml = PathJoinSubstitution([
        FindPackageShare('delivery_localization'),
        'config',
        'ekf.yaml'
    ])

    # ESP32 params ยังอยู่ใน delivery_startup
    esp32_yaml = PathJoinSubstitution([startup_share, 'config', 'esp32.yaml'])

    # RViz config
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('delivery_description'), 'rviz', 'display.rviz'
    ])

    # URDF/xacro จาก delivery_description
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('delivery_description'), 'urdf', 'delivery.urdf.xacro']
    )
    robot_description = Command(['xacro ', xacro_file])

    # -------- robot_state_publisher --------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
    )

    # ✅ ถ้าไม่ได้ใช้ ESP32 (ไม่มี driver ส่ง joint_states)
    # ให้เปิด joint_state_publisher_gui เพื่อให้ล้อ/ข้อต่อไม่หายใน RViz
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(PythonExpression(['not ', use_esp32])),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # -------- EKF --------
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
    )

    # -------- ESP32 bridge (optional) --------
    # สำคัญ: ใช้ EKF แล้วให้ esp32 publish_tf:=false (กัน TF ซ้อน)
    esp32 = Node(
        package='delivery_controller',
        executable='esp32_bridge',
        name='esp32_bridge',
        output='screen',
        parameters=[esp32_yaml, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_esp32),
    )

    # -------- LiDAR include (optional) --------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('delivery_bringup'),
                'launch',
                'ydlidar_x3.launch.py'
            ])
        ),
        condition=IfCondition(use_lidar),
    )

    # -------- RViz (optional) --------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='false'),
        DeclareLaunchArgument('use_esp32', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        rsp,
        jsp_gui,      # ✅ เพิ่มบรรทัดนี้
        ekf,
        esp32,
        lidar_launch,
        rviz,
    ])
