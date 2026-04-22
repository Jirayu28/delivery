#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -----------------------
    # Args
    # -----------------------
    use_sim_time  = LaunchConfiguration('use_sim_time')
    autostart     = LaunchConfiguration('autostart')
    map_yaml      = LaunchConfiguration('map')
    params_file   = LaunchConfiguration('params_file')
    mode          = LaunchConfiguration('mode')  # localization | navigation

    default_map = PathJoinSubstitution([
        FindPackageShare('delivery_navigation'),
        'maps',
        'office_map.yaml'
    ])

    default_params = PathJoinSubstitution([
        FindPackageShare('delivery_navigation'),
        'config',
        'nav2_params.yaml'
    ])

    nav2_share = FindPackageShare('nav2_bringup')

    # -----------------------
    # Include: Localization (map_server + amcl)
    # -----------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'localization_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_yaml,
            'params_file': params_file,
        }.items(),
        # run in both modes
    )

    # -----------------------
    # Include: Navigation stack (planner/controller/bt/etc.)
    # -----------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            # ❌ อย่าส่ง use_docking เพราะ nav2_bringup ของคุณไม่มี arg นี้
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"]))
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),

        # ✅ โหมดให้เหมือนคุณรัน localization_launch โดยตรง
        DeclareLaunchArgument(
            'mode',
            default_value='navigation',
            description="localization: map+amcl only, navigation: full nav2 stack"
        ),

        localization_launch,
        navigation_launch,
    ])
