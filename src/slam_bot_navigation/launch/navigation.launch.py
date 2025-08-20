#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_slam_bot_navigation = FindPackageShare('slam_bot_navigation')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )
    
    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': PathJoinSubstitution([pkg_slam_bot_navigation, 'maps', 'map.yaml']),
            'params_file': PathJoinSubstitution([pkg_slam_bot_navigation, 'config', 'nav2_params.yaml'])
        }.items()
    )
    
    # RViz2 for navigation
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
        ]),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': PathJoinSubstitution([pkg_slam_bot_navigation, 'config', 'nav2_default_view.rviz'])
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        nav2_bringup,
        rviz2
    ])
