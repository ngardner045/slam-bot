#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_slam_bot_navigation = FindPackageShare('slam_bot_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # RTAB-Map SLAM node
    rtabmap_slam = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'subscribe_scan': True,
            'subscribe_odom': True,
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'publish_tf': True,
            'database_path': PathJoinSubstitution([pkg_slam_bot_navigation, 'maps', 'rtabmap.db']),
            'queue_size': 10,
            'approx_sync': True,
            'wait_for_transform': 0.1
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/wheel_odom'),
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/camera_info', '/camera/depth/camera_info')
        ]
    )
    
    # Point cloud to laser scan converter
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_height': 0.1,
            'max_height': 1.0,
            'angle_min': -1.5708,
            'angle_max': 1.5708,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 10.0,
            'target_frame': 'base_link'
        }],
        remappings=[
            ('cloud_in', '/camera/depth/points'),
            ('scan', '/scan')
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        rtabmap_slam,
        pointcloud_to_laserscan
    ])
