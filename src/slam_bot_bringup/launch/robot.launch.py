#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package directories
    pkg_slam_bot_bringup = FindPackageShare('slam_bot_bringup')
    pkg_slam_bot_description = FindPackageShare('slam_bot_description')
    pkg_slam_bot_control = FindPackageShare('slam_bot_control')
    pkg_realsense2_camera = FindPackageShare('realsense2_camera')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_localization = LaunchConfiguration('use_robot_localization')
    use_camera = LaunchConfiguration('use_camera')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )
    
    declare_use_robot_localization = DeclareLaunchArgument(
        'use_robot_localization',
        default_value='true',
        description='Use robot_localization for odometry fusion'
    )
    
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch RealSense camera if true'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PathJoinSubstitution([
                pkg_slam_bot_description,
                'urdf',
                'slam_bot.urdf.xacro'
            ])
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )
    
    # Robot localization (EKF)
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_robot_localization),
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 50.0,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/odom',
            'odom1': '/wheel_odom',
            'imu0': '/imu/data'
        }]
    )
    
    # RealSense camera
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_realsense2_camera,
                'launch',
                'rs_launch.py'
            ])
        ]),
        condition=IfCondition(use_camera),
        launch_arguments={
            'camera_name': 'camera',
            'serial_no': '',
            'usb_port_id': '',
            'device_type': '',
            'enable_rgb': 'true',
            'enable_depth': 'true',
            'enable_imu': 'true',
            'enable_sync': 'true',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
            'rgb_width': '640',
            'rgb_height': '480',
            'rgb_fps': '30',
            'enable_pointcloud': 'true',
            'pointcloud_texture_stream': 'RS2_STREAM_COLOR',
            'pointcloud_texture_index': '0'
        }.items()
    )
    
    # Motor controller node
    motor_controller = Node(
        package='slam_bot_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'left_motor_address': 128,
            'right_motor_address': 129,
            'wheel_diameter': 0.065,
            'wheel_separation': 0.25,
            'encoder_ticks_per_rev': 1440
        }]
    )
    
    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', PathJoinSubstitution([
            pkg_slam_bot_bringup,
            'config',
            'robot.rviz'
        ])]
    )
    
    # TF static broadcaster for base_link to camera_link
    tf_static_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        output='screen',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_robot_localization,
        declare_use_camera,
        
        # Nodes
        robot_state_publisher,
        joint_state_publisher,
        robot_localization,
        motor_controller,
        tf_static_broadcaster,
        rviz2,
        
        # Launch files
        realsense_camera
    ])
