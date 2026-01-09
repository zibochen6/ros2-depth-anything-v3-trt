#!/usr/bin/env python3
# Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('depth_anything_v3')
    
    # Declare launch arguments
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='',
        description='Path to video file (required)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='onnx/DA3METRIC-LARGE.onnx',
        description='Path to ONNX model file'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Playback speed multiplier (1.0 = normal speed)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop video playback'
    )
    
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='1.0',
        description='Resolution scale factor (0.1-1.0, lower = faster inference)'
    )
    
    # Video depth node
    video_depth_node = Node(
        package='depth_anything_v3',
        executable='video_depth_node',
        name='video_depth_node',
        output='screen',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'model_path': LaunchConfiguration('model_path'),
            'frame_id': 'camera_link',
            'playback_speed': LaunchConfiguration('playback_speed'),
            'loop': LaunchConfiguration('loop'),
            'scale_factor': LaunchConfiguration('scale_factor'),
        }],
        remappings=[
            ('~/input/image', '/video/image'),
            ('~/output/depth_image', '/video/depth'),
            ('~/output/depth_colored', '/video/depth_colored'),
            ('~/output/point_cloud', '/video/point_cloud'),
            ('~/camera_info', '/video/camera_info'),
        ]
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(pkg_dir, 'config', 'video_depth.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Static transform publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='video_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    return LaunchDescription([
        video_path_arg,
        model_path_arg,
        playback_speed_arg,
        loop_arg,
        scale_factor_arg,
        video_depth_node,
        static_tf_node,
        rviz_node,
    ])
