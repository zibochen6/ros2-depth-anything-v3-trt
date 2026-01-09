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
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='onnx/DA3METRIC-LARGE.onnx',
        description='Path to ONNX model file'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='5.0',
        description='Publishing rate in Hz'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera width (320/640/1280)'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera height (240/480/720)'
    )
    
    # Camera depth node
    camera_depth_node = Node(
        package='depth_anything_v3',
        executable='camera_depth_node',
        name='camera_depth_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'model_path': LaunchConfiguration('model_path'),
            'frame_id': 'camera_link',
            'publish_rate': LaunchConfiguration('publish_rate'),
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
        }],
        remappings=[
            ('~/input/image', '/camera/image'),
            ('~/output/depth_image', '/camera/depth'),
            ('~/output/depth_colored', '/camera/depth_colored'),
            ('~/output/point_cloud', '/camera/point_cloud'),
            ('~/camera_info', '/camera/camera_info'),
        ]
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(pkg_dir, 'config', 'camera_depth.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Static transform publisher (camera to base_link)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    return LaunchDescription([
        camera_id_arg,
        model_path_arg,
        publish_rate_arg,
        camera_width_arg,
        camera_height_arg,
        camera_depth_node,
        static_tf_node,
        rviz_node,
    ])
