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
    
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='standard',
        description='Camera type (standard or gmsl)'
    )
    
    downsample_factor_arg = DeclareLaunchArgument(
        'downsample_factor',
        default_value='1',
        description='Downsample factor for faster inference (1=no downsampling, 2=half resolution, etc.)'
    )
    
    # Camera calibration parameters
    camera_info_file_arg = DeclareLaunchArgument(
        'camera_info_file',
        default_value='',
        description='Path to camera calibration YAML file (optional)'
    )
    
    use_calibration_arg = DeclareLaunchArgument(
        'use_calibration',
        default_value='false',
        description='Use calibrated camera parameters'
    )
    
    fx_arg = DeclareLaunchArgument('fx', default_value='824.147361', description='Focal length X (fisheye)')
    fy_arg = DeclareLaunchArgument('fy', default_value='823.660879', description='Focal length Y (fisheye)')
    cx_arg = DeclareLaunchArgument('cx', default_value='958.275200', description='Principal point X (fisheye)')
    cy_arg = DeclareLaunchArgument('cy', default_value='767.389372', description='Principal point Y (fisheye)')
    k1_arg = DeclareLaunchArgument('k1', default_value='1.486308', description='Fisheye distortion D[0]')
    k2_arg = DeclareLaunchArgument('k2', default_value='-13.386609', description='Fisheye distortion D[1]')
    p1_arg = DeclareLaunchArgument('p1', default_value='21.409334', description='Fisheye distortion D[2]')
    p2_arg = DeclareLaunchArgument('p2', default_value='3.817858', description='Fisheye distortion D[3]')
    k3_arg = DeclareLaunchArgument('k3', default_value='0.0', description='Distortion k3 (unused for fisheye)')
    
    # Camera depth node
    camera_depth_node = Node(
        package='depth_anything_v3',
        executable='camera_depth_node',
        name='camera_depth_node',
        output='screen',
        parameters=[{
            'camera_type': LaunchConfiguration('camera_type'),
            'camera_id': LaunchConfiguration('camera_id'),
            'model_path': LaunchConfiguration('model_path'),
            'frame_id': 'camera_link',
            'publish_rate': LaunchConfiguration('publish_rate'),
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'downsample_factor': LaunchConfiguration('downsample_factor'),
            'camera_info_file': LaunchConfiguration('camera_info_file'),
            'use_calibration': LaunchConfiguration('use_calibration'),
            'fx': LaunchConfiguration('fx'),
            'fy': LaunchConfiguration('fy'),
            'cx': LaunchConfiguration('cx'),
            'cy': LaunchConfiguration('cy'),
            'k1': LaunchConfiguration('k1'),
            'k2': LaunchConfiguration('k2'),
            'p1': LaunchConfiguration('p1'),
            'p2': LaunchConfiguration('p2'),
            'k3': LaunchConfiguration('k3'),
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
        camera_type_arg,
        camera_id_arg,
        model_path_arg,
        publish_rate_arg,
        camera_width_arg,
        camera_height_arg,
        downsample_factor_arg,
        camera_info_file_arg,
        use_calibration_arg,
        fx_arg, fy_arg, cx_arg, cy_arg,
        k1_arg, k2_arg, p1_arg, p2_arg, k3_arg,
        camera_depth_node,
        static_tf_node,
        rviz_node,
    ])
