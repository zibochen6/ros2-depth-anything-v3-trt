#!/usr/bin/env python3
# Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Multi-camera launch file for GMSL cameras with depth estimation
# This launch file supports launching multiple GMSL camera nodes simultaneously

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_camera_config(context):
    """Load camera configuration from YAML file or use defaults"""
    config_file = LaunchConfiguration('config_file').perform(context)
    
    if config_file and os.path.exists(config_file):
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            return config.get('cameras', [])
    
    # Default configuration for 4 GMSL cameras
    return [
        {
            'name': 'camera_0',
            'device_path': '/dev/video0',
            'frame_id': 'camera_0_link',
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },
        {
            'name': 'camera_1',
            'device_path': '/dev/video1',
            'frame_id': 'camera_1_link',
            'position': {'x': 0.5, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },
        {
            'name': 'camera_2',
            'device_path': '/dev/video2',
            'frame_id': 'camera_2_link',
            'position': {'x': -0.5, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },
        {
            'name': 'camera_3',
            'device_path': '/dev/video3',
            'frame_id': 'camera_3_link',
            'position': {'x': 0.0, 'y': 0.5, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },
    ]


def launch_setup(context, *args, **kwargs):
    """Generate launch description with multiple camera nodes"""
    
    # Get common parameters
    model_path = LaunchConfiguration('model_path').perform(context)
    publish_rate = LaunchConfiguration('publish_rate').perform(context)
    camera_width = LaunchConfiguration('camera_width').perform(context)
    camera_height = LaunchConfiguration('camera_height').perform(context)
    framerate = LaunchConfiguration('framerate').perform(context)
    format_param = LaunchConfiguration('format').perform(context)
    sensor_mode = LaunchConfiguration('sensor_mode').perform(context)
    
    # Load camera configurations
    cameras = load_camera_config(context)
    
    nodes = []
    
    # Create nodes for each camera
    for camera in cameras:
        camera_name = camera['name']
        device_path = camera['device_path']
        frame_id = camera['frame_id']
        pos = camera['position']
        orient = camera['orientation']
        
        # Camera depth node
        camera_node = Node(
            package='depth_anything_v3',
            executable='camera_depth_node',
            name=camera_name,
            namespace=camera_name,
            output='screen',
            parameters=[{
                'camera_type': 'gmsl',
                'device_path': device_path,
                'model_path': model_path,
                'frame_id': frame_id,
                'publish_rate': float(publish_rate),
                'camera_width': int(camera_width),
                'camera_height': int(camera_height),
                'framerate': int(framerate),
                'format': format_param,
                'sensor_mode': int(sensor_mode),
            }],
            remappings=[
                ('~/input/image', f'/{camera_name}/image'),
                ('~/output/depth_image', f'/{camera_name}/depth'),
                ('~/output/depth_colored', f'/{camera_name}/depth_colored'),
                ('~/output/point_cloud', f'/{camera_name}/point_cloud'),
                ('~/camera_info', f'/{camera_name}/camera_info'),
            ]
        )
        nodes.append(camera_node)
        
        # Static transform publisher for this camera
        # Convert roll, pitch, yaw to quaternion (simplified - assumes small angles)
        import math
        roll = orient['roll']
        pitch = orient['pitch']
        yaw = orient['yaw']
        
        # Simplified quaternion conversion (for small angles)
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{camera_name}_tf_publisher',
            arguments=[
                str(pos['x']), str(pos['y']), str(pos['z']),
                str(qx), str(qy), str(qz), str(qw),
                'base_link', frame_id
            ]
        )
        nodes.append(static_tf_node)
    
    return nodes


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to camera configuration YAML file'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='onnx/DA3METRIC-LARGE.onnx',
        description='Path to ONNX model file'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='1920',
        description='Camera width'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='1536',
        description='Camera height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )
    
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='YUYV',
        description='Video format (YUYV for GMSL cameras)'
    )
    
    sensor_mode_arg = DeclareLaunchArgument(
        'sensor_mode',
        default_value='0',
        description='Sensor mode'
    )
    
    return LaunchDescription([
        config_file_arg,
        model_path_arg,
        publish_rate_arg,
        camera_width_arg,
        camera_height_arg,
        framerate_arg,
        format_arg,
        sensor_mode_arg,
        OpaqueFunction(function=launch_setup)
    ])
