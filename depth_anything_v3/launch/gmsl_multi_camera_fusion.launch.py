#!/usr/bin/env python3
# Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Complete multi-camera launch file with point cloud fusion and RViz visualization

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml
import math


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
            'position': {'x': 0.5, 'y': 0.0, 'z': 0.2},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },
        {
            'name': 'camera_1',
            'device_path': '/dev/video1',
            'frame_id': 'camera_1_link',
            'position': {'x': 0.0, 'y': -0.5, 'z': 0.2},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': -1.5708}
        },
        {
            'name': 'camera_2',
            'device_path': '/dev/video2',
            'frame_id': 'camera_2_link',
            'position': {'x': 0.0, 'y': 0.5, 'z': 0.2},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 1.5708}
        },
        {
            'name': 'camera_3',
            'device_path': '/dev/video3',
            'frame_id': 'camera_3_link',
            'position': {'x': -0.5, 'y': 0.0, 'z': 0.2},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 3.14159}
        },
    ]


def launch_setup(context, *args, **kwargs):
    """Generate launch description with multiple camera nodes, fusion, and RViz"""
    
    pkg_dir = get_package_share_directory('depth_anything_v3')
    
    # Get common parameters
    camera_type = LaunchConfiguration('camera_type').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)
    publish_rate = LaunchConfiguration('publish_rate').perform(context)
    camera_width = LaunchConfiguration('camera_width').perform(context)
    camera_height = LaunchConfiguration('camera_height').perform(context)
    framerate = LaunchConfiguration('framerate').perform(context)
    format_param = LaunchConfiguration('format').perform(context)
    sensor_mode = LaunchConfiguration('sensor_mode').perform(context)
    fusion_rate = LaunchConfiguration('fusion_rate').perform(context)
    downsample_factor = LaunchConfiguration('downsample_factor').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context).lower() == 'true'
    
    # Get calibration parameters
    use_calibration = LaunchConfiguration('use_calibration').perform(context)
    fx = LaunchConfiguration('fx').perform(context)
    fy = LaunchConfiguration('fy').perform(context)
    cx = LaunchConfiguration('cx').perform(context)
    cy = LaunchConfiguration('cy').perform(context)
    k1 = LaunchConfiguration('k1').perform(context)
    k2 = LaunchConfiguration('k2').perform(context)
    p1 = LaunchConfiguration('p1').perform(context)
    p2 = LaunchConfiguration('p2').perform(context)
    k3 = LaunchConfiguration('k3').perform(context)
    
    # Load camera configurations
    cameras = load_camera_config(context)
    
    nodes = []
    point_cloud_topics = []
    
    # Create nodes for each camera
    for camera in cameras:
        camera_name = camera['name']
        device_path = camera['device_path']
        frame_id = camera['frame_id']
        pos = camera['position']
        orient = camera['orientation']
        
        # Camera depth node
        # For standard camera type, use device index (0, 1, 2, 3) instead of /dev/video path
        device_param = device_path
        camera_id_param = 0
        if camera_type == 'standard':
            # Extract device number from /dev/videoX
            if '/dev/video' in device_path:
                camera_id_param = int(device_path.replace('/dev/video', ''))
        
        camera_node = Node(
            package='depth_anything_v3',
            executable='camera_depth_node',
            name=camera_name,
            namespace=camera_name,
            output='screen',
            parameters=[{
                'camera_type': camera_type,
                'device_path': device_param if camera_type == 'gmsl' else '',
                'camera_id': camera_id_param if camera_type == 'standard' else 0,
                'model_path': model_path,
                'frame_id': frame_id,
                'publish_rate': float(publish_rate),
                'camera_width': int(camera_width) if camera_type == 'gmsl' else -1,
                'camera_height': int(camera_height) if camera_type == 'gmsl' else -1,
                'framerate': int(framerate) if camera_type == 'gmsl' else -1,
                'format': format_param if camera_type == 'gmsl' else '',
                'sensor_mode': int(sensor_mode) if camera_type == 'gmsl' else -1,
                'downsample_factor': int(downsample_factor),
                'use_calibration': use_calibration.lower() == 'true',
                'fx': float(fx),
                'fy': float(fy),
                'cx': float(cx),
                'cy': float(cy),
                'k1': float(k1),
                'k2': float(k2),
                'p1': float(p1),
                'p2': float(p2),
                'k3': float(k3),
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
        
        # Add to point cloud topics list
        point_cloud_topics.append(f'/{camera_name}/point_cloud')
        
        # Static transform publisher for this camera
        roll = orient['roll']
        pitch = orient['pitch']
        yaw = orient['yaw']
        
        # Convert roll, pitch, yaw to quaternion
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
    
    # Point cloud fusion node
    fusion_node = Node(
        package='depth_anything_v3',
        executable='point_cloud_fusion_node',
        name='point_cloud_fusion',
        output='screen',
        parameters=[{
            'input_topics': point_cloud_topics,
            'target_frame': 'base_link',
            'fusion_rate': float(fusion_rate),
            'timeout': 5.0,  # Increased timeout for slow depth estimation (was 0.5)
        }]
    )
    nodes.append(fusion_node)
    
    # RViz2 node (optional)
    if launch_rviz:
        # Choose RViz config based on number of cameras
        num_cameras = len(cameras)
        if num_cameras == 2:
            rviz_config_file = os.path.join(pkg_dir, 'config', '2camera_fusion.rviz')
        else:
            rviz_config_file = os.path.join(pkg_dir, 'config', 'multi_camera_fusion.rviz')
        
        # Use default config if specific config doesn't exist
        if not os.path.exists(rviz_config_file):
            rviz_config_file = os.path.join(pkg_dir, 'config', 'camera_depth.rviz')
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            output='screen'
        )
        nodes.append(rviz_node)
    
    return nodes


def generate_launch_description():
    # Declare launch arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='gmsl',
        description='Camera type: "standard" or "gmsl"'
    )
    
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
        description='Camera publishing rate in Hz'
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
    
    fusion_rate_arg = DeclareLaunchArgument(
        'fusion_rate',
        default_value='10.0',
        description='Point cloud fusion rate in Hz'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    downsample_factor_arg = DeclareLaunchArgument(
        'downsample_factor',
        default_value='1',
        description='Downsample factor for faster inference (1=no downsampling, 2=half resolution, etc.)'
    )
    
    # Camera calibration parameters (fisheye lens)
    use_calibration_arg = DeclareLaunchArgument(
        'use_calibration',
        default_value='false',
        description='Use calibrated fisheye camera parameters'
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
    
    return LaunchDescription([
        camera_type_arg,
        config_file_arg,
        model_path_arg,
        publish_rate_arg,
        camera_width_arg,
        camera_height_arg,
        framerate_arg,
        format_arg,
        sensor_mode_arg,
        fusion_rate_arg,
        launch_rviz_arg,
        downsample_factor_arg,
        use_calibration_arg,
        fx_arg, fy_arg, cx_arg, cy_arg,
        k1_arg, k2_arg, p1_arg, p2_arg, k3_arg,
        OpaqueFunction(function=launch_setup)
    ])
