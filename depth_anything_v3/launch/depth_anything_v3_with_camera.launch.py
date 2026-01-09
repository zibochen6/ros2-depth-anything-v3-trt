#!/usr/bin/env python3
# Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Complete launch file for depth estimation with camera and visualization.
Launches: USB camera -> Image republisher -> Depth estimation -> Visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('depth_anything_v3')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'depth_anything_v3.param.yaml'),
        description='Path to the parameter file'
    )
    
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Camera image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Camera image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable rqt_image_view for visualization'
    )

    # USB Camera node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'pixel_format': 'yuyv',
            'camera_name': 'depth_camera',
            'io_method': 'mmap',
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )

    # Image transport republisher (raw -> compressed)
    image_republisher = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        output='screen',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out/compressed', '/camera/image_raw/compressed'),
        ]
    )

    # Depth Anything V3 node
    depth_anything_v3_node = Node(
        package='depth_anything_v3',
        executable='depth_anything_v3_main',
        name='depth_anything_v3',
        output='screen',
        remappings=[
            ('~/input/image', '/camera/image_raw/compressed'),
            ('~/input/camera_info', '/camera/camera_info'),
        ],
        parameters=[LaunchConfiguration('params_file')]
    )

    # RQT Image View for input camera
    rqt_input_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_input_view',
        arguments=['/camera/image_raw'],
        condition=IfCondition(LaunchConfiguration('enable_visualization'))
    )

    # RQT Image View for depth output
    rqt_depth_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_depth_view',
        arguments=['/depth_anything_v3/output/depth_image_debug'],
        condition=IfCondition(LaunchConfiguration('enable_visualization'))
    )

    return LaunchDescription([
        # Launch arguments
        params_file_arg,
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        enable_visualization_arg,
        # Nodes
        usb_cam_node,
        image_republisher,
        depth_anything_v3_node,
        rqt_input_view,
        rqt_depth_view,
    ])
