#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("input_image_topic", default_value="/camera/image"),
        DeclareLaunchArgument("input_depth_topic", default_value="/camera/depth_colored"),
        DeclareLaunchArgument("input_pointcloud_topic", default_value="/camera/point_cloud"),
        DeclareLaunchArgument("output_mosaic_topic", default_value="/camera/depth_mosaic"),
        DeclareLaunchArgument("stream_fps", default_value="15"),
        DeclareLaunchArgument("panel_height", default_value="-1"),
        DeclareLaunchArgument("panel_width", default_value="-1"),
        DeclareLaunchArgument("max_points", default_value="30000"),
        DeclareLaunchArgument("view_yaw_deg", default_value="35.0"),
        DeclareLaunchArgument("view_pitch_deg", default_value="20.0"),
        DeclareLaunchArgument("point_radius", default_value="1"),
        DeclareLaunchArgument("rtsp_port", default_value="8554"),
        DeclareLaunchArgument("rtsp_mount", default_value="/depth"),
        DeclareLaunchArgument("encoder", default_value="auto"),
        DeclareLaunchArgument("bitrate_kbps", default_value="4000"),
    ]

    node = Node(
        package="depth_anything_v3",
        executable="depth_mosaic_rtsp_node.py",
        name="depth_mosaic_rtsp_node",
        output="screen",
        parameters=[
            {
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "input_depth_topic": LaunchConfiguration("input_depth_topic"),
                "input_pointcloud_topic": LaunchConfiguration("input_pointcloud_topic"),
                "output_mosaic_topic": LaunchConfiguration("output_mosaic_topic"),
                "stream_fps": LaunchConfiguration("stream_fps"),
                "panel_height": LaunchConfiguration("panel_height"),
                "panel_width": LaunchConfiguration("panel_width"),
                "max_points": LaunchConfiguration("max_points"),
                "view_yaw_deg": LaunchConfiguration("view_yaw_deg"),
                "view_pitch_deg": LaunchConfiguration("view_pitch_deg"),
                "point_radius": LaunchConfiguration("point_radius"),
                "rtsp_port": LaunchConfiguration("rtsp_port"),
                "rtsp_mount": LaunchConfiguration("rtsp_mount"),
                "encoder": LaunchConfiguration("encoder"),
                "bitrate_kbps": LaunchConfiguration("bitrate_kbps"),
            }
        ],
    )

    return LaunchDescription(args + [node])
