#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Camera/depth args
    camera_args = [
        DeclareLaunchArgument("camera_type", default_value="standard"),
        DeclareLaunchArgument("camera_id", default_value="0"),
        DeclareLaunchArgument("model_path", default_value="onnx/DA3METRIC-LARGE.onnx"),
        DeclareLaunchArgument("publish_rate", default_value="10.0"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("downsample_factor", default_value="1"),
        DeclareLaunchArgument("camera_info_file", default_value=""),
        DeclareLaunchArgument("use_calibration", default_value="false"),
        DeclareLaunchArgument("fx", default_value="824.147361"),
        DeclareLaunchArgument("fy", default_value="823.660879"),
        DeclareLaunchArgument("cx", default_value="958.275200"),
        DeclareLaunchArgument("cy", default_value="767.389372"),
        DeclareLaunchArgument("k1", default_value="1.486308"),
        DeclareLaunchArgument("k2", default_value="-13.386609"),
        DeclareLaunchArgument("p1", default_value="21.409334"),
        DeclareLaunchArgument("p2", default_value="3.817858"),
        DeclareLaunchArgument("k3", default_value="0.0"),
        DeclareLaunchArgument("enable_undistortion", default_value="false"),
        DeclareLaunchArgument("undistortion_balance", default_value="0.0"),
    ]

    # Mosaic/RTSP args
    rtsp_args = [
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
        DeclareLaunchArgument("output_mosaic_topic", default_value="/camera/depth_mosaic"),
    ]

    camera_depth_node = Node(
        package="depth_anything_v3",
        executable="camera_depth_node",
        name="camera_depth_node",
        output="screen",
        parameters=[
            {
                "camera_type": LaunchConfiguration("camera_type"),
                "camera_id": LaunchConfiguration("camera_id"),
                "model_path": LaunchConfiguration("model_path"),
                "frame_id": "camera_link",
                "publish_rate": LaunchConfiguration("publish_rate"),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
                "downsample_factor": LaunchConfiguration("downsample_factor"),
                "camera_info_file": LaunchConfiguration("camera_info_file"),
                "use_calibration": LaunchConfiguration("use_calibration"),
                "fx": LaunchConfiguration("fx"),
                "fy": LaunchConfiguration("fy"),
                "cx": LaunchConfiguration("cx"),
                "cy": LaunchConfiguration("cy"),
                "k1": LaunchConfiguration("k1"),
                "k2": LaunchConfiguration("k2"),
                "p1": LaunchConfiguration("p1"),
                "p2": LaunchConfiguration("p2"),
                "k3": LaunchConfiguration("k3"),
                "enable_undistortion": LaunchConfiguration("enable_undistortion"),
                "undistortion_balance": LaunchConfiguration("undistortion_balance"),
            }
        ],
        remappings=[
            ("~/input/image", "/camera/image"),
            ("~/output/depth_image", "/camera/depth"),
            ("~/output/depth_colored", "/camera/depth_colored"),
            ("~/output/point_cloud", "/camera/point_cloud"),
            ("~/camera_info", "/camera/camera_info"),
        ],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera_link"],
    )

    mosaic_rtsp_node = Node(
        package="depth_anything_v3",
        executable="depth_mosaic_rtsp_node.py",
        name="depth_mosaic_rtsp_node",
        output="screen",
        parameters=[
            {
                "input_image_topic": "/camera/image",
                "input_depth_topic": "/camera/depth_colored",
                "input_pointcloud_topic": "/camera/point_cloud",
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

    return LaunchDescription(camera_args + rtsp_args + [camera_depth_node, static_tf_node, mosaic_rtsp_node])
