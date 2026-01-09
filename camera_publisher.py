#!/usr/bin/env python3
"""
Camera Information Publisher Node
Uses OpenCV to read camera and publish images and camera intrinsic information
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import json
import yaml
import os


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.camera_info_file = self.get_parameter('camera_info_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # Create publishers - topic names match what the script subscribes to
        self.image_publisher = self.create_publisher(Image, '~/input/image', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '~/input/camera_info', 10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {self.camera_id}')
            raise RuntimeError(f'Cannot open camera {self.camera_id}')
        
        # Set camera parameters
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.publish_rate)
        
        # Get actual resolution
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Camera resolution: {actual_width}x{actual_height}')
        
        # Load or generate camera intrinsics
        self.camera_info = self.load_or_create_camera_info(actual_width, actual_height)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info(f'Camera publisher node started')
        self.get_logger().info(f'Camera ID: {self.camera_id}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Image topic: ~/input/image')
        self.get_logger().info(f'Camera info topic: ~/input/camera_info')
        if self.camera_info_file:
            self.get_logger().info(f'Camera info file: {self.camera_info_file}')
    
    def load_or_create_camera_info(self, width, height):
        """Load or create camera intrinsic information"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.width = width
        camera_info.height = height
        
        # If config file is provided, try to load it
        if self.camera_info_file and os.path.exists(self.camera_info_file):
            try:
                camera_info = self.load_camera_info_from_file(self.camera_info_file, width, height)
                self.get_logger().info(f'Successfully loaded camera info file: {self.camera_info_file}')
                return camera_info
            except Exception as e:
                self.get_logger().warn(f'Failed to load camera info file: {e}, using default values')
        
        # Use default values (based on assumed 60 degree FOV)
        camera_info = self.create_default_camera_info(width, height)
        self.get_logger().info('Using default camera intrinsics (based on 60 degree FOV assumption)')
        
        return camera_info
    
    def load_camera_info_from_file(self, file_path, width, height):
        """Load camera intrinsics from file"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.width = width
        camera_info.height = height
        
        file_ext = os.path.splitext(file_path)[1].lower()
        
        if file_ext == '.json':
            with open(file_path, 'r') as f:
                data = json.load(f)
        elif file_ext in ['.yml', '.yaml']:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
        else:
            raise ValueError(f'Unsupported file format: {file_ext}')
        
        # Extract camera intrinsics
        if 'camera_matrix' in data:
            K = data['camera_matrix']['data']
            camera_info.k = K
        elif 'K' in data:
            camera_info.k = data['K']
        else:
            raise ValueError('Camera matrix not found in camera info file')
        
        # Extract distortion coefficients
        if 'distortion_coefficients' in data:
            D = data['distortion_coefficients']['data']
            camera_info.d = D
        elif 'D' in data:
            camera_info.d = data['D']
        else:
            camera_info.d = [0.0] * 5  # Default: no distortion
        
        # Set rectification matrix (identity matrix)
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Set projection matrix (same as camera matrix)
        camera_info.p = [
            camera_info.k[0], camera_info.k[1], camera_info.k[2], 0.0,
            camera_info.k[3], camera_info.k[4], camera_info.k[5], 0.0,
            camera_info.k[6], camera_info.k[7], camera_info.k[8], 0.0
        ]
        
        return camera_info
    
    def create_default_camera_info(self, width, height):
        """Create default camera intrinsics (based on 60 degree FOV assumption)"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.width = width
        camera_info.height = height
        
        # Assume 60 degree horizontal FOV
        fov_rad = 60.0 * np.pi / 180.0
        focal_length = width / (2.0 * np.tan(fov_rad / 2.0))
        
        # Camera matrix K
        camera_info.k = [
            focal_length, 0.0, width / 2.0,
            0.0, focal_length, height / 2.0,
            0.0, 0.0, 1.0
        ]
        
        # Distortion coefficients (assume no distortion)
        camera_info.d = [0.0] * 5
        
        # Rectification matrix (identity matrix)
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix
        camera_info.p = [
            focal_length, 0.0, width / 2.0, 0.0,
            0.0, focal_length, height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return camera_info
    
    def timer_callback(self):
        """Timer callback function"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Cannot read camera frame')
            return
        
        # Get current timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Publish image
        try:
            # Convert to ROS2 image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = self.frame_id
            self.image_publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')
            return
        
        # Publish camera info
        self.camera_info.header.stamp = timestamp
        self.camera_info_publisher.publish(self.camera_info)
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Node runtime error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
