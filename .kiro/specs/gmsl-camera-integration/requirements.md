# Requirements Document

## Introduction

This document specifies the requirements for integrating GMSL (Gigabit Multimedia Serial Link) cameras into the existing ROS2 Depth Anything V3 system. The system currently uses OpenCV's VideoCapture for standard USB/V4L2 cameras, which does not support GMSL cameras properly. This feature will enable the system to work with GMSL cameras using GStreamer pipelines and support multiple cameras with point cloud fusion.

## Glossary

- **GMSL_Camera**: Gigabit Multimedia Serial Link camera, a high-speed camera interface commonly used in automotive applications
- **Camera_Node**: The ROS2 node responsible for capturing camera frames and performing depth estimation
- **GStreamer_Pipeline**: A multimedia framework pipeline used to capture and process video streams from GMSL cameras
- **Point_Cloud_Fusion**: The process of combining point clouds from multiple cameras into a single unified point cloud
- **V4L2**: Video4Linux2, a Linux kernel API for video capture devices
- **Frame_ID**: A unique identifier for each camera's coordinate frame in the ROS2 TF tree

## Requirements

### Requirement 1: GMSL Camera Initialization

**User Story:** As a robotics developer, I want to initialize GMSL cameras using GStreamer pipelines, so that I can capture frames from GMSL cameras instead of standard USB cameras.

#### Acceptance Criteria

1. WHEN the Camera_Node starts with GMSL camera configuration, THE System SHALL initialize the camera using a GStreamer pipeline with v4l2src
2. WHEN initializing a GMSL camera, THE System SHALL set the video format to width=1920, height=1536, framerate=30/1, format=UYVY
3. WHEN initializing a GMSL camera, THE System SHALL set sensor_mode=0 using v4l2-ctl parameters
4. WHEN the GStreamer pipeline is created, THE System SHALL include videoconvert element for format conversion
5. IF the GMSL camera initialization fails, THEN THE System SHALL log a descriptive error message and gracefully shut down

### Requirement 2: Camera Type Configuration

**User Story:** As a system operator, I want to configure whether to use standard cameras or GMSL cameras, so that the system can work with different camera hardware without code changes.

#### Acceptance Criteria

1. THE Camera_Node SHALL accept a parameter "camera_type" with values "standard" or "gmsl"
2. WHEN camera_type is "standard", THE System SHALL use OpenCV VideoCapture with camera_id
3. WHEN camera_type is "gmsl", THE System SHALL use GStreamer pipeline with device path
4. THE Camera_Node SHALL accept a parameter "device_path" for specifying the V4L2 device (e.g., "/dev/video0")
5. WHERE camera_type is "gmsl", THE Camera_Node SHALL accept parameters for width, height, framerate, and sensor_mode

### Requirement 3: Multi-Camera Support

**User Story:** As a robotics developer, I want to run multiple camera nodes simultaneously, so that I can process depth from multiple viewpoints.

#### Acceptance Criteria

1. THE System SHALL support launching multiple Camera_Node instances with different configurations
2. WHEN multiple Camera_Node instances are launched, THE System SHALL assign unique node names to each instance
3. WHEN multiple Camera_Node instances are launched, THE System SHALL assign unique topic namespaces to each instance
4. WHEN multiple Camera_Node instances are launched, THE System SHALL assign unique frame_id values to each camera
5. THE System SHALL provide a launch file that can instantiate multiple camera nodes with different device paths

### Requirement 4: Point Cloud Fusion

**User Story:** As a robotics developer, I want to combine point clouds from multiple cameras into a single unified point cloud, so that I can have a comprehensive 3D view of the environment.

#### Acceptance Criteria

1. THE System SHALL provide a Point_Cloud_Fusion node that subscribes to multiple point cloud topics
2. WHEN the Point_Cloud_Fusion node receives point clouds from multiple cameras, THE System SHALL transform them to a common reference frame
3. WHEN combining point clouds, THE System SHALL preserve the RGB color information from each camera
4. WHEN combining point clouds, THE System SHALL publish the fused point cloud on a dedicated topic
5. THE Point_Cloud_Fusion node SHALL accept a parameter specifying the list of input point cloud topics
6. THE Point_Cloud_Fusion node SHALL accept a parameter specifying the target reference frame for fusion

### Requirement 5: Transform Management

**User Story:** As a system operator, I want to define the spatial relationships between multiple cameras, so that point clouds can be correctly transformed and fused.

#### Acceptance Criteria

1. THE System SHALL publish static transforms from a base frame to each camera frame
2. WHEN launching multiple cameras, THE System SHALL accept parameters for each camera's position (x, y, z) and orientation (roll, pitch, yaw)
3. THE System SHALL use the ROS2 TF2 library for coordinate frame transformations
4. WHEN transforming point clouds, THE System SHALL use the most recent available transform
5. IF a required transform is not available, THEN THE Point_Cloud_Fusion node SHALL log a warning and skip that point cloud

### Requirement 6: Launch File Configuration

**User Story:** As a system operator, I want a convenient launch file to start multiple GMSL cameras with fusion, so that I can easily configure and run the complete system.

#### Acceptance Criteria

1. THE System SHALL provide a launch file for multi-camera GMSL setup with point cloud fusion
2. THE launch file SHALL accept parameters for the number of cameras to launch
3. THE launch file SHALL accept parameters for each camera's device path, position, and orientation
4. WHEN the launch file is executed, THE System SHALL start all camera nodes, transform publishers, and the fusion node
5. THE launch file SHALL include RViz2 with appropriate visualization configuration for the fused point cloud

### Requirement 7: Error Handling and Robustness

**User Story:** As a system operator, I want the system to handle camera failures gracefully, so that one camera failure doesn't crash the entire system.

#### Acceptance Criteria

1. IF a camera fails to initialize, THEN THE System SHALL log the error and shut down only that camera node
2. IF a camera stops producing frames during operation, THEN THE System SHALL log warnings but continue operating
3. WHEN the Point_Cloud_Fusion node doesn't receive data from a camera, THE System SHALL continue fusing available point clouds
4. THE System SHALL implement timeout mechanisms for camera frame capture
5. IF GStreamer pipeline creation fails, THEN THE System SHALL provide diagnostic information about the failure

### Requirement 8: Performance Optimization

**User Story:** As a robotics developer, I want the multi-camera system to process frames efficiently, so that I can achieve real-time performance.

#### Acceptance Criteria

1. WHEN processing multiple cameras, THE System SHALL process each camera's depth estimation independently
2. THE Point_Cloud_Fusion node SHALL use efficient data structures for combining point clouds
3. THE System SHALL support configurable publish rates for each camera to balance performance and latency
4. WHEN fusing point clouds, THE System SHALL use spatial filtering to remove duplicate or overlapping points
5. THE System SHALL provide timing information in log messages for performance monitoring
