# Implementation Plan: GMSL Camera Integration

## Overview

This implementation plan breaks down the GMSL camera integration feature into discrete coding tasks. The approach follows an incremental development strategy: first establishing the camera abstraction layer, then adding GMSL support, followed by multi-camera capabilities, and finally point cloud fusion.

## Tasks

- [x] 1. Create camera capture abstraction layer
  - Create abstract CameraCapture interface with initialize(), read(), isOpened(), release(), getWidth(), getHeight() methods
  - Implement StandardCameraCapture class wrapping existing cv::VideoCapture functionality
  - Refactor camera_depth_node.cpp to use CameraCapture interface instead of direct cv::VideoCapture
  - _Requirements: 2.2_

- [x] 1.1 Write property test for camera type selection
  - **Property 2: Camera Type Selection**
  - **Validates: Requirements 2.1, 2.2, 2.3**

- [x] 2. Implement GMSL camera support with GStreamer
  - [x] 2.1 Create GMSLCameraCapture class implementing CameraCapture interface
    - Implement GStreamer pipeline construction with v4l2src, video format parameters, and videoconvert
    - Add v4l2-ctl command execution for sensor_mode configuration before pipeline creation
    - Use cv::VideoCapture with CAP_GSTREAMER backend
    - _Requirements: 1.1, 1.2, 1.3, 1.4_

  - [x] 2.2 Write property test for GStreamer pipeline construction
    - **Property 1: GStreamer Pipeline Construction**
    - **Validates: Requirements 1.1, 1.2, 1.3, 1.4**

  - [x] 2.3 Add camera factory function to create appropriate CameraCapture instance
    - Implement factory function that reads camera_type parameter and returns correct instance
    - Add error handling for invalid camera_type values
    - _Requirements: 2.1, 2.2, 2.3_

  - [x] 2.4 Update camera_depth_node.cpp to support GMSL configuration
    - Add new ROS parameters: camera_type, device_path, sensor_mode, framerate
    - Update initCamera() to use camera factory
    - Add GMSL-specific error handling and logging
    - _Requirements: 2.4, 2.5, 1.5_

- [x] 2.5 Write unit tests for GMSL camera initialization
  - Test GStreamer pipeline string generation with various parameters
  - Test v4l2-ctl command construction
  - Test error handling for missing device
  - _Requirements: 1.5, 7.1, 7.2_

- [x] 3. Checkpoint - Ensure single GMSL camera works
  - Verify GMSL camera can be initialized and capture frames
  - Verify depth estimation works with GMSL camera input
  - Ask the user if questions arise

- [x] 4. Implement multi-camera support infrastructure
  - [x] 4.1 Create multi-camera launch file template
    - Create launch file that accepts camera configuration list
    - Generate unique node names and namespaces for each camera
    - Generate unique frame_id for each camera
    - Add static_transform_publisher nodes for each camera
    - _Requirements: 3.1, 3.2, 3.3, 3.4, 5.1, 5.2_

  - [x] 4.2 Write property test for multi-camera node uniqueness
    - **Property 3: Multi-Camera Node Uniqueness**
    - **Validates: Requirements 3.2, 3.3, 3.4**

  - [x] 4.3 Create configuration file for camera positions and orientations
    - Define YAML structure for camera configurations
    - Include device_path, position (x, y, z), orientation (roll, pitch, yaw) for each camera
    - _Requirements: 5.2, 6.3_

- [x] 5. Implement point cloud fusion node
  - [x] 5.1 Create point_cloud_fusion_node.cpp skeleton
    - Set up ROS2 node with parameters for input_topics, target_frame, fusion_rate, timeout
    - Create dynamic subscribers for each input topic
    - Create publisher for fused point cloud
    - Initialize TF2 buffer and listener
    - _Requirements: 4.1, 4.5, 4.6_

  - [x] 5.2 Implement point cloud buffering and timeout handling
    - Create map to store latest point cloud from each topic
    - Implement timeout mechanism to remove stale point clouds
    - Add callback for each subscribed topic to update buffer
    - _Requirements: 7.3, 8.2_

  - [x] 5.3 Implement point cloud transformation logic
    - Look up transform from source frame to target frame using TF2
    - Transform point cloud using tf2::doTransform
    - Handle transform lookup failures gracefully
    - _Requirements: 4.2, 5.3, 5.4, 5.5_

  - [x] 5.4 Write property test for point cloud transformation preservation
    - **Property 4: Point Cloud Transformation Preservation**
    - **Validates: Requirements 4.2, 5.3**

  - [x] 5.5 Implement point cloud merging algorithm
    - Iterate through buffered point clouds
    - Transform each to target frame
    - Concatenate all points into single output cloud
    - Preserve RGB color information
    - _Requirements: 4.3, 4.4_

  - [x] 5.6 Write property test for point cloud fusion completeness
    - **Property 5: Point Cloud Fusion Completeness**
    - **Validates: Requirements 4.3, 4.4**

  - [x] 5.7 Write property test for transform availability handling
    - **Property 6: Transform Availability Handling**
    - **Validates: Requirements 5.4, 5.5, 7.3**

  - [x] 5.8 Add fusion node to CMakeLists.txt and package.xml
    - Register point_cloud_fusion_node executable
    - Add TF2 dependencies
    - _Requirements: 4.1_

- [x] 6. Checkpoint - Ensure point cloud fusion works
  - Launch 2 GMSL cameras with fusion node
  - Verify fused point cloud is published
  - Verify point clouds are correctly transformed
  - Ask the user if questions arise

- [x] 7. Create complete multi-camera launch file with fusion
  - [x] 7.1 Implement gmsl_multi_camera_fusion.launch.py
    - Launch multiple camera_depth_node instances with GMSL configuration
    - Launch static_transform_publisher for each camera
    - Launch point_cloud_fusion_node
    - Launch RViz2 with appropriate configuration
    - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5_

  - [x] 7.2 Create RViz configuration file for multi-camera visualization
    - Configure PointCloud2 display for fused point cloud
    - Configure TF display for camera frames
    - Set appropriate fixed frame
    - _Requirements: 6.5_

  - [x] 7.3 Create shell script wrapper for easy launching
    - Add user prompts for number of cameras
    - Add device path configuration
    - Add camera position configuration
    - _Requirements: 6.1, 6.2, 6.3_

- [x] 8. Add error handling and robustness improvements
  - [x] 8.1 Enhance camera initialization error handling
    - Add detailed GStreamer error logging
    - Add device existence verification
    - Add retry logic for frame capture
    - _Requirements: 1.5, 7.1, 7.2, 7.4_

  - [x] 8.2 Add runtime error handling for camera failures
    - Implement frame capture timeout
    - Add graceful degradation when camera stops producing frames
    - Log warnings but continue operation
    - _Requirements: 7.2, 7.4_

  - [x] 8.3 Write property test for camera failure isolation
    - **Property 7: Camera Failure Isolation**
    - **Validates: Requirements 7.1, 7.2**

  - [x] 8.3 Add performance monitoring and logging
    - Add timing information for frame capture
    - Add timing information for depth estimation
    - Add timing information for point cloud fusion
    - Log frame rates and latencies
    - _Requirements: 8.5_

  - [x] 8.4 Write property test for frame timestamp consistency
    - **Property 8: Frame Timestamp Consistency**
    - **Validates: Requirements 8.1, 8.5**

- [x] 9. Performance optimization
  - [x] 9.1 Optimize point cloud fusion algorithm
    - Use efficient data structures for point concatenation
    - Consider spatial filtering for overlapping regions
    - Profile and optimize hot paths
    - _Requirements: 8.2, 8.4_

  - [x] 9.2 Add configurable publish rates
    - Allow different publish rates for each camera
    - Balance performance and latency
    - _Requirements: 8.3_

  - [x] 9.3 Write integration tests for multi-camera performance
    - Test with 4 cameras simultaneously
    - Measure frame rates and latencies
    - Verify real-time performance (>10 Hz)
    - _Requirements: 8.1, 8.3, 8.5_

- [x] 10. Documentation and examples
  - [x] 10.1 Update README with GMSL camera setup instructions
    - Document GStreamer dependencies
    - Document v4l2-ctl usage
    - Provide example configurations
    - _Requirements: 6.1, 6.2, 6.3_

  - [x] 10.2 Create example configuration files
    - Provide 4-camera GMSL configuration example
    - Provide camera calibration example
    - _Requirements: 6.3_

- [x] 11. Final checkpoint - Complete system validation
  - Launch 4 GMSL cameras with fusion
  - Verify all cameras capture frames
  - Verify fused point cloud in RViz2
  - Verify spatial alignment and color accuracy
  - Ensure all tests pass
  - Ask the user if questions arise

## Notes

- Each task references specific requirements for traceability
- Checkpoints ensure incremental validation
- Property tests validate universal correctness properties
- Unit tests validate specific examples and edge cases
- The implementation follows a bottom-up approach: abstraction layer → GMSL support → multi-camera → fusion
- GStreamer and TF2 dependencies must be available in the build environment
