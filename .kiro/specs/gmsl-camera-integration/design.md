# Design Document: GMSL Camera Integration

## Overview

This design extends the existing ROS2 Depth Anything V3 system to support GMSL cameras through GStreamer pipelines and enables multi-camera point cloud fusion. The current implementation uses OpenCV's VideoCapture which doesn't properly support GMSL cameras. This design introduces:

1. A flexible camera initialization system supporting both standard and GMSL cameras
2. GStreamer pipeline integration for GMSL camera capture
3. Multi-camera node architecture with independent processing
4. A point cloud fusion node that combines outputs from multiple cameras
5. TF2-based coordinate transformation for spatial alignment

## Architecture

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 System                              │
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Camera Node  │  │ Camera Node  │  │ Camera Node  │     │
│  │   (GMSL 0)   │  │   (GMSL 1)   │  │   (GMSL 2)   │     │
│  │              │  │              │  │              │     │
│  │ /dev/video0  │  │ /dev/video1  │  │ /dev/video2  │     │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘     │
│         │                 │                 │              │
│         │ PointCloud2     │ PointCloud2     │ PointCloud2  │
│         │                 │                 │              │
│         └─────────┬───────┴─────────┬───────┘              │
│                   │                 │                      │
│                   ▼                 ▼                      │
│         ┌─────────────────────────────────┐               │
│         │   Point Cloud Fusion Node       │               │
│         │                                 │               │
│         │  - Subscribe to multiple topics │               │
│         │  - Transform to common frame    │               │
│         │  - Merge point clouds           │               │
│         │  - Publish fused output         │               │
│         └─────────────┬───────────────────┘               │
│                       │                                    │
│                       │ Fused PointCloud2                  │
│                       ▼                                    │
│              ┌─────────────────┐                          │
│              │     RViz2       │                          │
│              └─────────────────┘                          │
└─────────────────────────────────────────────────────────────┘
```

### Camera Initialization Strategy

The system will use a factory pattern to create camera capture objects based on configuration:

```
CameraCapture (Abstract Interface)
    ├── StandardCameraCapture (OpenCV VideoCapture)
    └── GMSLCameraCapture (GStreamer Pipeline)
```

## Components and Interfaces

### 1. Camera Capture Abstraction

**Purpose**: Provide a unified interface for different camera types

**Interface**:
```cpp
class CameraCapture {
public:
    virtual ~CameraCapture() = default;
    virtual bool initialize() = 0;
    virtual bool read(cv::Mat& frame) = 0;
    virtual bool isOpened() const = 0;
    virtual void release() = 0;
    virtual int getWidth() const = 0;
    virtual int getHeight() const = 0;
};
```

**StandardCameraCapture Implementation**:
- Uses cv::VideoCapture
- Supports camera_id parameter
- Existing functionality preserved

**GMSLCameraCapture Implementation**:
- Constructs GStreamer pipeline string
- Uses cv::VideoCapture with CAP_GSTREAMER backend
- Pipeline format: `v4l2src device=/dev/videoX ! video/x-raw,width=W,height=H,framerate=F/1,format=UYVY ! videoconvert ! appsink`
- Applies v4l2-ctl settings before pipeline creation

### 2. Enhanced Camera Depth Node

**Modified Parameters**:
```yaml
camera_type: "gmsl"  # or "standard"
device_path: "/dev/video0"  # for GMSL cameras
camera_id: 0  # for standard cameras
camera_width: 1920
camera_height: 1536
framerate: 30
sensor_mode: 0  # GMSL specific
frame_id: "camera_0_link"
```

**Initialization Flow**:
```
1. Read parameters
2. Create appropriate CameraCapture instance based on camera_type
3. Initialize camera capture
4. Verify frame capture
5. Initialize depth estimator
6. Start publishing loop
```

### 3. Point Cloud Fusion Node

**Purpose**: Combine point clouds from multiple cameras into a unified representation

**Subscriptions**:
- Dynamic list of PointCloud2 topics (configured via parameter)
- Example: ["/camera_0/point_cloud", "/camera_1/point_cloud", "/camera_2/point_cloud"]

**Publications**:
- `/fused_point_cloud` (sensor_msgs::msg::PointCloud2)

**Parameters**:
```yaml
input_topics: ["/camera_0/point_cloud", "/camera_1/point_cloud"]
target_frame: "base_link"
fusion_rate: 10.0  # Hz
timeout: 0.5  # seconds, max age for point clouds
```

**Algorithm**:
```
For each timer callback:
    1. Collect latest point cloud from each subscribed topic
    2. For each point cloud:
        a. Check if transform from source frame to target frame exists
        b. Transform point cloud to target frame using TF2
        c. Add transformed points to output cloud
    3. Publish fused point cloud
```

### 4. Multi-Camera Launch File

**Purpose**: Simplify launching multiple camera nodes with fusion

**Structure**:
```python
def generate_launch_description():
    # Camera configurations
    cameras = [
        {"device": "/dev/video0", "name": "camera_0", "x": 0.0, "y": 0.0, "z": 0.0},
        {"device": "/dev/video1", "name": "camera_1", "x": 0.5, "y": 0.0, "z": 0.0},
        {"device": "/dev/video2", "name": "camera_2", "x": -0.5, "y": 0.0, "z": 0.0},
    ]
    
    # Launch camera nodes
    # Launch static transform publishers
    # Launch fusion node
    # Launch RViz2
```

## Data Models

### Camera Configuration
```cpp
struct CameraConfig {
    std::string camera_type;  // "standard" or "gmsl"
    std::string device_path;  // "/dev/videoX"
    int camera_id;            // for standard cameras
    int width;
    int height;
    int framerate;
    int sensor_mode;          // GMSL specific
    std::string frame_id;
};
```

### GStreamer Pipeline Parameters
```cpp
struct GStreamerParams {
    std::string device_path;
    int width;
    int height;
    int framerate;
    std::string format;  // "UYVY"
    int sensor_mode;
    
    std::string buildPipeline() const {
        return "v4l2src device=" + device_path + 
               " ! video/x-raw,width=" + std::to_string(width) +
               ",height=" + std::to_string(height) +
               ",framerate=" + std::to_string(framerate) + "/1" +
               ",format=" + format +
               " ! videoconvert ! appsink";
    }
};
```

### Point Cloud Message Buffer
```cpp
struct PointCloudBuffer {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    rclcpp::Time timestamp;
    std::string source_frame;
};

std::map<std::string, PointCloudBuffer> cloud_buffers_;  // topic -> buffer
```

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*

### Property 1: GStreamer Pipeline Construction

*For any* valid GStreamerParams configuration, the constructed pipeline string should contain all required elements (v4l2src, video/x-raw with correct parameters, videoconvert, appsink) in the correct order.

**Validates: Requirements 1.1, 1.2, 1.3, 1.4**

### Property 2: Camera Type Selection

*For any* camera configuration, when camera_type is "standard", the system should create a StandardCameraCapture instance, and when camera_type is "gmsl", the system should create a GMSLCameraCapture instance.

**Validates: Requirements 2.1, 2.2, 2.3**

### Property 3: Multi-Camera Node Uniqueness

*For any* set of camera configurations in a multi-camera launch, all node names, topic namespaces, and frame_ids should be unique (no duplicates).

**Validates: Requirements 3.2, 3.3, 3.4**

### Property 4: Point Cloud Transformation Preservation

*For any* point cloud and valid transform, transforming the point cloud should preserve the number of points and the relative distances between points (rigid transformation).

**Validates: Requirements 4.2, 5.3**

### Property 5: Point Cloud Fusion Completeness

*For any* set of input point clouds, the fused point cloud should contain at least as many points as the largest input point cloud (assuming no spatial filtering).

**Validates: Requirements 4.3, 4.4**

### Property 6: Transform Availability Handling

*For any* point cloud with an unavailable transform, the fusion node should skip that point cloud and continue processing other point clouds without crashing.

**Validates: Requirements 5.4, 5.5, 7.3**

### Property 7: Camera Failure Isolation

*For any* camera node failure during initialization, only that specific camera node should shut down, and other camera nodes should continue operating independently.

**Validates: Requirements 7.1, 7.2**

### Property 8: Frame Timestamp Consistency

*For any* camera node, all published messages (image, depth, point cloud, camera_info) from the same frame capture should have identical timestamps.

**Validates: Requirements 8.1, 8.5**

## Error Handling

### Camera Initialization Errors

1. **GStreamer Pipeline Creation Failure**
   - Log the complete pipeline string
   - Log GStreamer error messages
   - Attempt to verify device exists with system call
   - Shut down node gracefully

2. **V4L2 Device Not Found**
   - Check if device path exists
   - Suggest running `v4l2-ctl --list-devices`
   - Provide clear error message

3. **Frame Capture Timeout**
   - Retry frame capture up to 5 times
   - Log warning after each failure
   - Shut down if all retries fail

### Runtime Errors

1. **Point Cloud Transform Unavailable**
   - Log warning with source and target frames
   - Skip that point cloud for current fusion cycle
   - Continue with available point clouds

2. **Point Cloud Timeout**
   - Remove stale point clouds from buffer
   - Log info message about missing camera data
   - Continue fusion with available data

3. **Memory Allocation Failure**
   - Log error with requested size
   - Attempt to reduce point cloud density
   - Gracefully degrade performance

## Testing Strategy

### Unit Tests

1. **GStreamer Pipeline Construction**
   - Test pipeline string generation with various parameters
   - Verify all required elements are present
   - Test edge cases (empty device path, zero dimensions)

2. **Camera Capture Factory**
   - Test creation of StandardCameraCapture
   - Test creation of GMSLCameraCapture
   - Test invalid camera_type handling

3. **Point Cloud Transformation**
   - Test transformation with identity transform
   - Test transformation with translation
   - Test transformation with rotation
   - Verify point count preservation

4. **Point Cloud Fusion**
   - Test fusion of two point clouds
   - Test fusion with missing transforms
   - Test fusion with stale data
   - Verify color preservation

### Property-Based Tests

Each property test should run a minimum of 100 iterations with randomized inputs.

1. **Property 1: GStreamer Pipeline Construction**
   - Generate random valid GStreamerParams
   - Build pipeline string
   - Verify all required elements present
   - **Tag**: Feature: gmsl-camera-integration, Property 1: GStreamer Pipeline Construction

2. **Property 2: Camera Type Selection**
   - Generate random camera configurations
   - Create camera capture instances
   - Verify correct type instantiated
   - **Tag**: Feature: gmsl-camera-integration, Property 2: Camera Type Selection

3. **Property 3: Multi-Camera Node Uniqueness**
   - Generate random sets of camera configurations
   - Extract names, namespaces, frame_ids
   - Verify all unique
   - **Tag**: Feature: gmsl-camera-integration, Property 3: Multi-Camera Node Uniqueness

4. **Property 4: Point Cloud Transformation Preservation**
   - Generate random point clouds
   - Generate random rigid transforms
   - Transform point cloud
   - Verify point count and distances preserved
   - **Tag**: Feature: gmsl-camera-integration, Property 4: Point Cloud Transformation Preservation

5. **Property 5: Point Cloud Fusion Completeness**
   - Generate random sets of point clouds
   - Fuse point clouds
   - Verify fused cloud size >= max input size
   - **Tag**: Feature: gmsl-camera-integration, Property 5: Point Cloud Fusion Completeness

6. **Property 6: Transform Availability Handling**
   - Generate point clouds with missing transforms
   - Attempt fusion
   - Verify no crash and partial fusion succeeds
   - **Tag**: Feature: gmsl-camera-integration, Property 6: Transform Availability Handling

7. **Property 7: Camera Failure Isolation**
   - Simulate camera initialization failures
   - Verify only failed node shuts down
   - Verify other nodes continue
   - **Tag**: Feature: gmsl-camera-integration, Property 7: Camera Failure Isolation

8. **Property 8: Frame Timestamp Consistency**
   - Capture frames from camera node
   - Collect all published messages
   - Verify timestamps match
   - **Tag**: Feature: gmsl-camera-integration, Property 8: Frame Timestamp Consistency

### Integration Tests

1. **Single GMSL Camera Test**
   - Launch single GMSL camera node
   - Verify frame capture
   - Verify depth estimation
   - Verify point cloud publication

2. **Multi-Camera Launch Test**
   - Launch 4 GMSL cameras
   - Verify all nodes start
   - Verify unique topics
   - Verify transforms published

3. **Point Cloud Fusion Test**
   - Launch 2 cameras + fusion node
   - Verify fused point cloud published
   - Verify point count increases
   - Verify visualization in RViz2

4. **Camera Failure Recovery Test**
   - Launch 3 cameras
   - Simulate 1 camera failure
   - Verify other 2 continue
   - Verify fusion continues with 2 cameras

### Manual Testing

1. **Visual Verification**
   - Run system with 4 GMSL cameras
   - View fused point cloud in RViz2
   - Verify spatial alignment
   - Verify color accuracy

2. **Performance Testing**
   - Measure frame rate for each camera
   - Measure fusion latency
   - Monitor CPU and memory usage
   - Verify real-time performance (>10 Hz)

## Implementation Notes

### GStreamer Dependencies

The system requires GStreamer development libraries:
```bash
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

OpenCV must be compiled with GStreamer support:
```bash
cmake -D WITH_GSTREAMER=ON ...
```

### V4L2 Control Application

Before creating the GStreamer pipeline, apply v4l2 settings:
```cpp
std::string v4l2_cmd = "v4l2-ctl -d " + device_path + 
                       " --set-fmt-video=width=" + std::to_string(width) +
                       ",height=" + std::to_string(height) +
                       " -c sensor_mode=" + std::to_string(sensor_mode);
system(v4l2_cmd.c_str());
```

### TF2 Transform Lookup

Use TF2 buffer with timeout for robust transform lookup:
```cpp
geometry_msgs::msg::TransformStamped transform;
try {
    transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, 
        tf2::durationFromSec(0.1));
} catch (tf2::TransformException& ex) {
    RCLCPP_WARN(logger_, "Transform lookup failed: %s", ex.what());
    return false;
}
```

### Point Cloud Merging Optimization

Use PCL (Point Cloud Library) for efficient merging:
```cpp
pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>);
for (const auto& cloud : input_clouds) {
    *merged += *cloud;  // Efficient concatenation
}
```

Alternatively, use sensor_msgs::PointCloud2Modifier for ROS-native approach without PCL dependency.
