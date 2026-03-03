# Depth Anything V3 TensorRT ROS 2 - Complete System

A comprehensive ROS 2 system for Depth Anything V3 monocular depth estimation using TensorRT acceleration. Supports real-time camera processing, video file analysis, and standalone demos with 3D point cloud visualization.

## 🎥 Demo Video

https://github.com/user-attachments/assets/d119d3b8-bba1-43a3-9f86-75db24e01235

## ✨ Features

- **Real-time Camera Depth Estimation** - Process live camera feed with RViz2 3D visualization
- **Video File Processing** - Analyze video files with loop playback and speed control
- **Standalone C++ Demo** - OpenCV-based demo without ROS dependencies
- **TensorRT Acceleration** - Optimized inference with FP16/FP32 precision
- **3D Point Cloud Generation** - RGB-colored point clouds from metric depth
- **Performance Optimization** - Configurable resolution scaling for faster inference

## 📋 Table of Contents

- [System Requirements](#system-requirements)
- [Environment Setup](#environment-setup)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage Modes](#usage-modes)
- [Configuration](#configuration)
- [Performance Tuning](#performance-tuning)
- [Troubleshooting](#troubleshooting)
- [Topics and Parameters](#topics-and-parameters)
- [License](#license)

## 🖥️ System Requirements

### Hardware
- **GPU**: NVIDIA GPU with CUDA support (Tested on Jetson Orin, RTX series)
- **RAM**: 8GB minimum, 16GB recommended
- **Camera**: USB camera (UVC) or GMSL camera (for live processing)

### Software
- **OS**: Ubuntu 22.04 (Jetson) or Ubuntu 24.04
- **ROS 2**: Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- **CUDA**: 12.6+ (Jetson) or 12.8+ (Desktop)
- **TensorRT**: 10.3+ (Jetson) or 10.9+ (Desktop)
- **OpenCV**: 4.5+

### Tested Platforms
- ✅ NVIDIA Jetson Orin (Ubuntu 22.04, ROS 2 Humble, CUDA 12.6, TensorRT 10.3)
- ✅ Desktop RTX GPU (Ubuntu 24.04, ROS 2 Jazzy, CUDA 12.8, TensorRT 10.9)

## 🔧 Environment Setup

### 1. Install ROS 2

**For Ubuntu 22.04 (ROS 2 Humble):**
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport ros-humble-rviz2

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**For Ubuntu 24.04 (ROS 2 Jazzy):**
```bash
# Similar steps but install ros-jazzy-desktop
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-rviz2
```

### 2. Install CUDA and TensorRT

**For Jetson (Pre-installed):**
```bash
# Verify installation
nvcc --version
dpkg -l | grep TensorRT
```

**For Desktop:**
```bash
# Download and install CUDA Toolkit from NVIDIA website
# https://developer.nvidia.com/cuda-downloads

# Download and install TensorRT
# https://developer.nvidia.com/tensorrt

# Set environment variables
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export CUDACXX=$CUDA_HOME/bin/nvcc

# Add to ~/.bashrc for persistence
echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
echo 'export PATH=$CUDA_HOME/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export CUDACXX=$CUDA_HOME/bin/nvcc' >> ~/.bashrc
```

### 3. Install Dependencies

```bash
# System dependencies
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    python3-pip \
    python3-colcon-common-extensions \
    v4l-utils

# Python dependencies
pip3 install numpy opencv-python
```

## 📦 Installation

### 1. Clone Repository

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/ros2-depth-anything-v3-trt.git
cd ros2-depth-anything-v3-trt
```

### 2. Download Model

Download the ONNX model from Hugging Face:
```bash
# Create model directory
mkdir -p onnx

# Download model (choose one size)
# Option 1: Large model (best quality, ~700MB)
wget https://huggingface.co/TillBeemelmanns/Depth-Anything-V3-ONNX/resolve/main/DA3METRIC-LARGE.onnx -O onnx/DA3METRIC-LARGE.onnx

# Option 2: Base model (balanced, ~400MB)
wget https://huggingface.co/TillBeemelmanns/Depth-Anything-V3-ONNX/resolve/main/DA3METRIC-BASE.onnx -O onnx/DA3METRIC-BASE.onnx

# Option 3: Small model (fastest, ~200MB)
wget https://huggingface.co/TillBeemelmanns/Depth-Anything-V3-ONNX/resolve/main/DA3METRIC-SMALL.onnx -O onnx/DA3METRIC-SMALL.onnx
```

### 3. Build Project

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Set CUDA environment (if not in ~/.bashrc)
export CUDA_HOME=/usr/local/cuda
export CUDACXX=$CUDA_HOME/bin/nvcc

# Build
colcon build --packages-select depth_anything_v3 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
```

### 4. Generate TensorRT Engine (Optional but Recommended)

```bash
# This pre-builds the TensorRT engine (takes 5-10 minutes on first run)
cd ~/ros2_ws/src/ros2-depth-anything-v3-trt
./generate_engines.sh
```

## 🚀 Quick Start

### Mode 1: Single Camera Depth Estimation (Recommended for Testing)

Process live camera feed and visualize 3D point cloud in RViz2:

```bash
cd ~/ros2_ws/src/ros2-depth-anything-v3-trt
USE_CALIBRATION=1 ./test_single_camera_fast.sh
```

### Mode 2: Multi-Camera System (Recommended for Production)

Process multiple cameras simultaneously with point cloud fusion:

```bash
cd ~/ros2_ws/src/ros2-depth-anything-v3-trt

# 2-camera system (STABLE - Recommended)
USE_CALIBRATION=1 ./run_2cameras_fast.sh

# 4-camera system (May crash due to GPU memory)
USE_CALIBRATION=1 ./run_final_system_fast.sh
```

**⚠️ Important**: The 4-camera system may crash on Jetson Orin due to GPU memory exhaustion (~2.6-4.0 GB). Use the 2-camera system for stable operation (~1.3 GB).

**Interactive prompts:**
1. Select camera device (default: /dev/video0)
2. Choose resolution (640x480 recommended)
3. RViz2 will open showing:
   - Original camera image
   - Colored depth map
   - 3D point cloud

**Controls:**
- Rotate: Left mouse drag
- Zoom: Mouse wheel
- Pan: Middle mouse drag or Shift+Left drag
- Stop: Ctrl+C

### Mode 3: Video File Processing

Analyze video files with loop playback:

```bash
cd ~/ros2_ws/src/ros2-depth-anything-v3-trt
./run_video_depth.sh /path/to/your/video.mp4

# Or use the demo video
./run_video_depth.sh demo.mp4
```

**Interactive prompts:**
1. Playback speed (0.5x / 1.0x / 2.0x)
2. Loop mode (enabled/disabled)
3. Resolution scaling (1.0 / 0.75 / 0.5 / 0.25)

### Mode 4: Standalone C++ Demo

OpenCV-based demo without ROS:

```bash
cd ~/ros2_ws/src/ros2-depth-anything-v3-trt
./run_standalone_demo.sh
```

**Controls:**
- Press 's' to save current frame
- Press 'q' or ESC to quit

### Mode 5: Camera + Depth + PointCloud RTSP Streaming

Compose point cloud (top), camera image (bottom-left), and depth image (bottom-right),
then stream as H264 RTSP:

```bash
cd ~/ros2_ws/src/ros2-depth-anything-v3-trt
./run_camera_depth_rtsp.sh
```

Default URL:
- Local: `rtsp://127.0.0.1:8554/depth`
- LAN: `rtsp://<jetson-ip>:8554/depth`

Viewer examples:
```bash
ffplay rtsp://127.0.0.1:8554/depth
vlc rtsp://<jetson-ip>:8554/depth
```

## 📖 Usage Modes

### 1. Single Camera Depth Estimation

**One-line launch:**
```bash
USE_CALIBRATION=1 ./test_single_camera_fast.sh
```

**Features:**
- Fisheye camera calibration support
- 2x downsampling for faster inference (960×768)
- ~1.0-2.0 Hz depth estimation
- GPU memory: ~650 MiB

**Manual launch with custom parameters:**
```bash
source install/setup.bash

ros2 launch depth_anything_v3 camera_depth_rviz.launch.py \
  camera_id:=0 \
  camera_width:=1920 \
  camera_height:=1536 \
  publish_rate:=10.0 \
  downsample_factor:=2 \
  use_calibration:=true
```

**Published Topics:**
- `/camera/image` - Original camera image
- `/camera/depth` - Depth image (32FC1)
- `/camera/depth_colored` - Colored depth visualization
- `/camera/point_cloud` - RGB point cloud
- `/camera/camera_info` - Camera intrinsics

### 2. Multi-Camera System with Point Cloud Fusion

**Two-Camera System (Recommended):**
```bash
USE_CALIBRATION=1 ./run_2cameras_fast.sh
```

**Features:**
- 2 cameras (front + left side)
- Point cloud fusion in base_link frame
- Stable operation on Jetson Orin
- GPU memory: ~1.3 GB
- ~0.5-1.0 Hz per camera

**Four-Camera System (Experimental):**
```bash
USE_CALIBRATION=1 ./run_final_system_fast.sh
```

**⚠️ Warning**: May crash due to GPU memory exhaustion (~2.6-4.0 GB)

**Configuration Files:**
- `depth_anything_v3/config/2camera_config.yaml` - 2-camera setup
- `depth_anything_v3/config/2camera_fusion.rviz` - 2-camera RViz config
- `depth_anything_v3/config/multi_camera_fusion.rviz` - 4-camera RViz config

**Manual launch:**
```bash
source install/setup.bash

ros2 launch depth_anything_v3 gmsl_multi_camera_fusion.launch.py \
  camera_type:=standard \
  model_path:=onnx/DA3METRIC-LARGE.onnx \
  downsample_factor:=2 \
  config_file:=depth_anything_v3/config/2camera_config.yaml \
  use_calibration:=true \
  launch_rviz:=true
```

**Published Topics:**
- `/camera_0/image`, `/camera_1/image` - Camera images
- `/camera_0/depth`, `/camera_1/depth` - Depth images
- `/camera_0/point_cloud`, `/camera_1/point_cloud` - Individual point clouds
- `/fused_point_cloud` - Fused point cloud in base_link frame

**See Also:**
- [GPU_OPTIMIZATION.md](GPU_OPTIMIZATION.md) - GPU memory optimization guide
- [SYSTEM_STATUS.md](SYSTEM_STATUS.md) - System status and configuration
- [FISHEYE_CAMERA.md](FISHEYE_CAMERA.md) - Fisheye camera calibration
- [RVIZ_GUIDE.md](RVIZ_GUIDE.md) - RViz visualization guide

### 3. Camera Depth Estimation (Legacy)

**One-line launch:**
```bash
./run_camera_depth_rviz.sh
```

**Manual launch with custom parameters:**
```bash
source install/setup.bash

ros2 launch depth_anything_v3 camera_depth_rviz.launch.py \
  camera_id:=0 \
  camera_width:=640 \
  camera_height:=480 \
  publish_rate:=30.0
```

**Published Topics:**
- `/camera/image` - Original camera image
- `/camera/depth` - Depth image (32FC1)
- `/camera/depth_colored` - Colored depth visualization
- `/camera/point_cloud` - RGB point cloud
- `/camera/camera_info` - Camera intrinsics

### 4. Video Depth Estimation

**One-line launch:**
```bash
./run_video_depth.sh /path/to/video.mp4
```

**Manual launch with custom parameters:**
```bash
source install/setup.bash

ros2 launch depth_anything_v3 video_depth_rviz.launch.py \
  video_path:=/path/to/video.mp4 \
  playback_speed:=1.0 \
  loop:=true \
  scale_factor:=0.5
```

**Parameters:**
- `playback_speed`: 0.5 (slow) / 1.0 (normal) / 2.0 (fast)
- `loop`: true (loop playback) / false (play once)
- `scale_factor`: 1.0 (original) / 0.5 (half resolution, 2-3x faster)

**Published Topics:**
- `/video/image` - Video frame
- `/video/depth` - Depth image
- `/video/depth_colored` - Colored depth
- `/video/point_cloud` - Point cloud
- `/video/camera_info` - Camera info

### 5. Standalone Demo

**Launch:**
```bash
./run_standalone_demo.sh
```

**Features:**
- Real-time FPS display
- Inference time monitoring
- Frame saving with 's' key
- No ROS dependencies required

### 6. Camera Mosaic RTSP Streaming

**One-line launch:**
```bash
./run_camera_depth_rtsp.sh
```

**ROS2 launch (all-in-one):**
```bash
source install/setup.bash

ros2 launch depth_anything_v3 camera_depth_rtsp.launch.py \
  camera_id:=0 \
  camera_width:=640 \
  camera_height:=480 \
  stream_fps:=15 \
  rtsp_port:=8554 \
  rtsp_mount:=/depth \
  encoder:=auto \
  bitrate_kbps:=4000
```

**RTSP-only launch (reuse existing topics):**
```bash
source install/setup.bash

ros2 launch depth_anything_v3 depth_mosaic_rtsp.launch.py \
  input_image_topic:=/camera/image \
  input_depth_topic:=/camera/depth_colored \
  input_pointcloud_topic:=/camera/point_cloud
```

**Required packages (Ubuntu):**
```bash
sudo apt update
sudo apt install -y \
  python3-gi python3-gst-1.0 gir1.2-gst-rtsp-server-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-libav
```

**Published topic:**
- `/camera/depth_mosaic` - 2x2 mosaic image (PointCloud + Camera + Depth)

**RTSP URL:**
- Local: `rtsp://127.0.0.1:8554/depth`
- LAN: `rtsp://<jetson-ip>:8554/depth`

## ⚙️ Configuration

### Camera Resolution Settings

Edit `depth_anything_v3/src/camera_depth_node.cpp` or use launch parameters:

```bash
ros2 launch depth_anything_v3 camera_depth_rviz.launch.py \
  camera_width:=1280 \
  camera_height:=720
```

**Recommended resolutions:**
- **640x480**: Best compatibility, good performance
- **320x240**: Fastest (2-3x speed), but not all cameras support
- **1280x720**: High quality, slower inference

### Model Selection

Edit `depth_anything_v3/config/depth_anything_v3.param.yaml`:

```yaml
depth_anything_v3:
  ros__parameters:
    onnx_path: "onnx/DA3METRIC-LARGE.onnx"  # or BASE, SMALL
    precision: "fp16"  # or "fp32"
```

### Point Cloud Settings

```yaml
depth_anything_v3:
  ros__parameters:
    point_cloud_downsample_factor: 2  # 1=full, 2=half, 4=quarter
    colorize_point_cloud: true
```

### Sky Handling

```yaml
depth_anything_v3:
  ros__parameters:
    sky_threshold: 0.3  # Lower = more sky detected
    sky_depth_cap: 200.0  # Max depth for sky (meters)
```

## 🎯 Performance Tuning

### Resolution Scaling for Video

Lower resolution = faster inference:

```bash
# Original resolution (slowest, best quality)
./run_video_depth.sh video.mp4
# Select option 1: scale_factor=1.0

# Half resolution (2-3x faster, recommended for 1080p)
./run_video_depth.sh video.mp4
# Select option 3: scale_factor=0.5

# Quarter resolution (fastest, for 4K videos)
./run_video_depth.sh video.mp4
# Select option 4: scale_factor=0.25
```

### Camera Resolution Optimization

```bash
# Fast mode (320x240)
./run_camera_depth_rviz.sh
# Select option 2

# Balanced mode (640x480, recommended)
./run_camera_depth_rviz.sh
# Select option 1

# Quality mode (1280x720)
./run_camera_depth_rviz.sh
# Select option 3
```

### Model Size Selection

| Model | Size | Speed | Quality | Use Case |
|-------|------|-------|---------|----------|
| SMALL | ~200MB | Fastest | Good | Real-time on edge devices |
| BASE | ~400MB | Fast | Better | Balanced performance |
| LARGE | ~700MB | Slower | Best | High-quality offline processing |

### Performance Benchmarks

**NVIDIA Jetson Orin (CUDA 12.6, TensorRT 10.3):**
- DA3METRIC-LARGE @ 640x480: ~15-20 FPS
- DA3METRIC-BASE @ 640x480: ~25-30 FPS
- DA3METRIC-SMALL @ 320x240: ~40-50 FPS

**Desktop RTX 3080 (CUDA 12.8, TensorRT 10.9):**
- DA3METRIC-LARGE @ 1280x720: ~50-60 FPS
- DA3METRIC-BASE @ 1280x720: ~80-100 FPS

## 🔍 Troubleshooting

### Camera Not Found

```bash
# List available cameras
ls -l /dev/video*

# Test camera
v4l2-ctl --device=/dev/video0 --list-formats-ext

# Check camera permissions
sudo usermod -aG video $USER
# Log out and log back in
```

### Empty Frame Captured

Some cameras don't support 320x240 resolution:
```bash
# Use 640x480 instead
./run_camera_depth_rviz.sh
# Select option 1 (640x480)
```

### TensorRT Engine Build Fails

```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select depth_anything_v3 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check CUDA environment
echo $CUDA_HOME
echo $CUDACXX
nvcc --version
```

### RTSP Node Fails to Start (missing gi/GstRtspServer)

```bash
sudo apt update
sudo apt install -y \
  python3-gi python3-gst-1.0 gir1.2-gst-rtsp-server-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-libav
```

Then rebuild and relaunch:
```bash
cd ~/ros2_ws
colcon build --packages-select depth_anything_v3
source install/setup.bash
./run_camera_depth_rtsp.sh
```

### RTSP URL Cannot Be Opened From Another PC

1. Confirm Jetson and PC are on the same LAN.
2. Use Jetson IP address (not 127.0.0.1) on the PC.
3. Check port access:
   ```bash
   ss -ltnp | grep 8554
   ```
4. Test from Jetson local first:
   ```bash
   ffplay rtsp://127.0.0.1:8554/depth
   ```
5. If firewall is enabled, allow TCP 8554.

### RViz2 Point Cloud Not Showing

1. Check Fixed Frame is set to `base_link`
2. Verify point cloud topic: `/camera/point_cloud` or `/video/point_cloud`
3. Increase point size in PointCloud2 display settings
4. Check if points are being published:
   ```bash
   ros2 topic hz /camera/point_cloud
   ros2 topic echo /camera/point_cloud --no-arr
   ```

### Low FPS / Performance Issues

1. **Use FP16 precision** (default)
2. **Lower camera resolution** (320x240 or 640x480)
3. **Use smaller model** (BASE or SMALL)
4. **Reduce point cloud density**:
   ```yaml
   point_cloud_downsample_factor: 4  # Publish every 4th point
   ```
5. **For video: use scale_factor**:
   ```bash
   ./run_video_depth.sh video.mp4
   # Select scale_factor=0.5 or 0.25
   ```

### CUDA Out of Memory

```bash
# Use smaller model
# Edit depth_anything_v3/config/depth_anything_v3.param.yaml
onnx_path: "onnx/DA3METRIC-SMALL.onnx"

# Or reduce resolution
camera_width: 320
camera_height: 240
```

## 📡 Topics and Parameters

### ROS 2 Topics

#### Camera Mode
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image` | sensor_msgs/Image | Original camera image |
| `/camera/depth` | sensor_msgs/Image | Depth image (32FC1) |
| `/camera/depth_colored` | sensor_msgs/Image | Colored depth visualization |
| `/camera/depth_mosaic` | sensor_msgs/Image | Mosaic image for RTSP streaming |
| `/camera/point_cloud` | sensor_msgs/PointCloud2 | RGB point cloud |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |

#### Video Mode
| Topic | Type | Description |
|-------|------|-------------|
| `/video/image` | sensor_msgs/Image | Video frame |
| `/video/depth` | sensor_msgs/Image | Depth image |
| `/video/depth_colored` | sensor_msgs/Image | Colored depth |
| `/video/point_cloud` | sensor_msgs/PointCloud2 | Point cloud |
| `/video/camera_info` | sensor_msgs/CameraInfo | Camera info |

### RTSP Endpoints

- `rtsp://127.0.0.1:8554/depth` (local Jetson)
- `rtsp://<jetson-ip>:8554/depth` (other PC in LAN)

### Parameters

See [Configuration](#configuration) section for detailed parameter descriptions.

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgements

This project builds upon:
- [Depth Anything V3](https://github.com/ByteDance-Seed/depth-anything-3) - Original depth estimation model
- [ROS 2 MoGE TRT](https://github.com/ika-rwth-aachen/ros2-moge-trt) - TensorRT ROS 2 integration
- [ika-rwth-aachen](https://github.com/ika-rwth-aachen) - Institute for Automotive Engineering

## 📧 Contact

For questions or issues, please open an issue on GitHub or contact the maintainers.

## 🌟 Citation

If you use this code in your research, please cite:

```bibtex
@misc{beemelmanns2024depth,
  author = {Till Beemelmanns},
  title = {ros2-depth-anything-v3-trt: ROS2 TensorRT Node for Monocular Metric Depth estimation},
  year = {2025},
  publisher = {GitHub},
  url = {https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt}
}
```
