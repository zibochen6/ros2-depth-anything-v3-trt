#!/bin/bash

# Depth Anything V3 Video Depth Estimation Launch Script

echo "=========================================="
echo "Depth Anything V3 Video Depth Estimation + RViz2"
echo "=========================================="
echo ""

# Check environment
echo "[1/3] Checking environment..."

# Check if ROS 2 environment is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 environment not found"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if workspace is built
if [ ! -f "install/setup.bash" ]; then
    echo "Error: Built workspace not found"
    echo "Please run: colcon build --packages-select depth_anything_v3"
    exit 1
fi

source install/setup.bash

echo "✓ Environment check complete"
echo ""

# Get video path
echo "[2/3] Select video file..."

if [ -z "$1" ]; then
    echo "Enter video file path:"
    read -p "Video path: " VIDEO_PATH
else
    VIDEO_PATH="$1"
fi

# Check if video file exists
if [ ! -f "$VIDEO_PATH" ]; then
    echo "Error: Video file does not exist: $VIDEO_PATH"
    exit 1
fi

echo "✓ Video file: $VIDEO_PATH"
echo ""

# Select playback speed
echo "Select playback speed:"
echo "  1) 0.5x (Slow)"
echo "  2) 1.0x (Normal speed, default)"
echo "  3) 2.0x (Fast)"
echo ""
read -p "Choose (1/2/3, default: 2): " speed_choice

case $speed_choice in
    1)
        SPEED=0.5
        echo "✓ Playback speed: 0.5x"
        ;;
    2)
        SPEED=1.0
        echo "✓ Playback speed: 1.0x"
        ;;
    3)
        SPEED=2.0
        echo "✓ Playback speed: 2.0x"
        ;;
    *)
        SPEED=1.0
        echo "✓ Using default speed: 1.0x"
        ;;
esac
echo ""

# Select loop mode
echo "Enable loop playback?"
echo "  1) Yes (default)"
echo "  2) No"
echo ""
read -p "Choose (1/2, default: 1): " loop_choice

case $loop_choice in
    2)
        LOOP=false
        echo "✓ Loop playback: Disabled"
        ;;
    *)
        LOOP=true
        echo "✓ Loop playback: Enabled"
        ;;
esac
echo ""

# Select resolution scaling
echo "Select resolution scaling (lower resolution = faster inference):"
echo "  1) 1.0 (Original resolution, default)"
echo "  2) 0.75 (75%)"
echo "  3) 0.5 (50%, recommended for high-res videos)"
echo "  4) 0.25 (25%, fastest)"
echo ""
read -p "Choose (1/2/3/4, default: 1): " scale_choice

case $scale_choice in
    2)
        SCALE=0.75
        echo "✓ Resolution scaling: 75%"
        ;;
    3)
        SCALE=0.5
        echo "✓ Resolution scaling: 50%"
        ;;
    4)
        SCALE=0.25
        echo "✓ Resolution scaling: 25%"
        ;;
    *)
        SCALE=1.0
        echo "✓ Using original resolution"
        ;;
esac
echo ""

# Launch
echo "[3/3] Starting video depth estimation..."
echo "=========================================="
echo "Video: $VIDEO_PATH"
echo "Speed: ${SPEED}x"
echo "Loop: $LOOP"
echo "Resolution scaling: ${SCALE}"
echo ""
echo "Controls:"
echo "  - Rotate and zoom in RViz2 to view 3D point cloud"
echo "  - Press Ctrl+C to stop"
echo "=========================================="
echo ""

ros2 launch depth_anything_v3 video_depth_rviz.launch.py \
  video_path:="$VIDEO_PATH" \
  playback_speed:=$SPEED \
  loop:=$LOOP \
  scale_factor:=$SCALE
