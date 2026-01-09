#!/bin/bash

# Depth Anything V3 Camera + RViz2 Launch Script

echo "=========================================="
echo "Depth Anything V3 Camera Depth Estimation + RViz2"
echo "=========================================="
echo ""

# Check environment
echo "[1/2] Checking environment..."

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

# Check camera
echo "[2/2] Checking camera device..."

CAMERA_ID=0
if [ ! -e "/dev/video${CAMERA_ID}" ]; then
    echo "Warning: Camera device /dev/video${CAMERA_ID} not found"
    echo "Available video devices:"
    ls -l /dev/video* 2>/dev/null || echo "  None"
    echo ""
    read -p "Enter camera ID (default: 0): " input_id
    if [ ! -z "$input_id" ]; then
        CAMERA_ID=$input_id
    fi
fi

echo "✓ Using camera device: /dev/video${CAMERA_ID}"
echo ""

# Select resolution
echo "Select camera resolution:"
echo "  1) 640x480  (Recommended, best compatibility)"
echo "  2) 320x240  (Faster, but some cameras don't support)"
echo "  3) 1280x720 (High quality, slower)"
echo ""
read -p "Choose (1/2/3, default: 1): " resolution_choice

case $resolution_choice in
    1)
        WIDTH=640
        HEIGHT=480
        echo "✓ Selected resolution: 640x480 (Recommended)"
        ;;
    2)
        WIDTH=320
        HEIGHT=240
        echo "✓ Selected resolution: 320x240 (Fast mode)"
        ;;
    3)
        WIDTH=1280
        HEIGHT=720
        echo "✓ Selected resolution: 1280x720 (High quality mode)"
        ;;
    *)
        WIDTH=640
        HEIGHT=480
        echo "✓ Using default resolution: 640x480 (Recommended)"
        ;;
esac
echo ""

# Launch
echo "=========================================="
echo "Launching ROS 2 nodes and RViz2..."
echo "Resolution: ${WIDTH}x${HEIGHT}"
echo ""
echo "Controls:"
echo "  - Rotate and zoom in RViz2 to view 3D point cloud"
echo "  - Press Ctrl+C to stop all nodes"
echo "=========================================="
echo ""

ros2 launch depth_anything_v3 camera_depth_rviz.launch.py \
  camera_id:=${CAMERA_ID} \
  camera_width:=${WIDTH} \
  camera_height:=${HEIGHT}
