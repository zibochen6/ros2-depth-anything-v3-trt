#!/bin/bash
# Test single camera with lower resolution for faster inference
# Optional: Set USE_CALIBRATION=1 to use calibrated camera parameters

echo "=========================================="
echo "Single Camera Fast Test"
echo "=========================================="
echo ""

# Kill existing
echo "Cleaning up..."
pkill -9 -f camera_depth_node 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
sleep 2

# Source
source install/setup.bash

echo ""
echo "Configuration:"
echo "  - Camera: 0"
echo "  - Camera resolution: 1920x1536"
echo "  - Downsample factor: 2x (960x768 for inference)"
echo "  - Depth estimation: 518x518 (TensorRT model input)"

# Check if using calibration
if [ "${USE_CALIBRATION}" = "1" ]; then
    echo "  - Camera parameters: CALIBRATED (FISHEYE)"
    echo "    fx=824.15, fy=823.66, cx=958.28, cy=767.39"
    echo "    Fisheye distortion: D=[1.49, -13.39, 21.41, 3.82]"
    CALIB_PARAMS="use_calibration:=true fx:=824.147361 fy:=823.660879 cx:=958.275200 cy:=767.389372 k1:=1.486308 k2:=-13.386609 p1:=21.409334 p2:=3.817858 k3:=0.0"
else
    echo "  - Camera parameters: ESTIMATED (60° FOV)"
    echo "    Tip: Set USE_CALIBRATION=1 to use calibrated fisheye parameters"
    CALIB_PARAMS=""
fi

echo "  - RViz: enabled"
echo ""
echo "Expected: 2-4x faster inference than full resolution"
echo ""
echo "Starting..."
echo "=========================================="
echo ""

# Run single camera with RViz and downsampling
ros2 launch depth_anything_v3 camera_depth_rviz.launch.py \
    camera_type:=standard \
    camera_id:=0 \
    model_path:=onnx/DA3METRIC-LARGE.onnx \
    publish_rate:=10.0 \
    downsample_factor:=2 \
    ${CALIB_PARAMS}


