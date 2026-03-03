#!/bin/bash
# Single camera depth estimation launcher
# Optional: Set USB_SIMPLE=1 for local USB camera quick mode (640x480, no calibration, no undistortion)
# Optional: Set CAMERA_ID=0 to select /dev/videoX
# Optional: Set CAMERA_WIDTH, CAMERA_HEIGHT, DOWNSAMPLE_FACTOR for resolution/performance tuning
# Optional: Set MODEL_PATH=onnx/DA3METRIC-LARGE.onnx and PUBLISH_RATE=10.0
# Optional: Set USE_CALIBRATION=1 to use calibrated camera parameters
# Optional: Set CAMERA_INFO_FILE=/path/to/camera_info.yaml to load from file
# Optional: Set ENABLE_UNDISTORTION=1 to enable fisheye undistortion
# Optional: Set UNDISTORTION_BALANCE=0.0 to 1.0 (0.0=crop, 1.0=full FOV)

echo "=========================================="
echo "Single Camera Depth Test"
echo "=========================================="
echo ""

# Kill existing
echo "Cleaning up..."
pkill -9 -f camera_depth_node 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
sleep 2

# Source
source install/setup.bash

# Common launch params
CAMERA_ID="${CAMERA_ID:-0}"
MODEL_PATH="${MODEL_PATH:-onnx/DA3METRIC-LARGE.onnx}"
PUBLISH_RATE="${PUBLISH_RATE:-10.0}"

# USB simple mode: local USB camera, no calibration/undistortion
if [ "${USB_SIMPLE}" = "1" ]; then
    CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
    CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
    DOWNSAMPLE_FACTOR="${DOWNSAMPLE_FACTOR:-1}"

    echo "Configuration:"
    echo "  - Mode: USB simple"
    echo "  - Camera: ${CAMERA_ID} (/dev/video${CAMERA_ID})"
    echo "  - Camera resolution: ${CAMERA_WIDTH}x${CAMERA_HEIGHT}"
    echo "  - Downsample factor: ${DOWNSAMPLE_FACTOR}x"
    echo "  - Calibration: disabled"
    echo "  - Fisheye undistortion: disabled"
    echo "  - RViz: enabled"
    echo ""
    echo "Starting..."
    echo "=========================================="
    echo ""

    ros2 launch depth_anything_v3 camera_depth_rviz.launch.py \
        camera_type:=standard \
        camera_id:=${CAMERA_ID} \
        camera_width:=${CAMERA_WIDTH} \
        camera_height:=${CAMERA_HEIGHT} \
        model_path:=${MODEL_PATH} \
        publish_rate:=${PUBLISH_RATE} \
        downsample_factor:=${DOWNSAMPLE_FACTOR} \
        use_calibration:=false \
        enable_undistortion:=false
    exit $?
fi

CAMERA_WIDTH="${CAMERA_WIDTH:-1920}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-1536}"
DOWNSAMPLE_FACTOR="${DOWNSAMPLE_FACTOR:-2}"
INFER_WIDTH=$((CAMERA_WIDTH / DOWNSAMPLE_FACTOR))
INFER_HEIGHT=$((CAMERA_HEIGHT / DOWNSAMPLE_FACTOR))

echo "Configuration:"
echo "  - Mode: fisheye/general"
echo "  - Camera: ${CAMERA_ID} (/dev/video${CAMERA_ID})"
echo "  - Camera resolution: ${CAMERA_WIDTH}x${CAMERA_HEIGHT}"
echo "  - Downsample factor: ${DOWNSAMPLE_FACTOR}x (${INFER_WIDTH}x${INFER_HEIGHT} for inference)"
echo "  - Depth estimation: 518x518 (TensorRT model input)"

# Initialize parameters
CALIB_PARAMS=""
UNDISTORT_PARAMS=""

# Check if using calibration file
if [ -n "${CAMERA_INFO_FILE}" ] && [ -f "${CAMERA_INFO_FILE}" ]; then
    echo "  - Camera parameters: FROM FILE"
    echo "    File: ${CAMERA_INFO_FILE}"
    CALIB_PARAMS="camera_info_file:=${CAMERA_INFO_FILE}"
elif [ "${USE_CALIBRATION}" = "1" ]; then
    echo "  - Camera parameters: CALIBRATED (FISHEYE)"
    echo "    fx=824.15, fy=823.66, cx=958.28, cy=767.39"
    echo "    Fisheye distortion: D=[1.49, -13.39, 21.41, 3.82]"
    CALIB_PARAMS="use_calibration:=true fx:=824.147361 fy:=823.660879 cx:=958.275200 cy:=767.389372 k1:=1.486308 k2:=-13.386609 p1:=21.409334 p2:=3.817858 k3:=0.0"
else
    echo "  - Camera parameters: ESTIMATED (60 deg FOV)"
    echo "    Tip: Set USE_CALIBRATION=1 to use calibrated fisheye parameters"
    echo "    Or: Set CAMERA_INFO_FILE=/path/to/camera_info.yaml to load from file"
fi

# Check if undistortion is enabled
if [ "${ENABLE_UNDISTORTION}" = "1" ]; then
    # Default balance to 0.0 if not set
    BALANCE="${UNDISTORTION_BALANCE:-0.0}"
    echo "  - Fisheye undistortion: ENABLED"
    echo "    Balance: ${BALANCE} (0.0=crop, 1.0=full FOV)"
    UNDISTORT_PARAMS="enable_undistortion:=true undistortion_balance:=${BALANCE}"

    # Warn if no calibration
    if [ -z "${CALIB_PARAMS}" ]; then
        echo ""
        echo "  WARNING: Undistortion requires calibration parameters!"
        echo "  Please set USE_CALIBRATION=1 or CAMERA_INFO_FILE"
        echo ""
    fi
else
    echo "  - Fisheye undistortion: DISABLED"
    echo "    Tip: Set ENABLE_UNDISTORTION=1 to enable fisheye undistortion"
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
    camera_id:=${CAMERA_ID} \
    camera_width:=${CAMERA_WIDTH} \
    camera_height:=${CAMERA_HEIGHT} \
    model_path:=${MODEL_PATH} \
    publish_rate:=${PUBLISH_RATE} \
    downsample_factor:=${DOWNSAMPLE_FACTOR} \
    ${CALIB_PARAMS} \
    ${UNDISTORT_PARAMS}