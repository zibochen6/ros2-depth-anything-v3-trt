#!/bin/bash

# Camera depth + mosaic RTSP launcher
# Optional env vars:
#   CAMERA_ID, CAMERA_WIDTH, CAMERA_HEIGHT, DOWNSAMPLE_FACTOR
#   MODEL_PATH, PUBLISH_RATE
#   USE_CALIBRATION=1, CAMERA_INFO_FILE=/path/to/camera_info.yaml
#   ENABLE_UNDISTORTION=1, UNDISTORTION_BALANCE=0.0..1.0
#   STREAM_FPS, RTSP_PORT, RTSP_MOUNT, ENCODER, BITRATE_KBPS
#   PANEL_WIDTH, PANEL_HEIGHT, MAX_POINTS, POINT_RADIUS

echo "=========================================="
echo "Camera + Depth + PointCloud RTSP Stream"
echo "=========================================="
echo ""

if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 environment not found"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ ! -f "install/setup.bash" ]; then
    echo "Error: Built workspace not found"
    echo "Please run: colcon build --packages-select depth_anything_v3"
    exit 1
fi

source install/setup.bash

CAMERA_ID="${CAMERA_ID:-0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
DOWNSAMPLE_FACTOR="${DOWNSAMPLE_FACTOR:-1}"
MODEL_PATH="${MODEL_PATH:-onnx/DA3METRIC-LARGE.onnx}"
PUBLISH_RATE="${PUBLISH_RATE:-10.0}"

STREAM_FPS="${STREAM_FPS:-15}"
RTSP_PORT="${RTSP_PORT:-8554}"
RTSP_MOUNT="${RTSP_MOUNT:-/depth}"
ENCODER="${ENCODER:-auto}"
BITRATE_KBPS="${BITRATE_KBPS:-4000}"
PANEL_WIDTH="${PANEL_WIDTH:--1}"
PANEL_HEIGHT="${PANEL_HEIGHT:--1}"
MAX_POINTS="${MAX_POINTS:-30000}"
POINT_RADIUS="${POINT_RADIUS:-1}"

CALIB_PARAMS="use_calibration:=false"
if [ -n "${CAMERA_INFO_FILE}" ] && [ -f "${CAMERA_INFO_FILE}" ]; then
    CALIB_PARAMS="camera_info_file:=${CAMERA_INFO_FILE}"
elif [ "${USE_CALIBRATION}" = "1" ]; then
    CALIB_PARAMS="use_calibration:=true fx:=824.147361 fy:=823.660879 cx:=958.275200 cy:=767.389372 k1:=1.486308 k2:=-13.386609 p1:=21.409334 p2:=3.817858 k3:=0.0"
fi

UNDISTORT_PARAMS="enable_undistortion:=false"
if [ "${ENABLE_UNDISTORTION}" = "1" ]; then
    BALANCE="${UNDISTORTION_BALANCE:-0.0}"
    UNDISTORT_PARAMS="enable_undistortion:=true undistortion_balance:=${BALANCE}"
fi

echo "Configuration:"
echo "  - Camera: /dev/video${CAMERA_ID}"
echo "  - Resolution: ${CAMERA_WIDTH}x${CAMERA_HEIGHT}"
echo "  - Downsample factor: ${DOWNSAMPLE_FACTOR}"
echo "  - Publish rate: ${PUBLISH_RATE} Hz"
echo "  - Stream FPS: ${STREAM_FPS}"
echo "  - RTSP: rtsp://127.0.0.1:${RTSP_PORT}${RTSP_MOUNT}"
echo "  - LAN : rtsp://<jetson-ip>:${RTSP_PORT}${RTSP_MOUNT}"
echo "  - Encoder: ${ENCODER} (${BITRATE_KBPS} kbps)"
echo "  - Mosaic panel: ${PANEL_WIDTH}x${PANEL_HEIGHT} (-1 means auto)"
echo "  - Point cloud: max_points=${MAX_POINTS}, point_radius=${POINT_RADIUS}"
echo ""
echo "Starting camera depth + RTSP stream..."
echo ""

ros2 launch depth_anything_v3 camera_depth_rtsp.launch.py \
    camera_type:=standard \
    camera_id:=${CAMERA_ID} \
    camera_width:=${CAMERA_WIDTH} \
    camera_height:=${CAMERA_HEIGHT} \
    model_path:=${MODEL_PATH} \
    publish_rate:=${PUBLISH_RATE} \
    downsample_factor:=${DOWNSAMPLE_FACTOR} \
    stream_fps:=${STREAM_FPS} \
    rtsp_port:=${RTSP_PORT} \
    rtsp_mount:=${RTSP_MOUNT} \
    encoder:=${ENCODER} \
    bitrate_kbps:=${BITRATE_KBPS} \
    panel_width:=${PANEL_WIDTH} \
    panel_height:=${PANEL_HEIGHT} \
    max_points:=${MAX_POINTS} \
    point_radius:=${POINT_RADIUS} \
    ${CALIB_PARAMS} \
    ${UNDISTORT_PARAMS}
