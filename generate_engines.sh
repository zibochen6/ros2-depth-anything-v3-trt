#!/bin/bash
# Pre-generate TensorRT engines for faster first-time startup
# This script builds TensorRT engines from ONNX models

set -euo pipefail

MODELS_DIR="${1:-onnx}"

echo "=========================================="
echo "TensorRT Engine Generation"
echo "=========================================="
echo ""

# Check if ONNX models exist
if [ ! -d "${MODELS_DIR}" ]; then
    echo "Error: models directory not found: ${MODELS_DIR}"
    echo "Please provide a directory containing .onnx files"
    exit 1
fi

# Find ONNX models
MODELS=$(find "${MODELS_DIR}" -maxdepth 1 -name "*.onnx" -type f 2>/dev/null)

if [ -z "${MODELS}" ]; then
    echo "Error: No ONNX models found in ${MODELS_DIR}"; 
    echo "Please download or export models first"
    exit 1
fi

echo "Found ONNX models:"
echo "${MODELS}"
echo ""

# Source ROS environment
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: Workspace not built"
    echo "Please run: colcon build --packages-select depth_anything_v3"
    exit 1
fi

echo "Generating TensorRT engines..."
echo "This will take several minutes on first run"
echo ""

# Use the dedicated engine generator (does not require a camera)
ros2 run depth_anything_v3 generate_engines "${MODELS_DIR}"

echo ""
ENGINE_FILES=$(find "${MODELS_DIR}" -maxdepth 1 -name "*.engine" -type f 2>/dev/null || true)
if [ -z "${ENGINE_FILES}" ]; then
    echo "Error: No .engine files were generated in ${MODELS_DIR}"
    echo "Check TensorRT/CUDA logs above for errors"
    exit 1
fi

echo "Generated engine files:"
echo "${ENGINE_FILES}"
echo ""

echo "=========================================="
echo "Engine generation complete!"
echo "Next startup will be faster"
echo "=========================================="

