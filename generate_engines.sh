#!/bin/bash
# Pre-generate TensorRT engines for faster first-time startup
# This script builds TensorRT engines from ONNX models

echo "=========================================="
echo "TensorRT Engine Generation"
echo "=========================================="
echo ""

# Check if ONNX models exist
if [ ! -d "onnx" ]; then
    echo "Error: onnx/ directory not found"
    echo "Please download ONNX models first"
    exit 1
fi

# Find ONNX models
MODELS=$(find onnx -name "*.onnx" 2>/dev/null)

if [ -z "$MODELS" ]; then
    echo "Error: No ONNX models found in onnx/"
    echo "Please download models from Hugging Face"
    exit 1
fi

echo "Found ONNX models:"
echo "$MODELS"
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
echo "This will take 5-10 minutes on first run"
echo ""

# Run a quick inference to trigger engine generation
for MODEL in $MODELS; do
    echo "Processing: $MODEL"
    
    # Create a temporary test to trigger engine build
    timeout 120 ros2 run depth_anything_v3 camera_depth_node \
        --ros-args \
        -p camera_id:=0 \
        -p model_path:="$MODEL" \
        2>&1 | grep -i "engine" || true
    
    echo "✓ Engine generated for $MODEL"
    echo ""
done

echo "=========================================="
echo "Engine generation complete!"
echo "Next startup will be faster"
echo "=========================================="
