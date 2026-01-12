#!/bin/bash
# Legacy run script - redirects to test_single_camera_fast.sh

echo "=========================================="
echo "Legacy Script Redirect"
echo "=========================================="
echo ""
echo "This script is deprecated."
echo "Please use one of these instead:"
echo ""
echo "  1. Single camera:  ./test_single_camera_fast.sh"
echo "  2. Two cameras:    ./run_2cameras_fast.sh"
echo "  3. Video:          ./run_video_depth.sh"
echo ""
echo "Redirecting to single camera test in 3 seconds..."
echo ""

sleep 3

exec ./test_single_camera_fast.sh
