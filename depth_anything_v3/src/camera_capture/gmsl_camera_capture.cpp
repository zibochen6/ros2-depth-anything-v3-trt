// Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "depth_anything_v3/gmsl_camera_capture.hpp"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <sys/stat.h>

namespace depth_anything_v3
{

GMSLCameraCapture::GMSLCameraCapture(
  const std::string& device_path,
  int width,
  int height,
  int framerate,
  const std::string& format,
  int sensor_mode)
: device_path_(device_path),
  width_(width),
  height_(height),
  framerate_(framerate),
  format_(format),
  sensor_mode_(sensor_mode)
{
}

GMSLCameraCapture::~GMSLCameraCapture()
{
  release();
}

std::string GMSLCameraCapture::buildGStreamerPipeline() const
{
  std::ostringstream pipeline;
  
  // Build GStreamer pipeline string for OpenCV
  // Explicitly specify YUYV format for GMSL cameras
  pipeline << "v4l2src device=" << device_path_
           << " ! video/x-raw,format=YUY2"  // YUY2 is GStreamer's name for YUYV
           << ",width=" << width_
           << ",height=" << height_
           << ",framerate=" << framerate_ << "/1"
           << " ! videoconvert"
           << " ! appsink";
  
  return pipeline.str();
}

bool GMSLCameraCapture::applyV4L2Settings()
{
  // Check if device exists
  struct stat buffer;
  if (stat(device_path_.c_str(), &buffer) != 0) {
    std::cerr << "[GMSLCameraCapture] Error: Device " << device_path_ 
              << " does not exist" << std::endl;
    std::cerr << "[GMSLCameraCapture] Hint: Run 'v4l2-ctl --list-devices' to see available devices" 
              << std::endl;
    return false;
  }
  
  // Build v4l2-ctl command
  std::ostringstream cmd;
  cmd << "v4l2-ctl -d " << device_path_
      << " --set-fmt-video=width=" << width_
      << ",height=" << height_
      << " -c sensor_mode=" << sensor_mode_
      << " 2>&1";
  
  std::cout << "[GMSLCameraCapture] Applying v4l2 settings: " << cmd.str() << std::endl;
  
  // Execute v4l2-ctl command
  FILE* pipe = popen(cmd.str().c_str(), "r");
  if (!pipe) {
    std::cerr << "[GMSLCameraCapture] Error: Failed to execute v4l2-ctl command" << std::endl;
    return false;
  }
  
  // Read command output
  char buffer_line[256];
  std::string output;
  while (fgets(buffer_line, sizeof(buffer_line), pipe) != nullptr) {
    output += buffer_line;
  }
  
  int return_code = pclose(pipe);
  
  if (return_code != 0) {
    std::cerr << "[GMSLCameraCapture] Warning: v4l2-ctl returned non-zero exit code: " 
              << return_code << std::endl;
    if (!output.empty()) {
      std::cerr << "[GMSLCameraCapture] v4l2-ctl output: " << output << std::endl;
    }
    // Don't fail here - some cameras may not support all settings
    // but the pipeline might still work
  } else {
    std::cout << "[GMSLCameraCapture] ✓ v4l2 settings applied successfully" << std::endl;
    if (!output.empty()) {
      std::cout << "[GMSLCameraCapture] v4l2-ctl output: " << output << std::endl;
    }
  }
  
  return true;
}

bool GMSLCameraCapture::initialize()
{
  std::cout << "[GMSLCameraCapture] Initializing GMSL camera..." << std::endl;
  std::cout << "[GMSLCameraCapture] Device: " << device_path_ << std::endl;
  std::cout << "[GMSLCameraCapture] Resolution: " << width_ << "x" << height_ << std::endl;
  std::cout << "[GMSLCameraCapture] Framerate: " << framerate_ << " fps" << std::endl;
  std::cout << "[GMSLCameraCapture] Format: " << format_ << std::endl;
  std::cout << "[GMSLCameraCapture] Sensor mode: " << sensor_mode_ << std::endl;
  
  // Apply v4l2 settings first
  if (!applyV4L2Settings()) {
    std::cerr << "[GMSLCameraCapture] Warning: Failed to apply v4l2 settings, continuing anyway..." 
              << std::endl;
  }
  
  // Small delay to let settings take effect
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Build GStreamer pipeline
  std::string pipeline = buildGStreamerPipeline();
  std::cout << "[GMSLCameraCapture] GStreamer pipeline: " << pipeline << std::endl;
  
  // Open camera with GStreamer backend
  cap_.open(pipeline, cv::CAP_GSTREAMER);
  
  if (!cap_.isOpened()) {
    std::cerr << "[GMSLCameraCapture] Error: Failed to open GStreamer pipeline" << std::endl;
    std::cerr << "[GMSLCameraCapture] Possible causes:" << std::endl;
    std::cerr << "[GMSLCameraCapture]   - GStreamer not installed or OpenCV not compiled with GStreamer support" 
              << std::endl;
    std::cerr << "[GMSLCameraCapture]   - Device " << device_path_ << " is busy or not accessible" 
              << std::endl;
    std::cerr << "[GMSLCameraCapture]   - Unsupported resolution or format" << std::endl;
    std::cerr << "[GMSLCameraCapture] Hint: Test manually with:" << std::endl;
    std::cerr << "[GMSLCameraCapture]   gst-launch-1.0 " << pipeline.replace(pipeline.find("appsink"), 7, "autovideosink") 
              << std::endl;
    return false;
  }
  
  // Verify we can capture a frame
  cv::Mat test_frame;
  for (int i = 0; i < 10; i++) {
    cap_ >> test_frame;
    if (!test_frame.empty()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  if (test_frame.empty()) {
    std::cerr << "[GMSLCameraCapture] Error: Cannot capture frames from camera" << std::endl;
    std::cerr << "[GMSLCameraCapture] Pipeline opened but no frames received" << std::endl;
    return false;
  }
  
  // Check if frame is all black (debugging)
  cv::Scalar mean_val = cv::mean(test_frame);
  std::cout << "[GMSLCameraCapture] Test frame mean pixel value: " 
            << "B=" << mean_val[0] << ", G=" << mean_val[1] << ", R=" << mean_val[2] << std::endl;
  
  if (mean_val[0] < 1.0 && mean_val[1] < 1.0 && mean_val[2] < 1.0) {
    std::cerr << "[GMSLCameraCapture] WARNING: Captured frame appears to be all black!" << std::endl;
    std::cerr << "[GMSLCameraCapture] This may indicate:" << std::endl;
    std::cerr << "[GMSLCameraCapture]   - Camera lens cap is on" << std::endl;
    std::cerr << "[GMSLCameraCapture]   - No video signal from camera" << std::endl;
    std::cerr << "[GMSLCameraCapture]   - Camera is not properly initialized" << std::endl;
    std::cerr << "[GMSLCameraCapture] Continuing anyway..." << std::endl;
  }
  
  // Verify frame dimensions
  if (test_frame.cols != width_ || test_frame.rows != height_) {
    std::cout << "[GMSLCameraCapture] Warning: Received frame size " 
              << test_frame.cols << "x" << test_frame.rows
              << " differs from requested " << width_ << "x" << height_ << std::endl;
    // Update dimensions to actual
    width_ = test_frame.cols;
    height_ = test_frame.rows;
  }
  
  std::cout << "[GMSLCameraCapture] ✓ GMSL camera opened successfully: " 
            << width_ << "x" << height_ << std::endl;
  
  return true;
}

bool GMSLCameraCapture::read(cv::Mat& frame)
{
  if (!cap_.isOpened()) {
    return false;
  }
  
  cap_ >> frame;
  return !frame.empty();
}

bool GMSLCameraCapture::isOpened() const
{
  return cap_.isOpened();
}

void GMSLCameraCapture::release()
{
  if (cap_.isOpened()) {
    cap_.release();
  }
}

int GMSLCameraCapture::getWidth() const
{
  return width_;
}

int GMSLCameraCapture::getHeight() const
{
  return height_;
}

}  // namespace depth_anything_v3
