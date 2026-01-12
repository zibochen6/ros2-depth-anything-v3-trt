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

#include "depth_anything_v3/standard_camera_capture.hpp"
#include <thread>
#include <chrono>
#include <iostream>

namespace depth_anything_v3
{

StandardCameraCapture::StandardCameraCapture(int camera_id, int width, int height, int fps)
: camera_id_(camera_id),
  requested_width_(width),
  requested_height_(height),
  fps_(fps),
  actual_width_(0),
  actual_height_(0)
{
}

StandardCameraCapture::~StandardCameraCapture()
{
  release();
}

bool StandardCameraCapture::initialize()
{
  std::cout << "[StandardCameraCapture] Opening camera " << camera_id_ << "..." << std::endl;
  
  // Use V4L2 backend directly (like test.py), not GStreamer
  cap_.open(camera_id_, cv::CAP_V4L2);
  
  if (!cap_.isOpened()) {
    std::cerr << "[StandardCameraCapture] Cannot open camera " << camera_id_ << std::endl;
    return false;
  }
  
  // Set camera properties (only if requested values are valid)
  if (requested_width_ > 0 && requested_height_ > 0) {
    std::cout << "[StandardCameraCapture] Requesting resolution: " 
              << requested_width_ << "x" << requested_height_ << std::endl;
    
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, requested_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, requested_height_);
  }
  
  if (fps_ > 0) {
    cap_.set(cv::CAP_PROP_FPS, fps_);
  }
  
  // Get actual resolution (may differ from requested)
  actual_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
  actual_height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
  
  if (actual_width_ != requested_width_ || actual_height_ != requested_height_) {
    std::cout << "[StandardCameraCapture] Warning: Camera does not support " 
              << requested_width_ << "x" << requested_height_ 
              << ", using " << actual_width_ << "x" << actual_height_ << " instead" << std::endl;
  }
  
  // Verify we can capture a frame
  cv::Mat test_frame;
  for (int i = 0; i < 5; i++) {
    cap_ >> test_frame;
    if (!test_frame.empty()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  if (test_frame.empty()) {
    std::cerr << "[StandardCameraCapture] Cannot capture frames from camera" << std::endl;
    return false;
  }
  
  std::cout << "[StandardCameraCapture] ✓ Camera opened successfully: " 
            << actual_width_ << "x" << actual_height_ << std::endl;
  
  return true;
}

bool StandardCameraCapture::read(cv::Mat& frame)
{
  if (!cap_.isOpened()) {
    return false;
  }
  
  cap_ >> frame;
  return !frame.empty();
}

bool StandardCameraCapture::isOpened() const
{
  return cap_.isOpened();
}

void StandardCameraCapture::release()
{
  if (cap_.isOpened()) {
    cap_.release();
  }
}

int StandardCameraCapture::getWidth() const
{
  return actual_width_;
}

int StandardCameraCapture::getHeight() const
{
  return actual_height_;
}

}  // namespace depth_anything_v3
