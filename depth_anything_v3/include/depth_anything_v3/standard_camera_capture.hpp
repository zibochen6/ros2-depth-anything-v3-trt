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

#ifndef DEPTH_ANYTHING_V3__STANDARD_CAMERA_CAPTURE_HPP_
#define DEPTH_ANYTHING_V3__STANDARD_CAMERA_CAPTURE_HPP_

#include "depth_anything_v3/camera_capture.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

namespace depth_anything_v3
{

/**
 * @class StandardCameraCapture
 * @brief Camera capture implementation for standard USB/V4L2 cameras
 * 
 * This class wraps OpenCV's VideoCapture to provide camera access
 * for standard USB cameras and V4L2 devices.
 */
class StandardCameraCapture : public CameraCapture
{
public:
  /**
   * @brief Construct a StandardCameraCapture
   * @param camera_id Camera device ID (e.g., 0 for /dev/video0)
   * @param width Desired frame width
   * @param height Desired frame height
   * @param fps Desired frames per second
   */
  StandardCameraCapture(int camera_id, int width, int height, int fps = 30);

  /**
   * @brief Destructor - releases camera resources
   */
  ~StandardCameraCapture() override;

  bool initialize() override;
  bool read(cv::Mat& frame) override;
  bool isOpened() const override;
  void release() override;
  int getWidth() const override;
  int getHeight() const override;

private:
  int camera_id_;
  int requested_width_;
  int requested_height_;
  int fps_;
  int actual_width_;
  int actual_height_;
  cv::VideoCapture cap_;
};

}  // namespace depth_anything_v3

#endif  // DEPTH_ANYTHING_V3__STANDARD_CAMERA_CAPTURE_HPP_
