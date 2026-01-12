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

#ifndef DEPTH_ANYTHING_V3__CAMERA_CAPTURE_HPP_
#define DEPTH_ANYTHING_V3__CAMERA_CAPTURE_HPP_

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

namespace depth_anything_v3
{

/**
 * @class CameraCapture
 * @brief Abstract interface for camera capture implementations
 * 
 * This interface provides a unified API for different camera types,
 * allowing the system to work with standard USB cameras, GMSL cameras,
 * or other camera hardware without changing the application code.
 */
class CameraCapture
{
public:
  virtual ~CameraCapture() = default;

  /**
   * @brief Initialize the camera capture
   * @return true if initialization successful, false otherwise
   */
  virtual bool initialize() = 0;

  /**
   * @brief Read a frame from the camera
   * @param[out] frame The captured frame
   * @return true if frame captured successfully, false otherwise
   */
  virtual bool read(cv::Mat& frame) = 0;

  /**
   * @brief Check if camera is opened and ready
   * @return true if camera is opened, false otherwise
   */
  virtual bool isOpened() const = 0;

  /**
   * @brief Release camera resources
   */
  virtual void release() = 0;

  /**
   * @brief Get the actual frame width
   * @return Frame width in pixels
   */
  virtual int getWidth() const = 0;

  /**
   * @brief Get the actual frame height
   * @return Frame height in pixels
   */
  virtual int getHeight() const = 0;
};

}  // namespace depth_anything_v3

#endif  // DEPTH_ANYTHING_V3__CAMERA_CAPTURE_HPP_
