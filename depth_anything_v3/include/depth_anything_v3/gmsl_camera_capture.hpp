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

#ifndef DEPTH_ANYTHING_V3__GMSL_CAMERA_CAPTURE_HPP_
#define DEPTH_ANYTHING_V3__GMSL_CAMERA_CAPTURE_HPP_

#include "depth_anything_v3/camera_capture.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <string>

namespace depth_anything_v3
{

/**
 * @class GMSLCameraCapture
 * @brief Camera capture implementation for GMSL cameras using GStreamer
 * 
 * This class uses GStreamer pipelines to capture frames from GMSL cameras.
 * It constructs a GStreamer pipeline with v4l2src and uses OpenCV's
 * VideoCapture with CAP_GSTREAMER backend.
 */
class GMSLCameraCapture : public CameraCapture
{
public:
  /**
   * @brief Construct a GMSLCameraCapture
   * @param device_path V4L2 device path (e.g., "/dev/video0")
   * @param width Desired frame width
   * @param height Desired frame height
   * @param framerate Desired frames per second
   * @param format Video format (e.g., "UYVY")
   * @param sensor_mode Sensor mode for v4l2-ctl configuration
   */
  GMSLCameraCapture(
    const std::string& device_path,
    int width,
    int height,
    int framerate = 30,
    const std::string& format = "UYVY",
    int sensor_mode = 0);

  /**
   * @brief Destructor - releases camera resources
   */
  ~GMSLCameraCapture() override;

  bool initialize() override;
  bool read(cv::Mat& frame) override;
  bool isOpened() const override;
  void release() override;
  int getWidth() const override;
  int getHeight() const override;

private:
  /**
   * @brief Build GStreamer pipeline string
   * @return Complete GStreamer pipeline string
   */
  std::string buildGStreamerPipeline() const;

  /**
   * @brief Apply v4l2-ctl settings before opening camera
   * @return true if settings applied successfully, false otherwise
   */
  bool applyV4L2Settings();

  std::string device_path_;
  int width_;
  int height_;
  int framerate_;
  std::string format_;
  int sensor_mode_;
  cv::VideoCapture cap_;
};

}  // namespace depth_anything_v3

#endif  // DEPTH_ANYTHING_V3__GMSL_CAMERA_CAPTURE_HPP_
