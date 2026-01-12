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

#ifndef DEPTH_ANYTHING_V3__CAMERA_FACTORY_HPP_
#define DEPTH_ANYTHING_V3__CAMERA_FACTORY_HPP_

#include "depth_anything_v3/camera_capture.hpp"
#include "depth_anything_v3/standard_camera_capture.hpp"
#include "depth_anything_v3/gmsl_camera_capture.hpp"
#include <memory>
#include <string>

namespace depth_anything_v3
{

/**
 * @struct CameraConfig
 * @brief Configuration parameters for camera creation
 */
struct CameraConfig {
    std::string camera_type;  // "standard" or "gmsl"
    int camera_id;            // For standard cameras
    std::string device_path;  // For GMSL cameras (e.g., "/dev/video0")
    int width;
    int height;
    int framerate;
    std::string format;       // For GMSL cameras (e.g., "UYVY")
    int sensor_mode;          // For GMSL cameras
};

/**
 * @class CameraFactory
 * @brief Factory class for creating camera capture instances
 * 
 * This factory creates the appropriate CameraCapture implementation
 * based on the camera_type specified in the configuration.
 */
class CameraFactory {
public:
    /**
     * @brief Create a camera capture instance based on configuration
     * @param config Camera configuration parameters
     * @return Unique pointer to CameraCapture instance, or nullptr if invalid type
     */
    static std::unique_ptr<CameraCapture> createCamera(const CameraConfig& config);
};

}  // namespace depth_anything_v3

#endif  // DEPTH_ANYTHING_V3__CAMERA_FACTORY_HPP_
