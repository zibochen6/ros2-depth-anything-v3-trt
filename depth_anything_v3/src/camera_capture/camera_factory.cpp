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

#include "depth_anything_v3/camera_factory.hpp"
#include <iostream>

namespace depth_anything_v3
{

std::unique_ptr<CameraCapture> CameraFactory::createCamera(const CameraConfig& config)
{
    if (config.camera_type == "standard") {
        std::cout << "[CameraFactory] Creating StandardCameraCapture" << std::endl;
        return std::make_unique<StandardCameraCapture>(
            config.camera_id,
            config.width,
            config.height,
            config.framerate
        );
    } else if (config.camera_type == "gmsl") {
        std::cout << "[CameraFactory] Creating GMSLCameraCapture" << std::endl;
        return std::make_unique<GMSLCameraCapture>(
            config.device_path,
            config.width,
            config.height,
            config.framerate,
            config.format,
            config.sensor_mode
        );
    } else {
        std::cerr << "[CameraFactory] Error: Invalid camera_type '" << config.camera_type << "'" 
                  << std::endl;
        std::cerr << "[CameraFactory] Valid types are: 'standard' or 'gmsl'" << std::endl;
        return nullptr;
    }
}

}  // namespace depth_anything_v3
