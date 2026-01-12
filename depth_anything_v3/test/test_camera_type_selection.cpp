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

/**
 * Property-Based Test for Camera Type Selection
 * 
 * Feature: gmsl-camera-integration, Property 2: Camera Type Selection
 * Validates: Requirements 2.1, 2.2, 2.3
 * 
 * Property: For any camera configuration, when camera_type is "standard", 
 * the system should create a StandardCameraCapture instance, and when 
 * camera_type is "gmsl", the system should create a GMSLCameraCapture instance.
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include "depth_anything_v3/camera_factory.hpp"
#include "depth_anything_v3/camera_capture.hpp"
#include "depth_anything_v3/standard_camera_capture.hpp"
#include "depth_anything_v3/gmsl_camera_capture.hpp"

namespace depth_anything_v3
{
namespace test
{

// Use the factory's CameraConfig
using CameraConfig = depth_anything_v3::CameraConfig;

// Factory function to create camera capture based on type
std::unique_ptr<CameraCapture> createCameraCapture(const CameraConfig& config) {
    return CameraFactory::createCamera(config);
}

// Random configuration generator for property-based testing
class CameraConfigGenerator {
public:
    CameraConfigGenerator() : gen_(std::random_device{}()) {}
    
    CameraConfig generateStandardConfig() {
        std::uniform_int_distribution<> camera_id_dist(0, 3);
        std::uniform_int_distribution<> width_dist(320, 1920);
        std::uniform_int_distribution<> height_dist(240, 1536);
        std::uniform_int_distribution<> fps_dist(10, 60);
        
        CameraConfig config;
        config.camera_type = "standard";
        config.camera_id = camera_id_dist(gen_);
        config.device_path = "/dev/video" + std::to_string(config.camera_id);
        config.width = width_dist(gen_);
        config.height = height_dist(gen_);
        config.framerate = fps_dist(gen_);
        config.format = "UYVY";
        config.sensor_mode = 0;
        
        return config;
    }
    
    CameraConfig generateGMSLConfig() {
        std::uniform_int_distribution<> device_dist(0, 3);
        std::uniform_int_distribution<> width_dist(640, 1920);
        std::uniform_int_distribution<> height_dist(480, 1536);
        std::uniform_int_distribution<> fps_dist(10, 60);
        std::uniform_int_distribution<> sensor_dist(0, 3);
        
        CameraConfig config;
        config.camera_type = "gmsl";
        config.camera_id = -1;  // Not used for GMSL
        config.device_path = "/dev/video" + std::to_string(device_dist(gen_));
        config.width = width_dist(gen_);
        config.height = height_dist(gen_);
        config.framerate = fps_dist(gen_);
        config.format = "UYVY";
        config.sensor_mode = sensor_dist(gen_);
        
        return config;
    }
    
private:
    std::mt19937 gen_;
};

/**
 * Property Test: Standard camera type creates StandardCameraCapture
 * 
 * For any valid standard camera configuration, the factory should create
 * a StandardCameraCapture instance (not nullptr).
 */
TEST(CameraTypeSelectionProperty, StandardTypeCreatesStandardCapture) {
    CameraConfigGenerator generator;
    const int NUM_ITERATIONS = 100;
    
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        // Generate random standard camera configuration
        CameraConfig config = generator.generateStandardConfig();
        
        // Create camera capture
        auto capture = createCameraCapture(config);
        
        // Verify that a capture object was created (not nullptr)
        ASSERT_NE(capture, nullptr) 
            << "Failed to create StandardCameraCapture for config: "
            << "camera_id=" << config.camera_id 
            << ", width=" << config.width 
            << ", height=" << config.height;
        
        // Verify it's actually a StandardCameraCapture by checking dynamic type
        auto* standard_capture = dynamic_cast<StandardCameraCapture*>(capture.get());
        ASSERT_NE(standard_capture, nullptr)
            << "Created capture is not a StandardCameraCapture instance";
    }
}

/**
 * Property Test: GMSL camera type creates GMSLCameraCapture
 * 
 * For any valid GMSL camera configuration, the factory should create
 * a GMSLCameraCapture instance.
 */
TEST(CameraTypeSelectionProperty, GMSLTypeCreatesGMSLCapture) {
    CameraConfigGenerator generator;
    const int NUM_ITERATIONS = 100;
    
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        // Generate random GMSL camera configuration
        CameraConfig config = generator.generateGMSLConfig();
        
        // Create camera capture
        auto capture = createCameraCapture(config);
        
        // Verify that a capture object was created (not nullptr)
        ASSERT_NE(capture, nullptr)
            << "Failed to create GMSLCameraCapture for config: "
            << "device_path=" << config.device_path
            << ", width=" << config.width
            << ", height=" << config.height;
        
        // Verify it's actually a GMSLCameraCapture by checking dynamic type
        auto* gmsl_capture = dynamic_cast<GMSLCameraCapture*>(capture.get());
        ASSERT_NE(gmsl_capture, nullptr)
            << "Created capture is not a GMSLCameraCapture instance";
    }
}

/**
 * Property Test: Invalid camera type returns nullptr
 * 
 * For any invalid camera type string, the factory should return nullptr.
 */
TEST(CameraTypeSelectionProperty, InvalidTypeReturnsNull) {
    std::vector<std::string> invalid_types = {
        "", "invalid", "usb", "webcam", "STANDARD", "GMSL", "Standard", "Gmsl"
    };
    
    for (const auto& invalid_type : invalid_types) {
        CameraConfig config;
        config.camera_type = invalid_type;
        config.camera_id = 0;
        config.device_path = "/dev/video0";
        config.width = 640;
        config.height = 480;
        config.framerate = 30;
        config.format = "UYVY";
        config.sensor_mode = 0;
        
        auto capture = createCameraCapture(config);
        
        ASSERT_EQ(capture, nullptr)
            << "Invalid camera type '" << invalid_type << "' should return nullptr";
    }
}

/**
 * Property Test: Camera type selection is case-sensitive
 * 
 * The camera type string matching should be case-sensitive.
 */
TEST(CameraTypeSelectionProperty, CameraTypeIsCaseSensitive) {
    CameraConfig config;
    config.camera_id = 0;
    config.device_path = "/dev/video0";
    config.width = 640;
    config.height = 480;
    config.framerate = 30;
    config.format = "UYVY";
    config.sensor_mode = 0;
    
    // Test uppercase variants
    config.camera_type = "STANDARD";
    ASSERT_EQ(createCameraCapture(config), nullptr)
        << "Uppercase 'STANDARD' should not match";
    
    config.camera_type = "GMSL";
    ASSERT_EQ(createCameraCapture(config), nullptr)
        << "Uppercase 'GMSL' should not match";
    
    // Test mixed case variants
    config.camera_type = "Standard";
    ASSERT_EQ(createCameraCapture(config), nullptr)
        << "Mixed case 'Standard' should not match";
    
    config.camera_type = "Gmsl";
    ASSERT_EQ(createCameraCapture(config), nullptr)
        << "Mixed case 'Gmsl' should not match";
}

}  // namespace test
}  // namespace depth_anything_v3

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
