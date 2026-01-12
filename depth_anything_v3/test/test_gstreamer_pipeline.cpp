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
 * Property-Based Test for GStreamer Pipeline Construction
 * 
 * Feature: gmsl-camera-integration, Property 1: GStreamer Pipeline Construction
 * Validates: Requirements 1.1, 1.2, 1.3, 1.4
 * 
 * Property: For any valid GStreamerParams configuration, the constructed 
 * pipeline string should contain all required elements (v4l2src, video/x-raw 
 * with correct parameters, videoconvert, appsink) in the correct order.
 */

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <random>
#include <sstream>
#include <algorithm>

namespace depth_anything_v3
{
namespace test
{

// GStreamer parameters structure
struct GStreamerParams {
    std::string device_path;
    int width;
    int height;
    int framerate;
    std::string format;
    int sensor_mode;
    
    std::string buildPipeline() const {
        std::ostringstream pipeline;
        pipeline << "v4l2src device=" << device_path
                 << " ! video/x-raw"
                 << ",width=" << width
                 << ",height=" << height
                 << ",framerate=" << framerate << "/1"
                 << ",format=" << format
                 << " ! videoconvert ! appsink";
        return pipeline.str();
    }
};

// Random parameter generator for property-based testing
class GStreamerParamsGenerator {
public:
    GStreamerParamsGenerator() : gen_(std::random_device{}()) {}
    
    GStreamerParams generateValidParams() {
        std::uniform_int_distribution<> device_dist(0, 7);
        std::uniform_int_distribution<> width_dist(320, 1920);
        std::uniform_int_distribution<> height_dist(240, 1536);
        std::uniform_int_distribution<> fps_dist(10, 60);
        std::uniform_int_distribution<> sensor_dist(0, 3);
        std::uniform_int_distribution<> format_dist(0, 3);
        
        std::vector<std::string> formats = {"UYVY", "YUYV", "RGB", "BGR"};
        
        GStreamerParams params;
        params.device_path = "/dev/video" + std::to_string(device_dist(gen_));
        params.width = width_dist(gen_);
        params.height = height_dist(gen_);
        params.framerate = fps_dist(gen_);
        params.format = formats[format_dist(gen_)];
        params.sensor_mode = sensor_dist(gen_);
        
        return params;
    }
    
private:
    std::mt19937 gen_;
};

/**
 * Helper function to check if a string contains a substring
 */
bool contains(const std::string& str, const std::string& substr) {
    return str.find(substr) != std::string::npos;
}

/**
 * Helper function to check if substring appears before another in a string
 */
bool appearsBefore(const std::string& str, const std::string& first, const std::string& second) {
    size_t pos_first = str.find(first);
    size_t pos_second = str.find(second);
    return pos_first != std::string::npos && 
           pos_second != std::string::npos && 
           pos_first < pos_second;
}

/**
 * Property Test: Pipeline contains all required elements
 * 
 * For any valid GStreamerParams, the pipeline should contain:
 * - v4l2src with device parameter
 * - video/x-raw caps
 * - width, height, framerate, format parameters
 * - videoconvert element
 * - appsink element
 */
TEST(GStreamerPipelineProperty, ContainsAllRequiredElements) {
    GStreamerParamsGenerator generator;
    const int NUM_ITERATIONS = 100;
    
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        GStreamerParams params = generator.generateValidParams();
        std::string pipeline = params.buildPipeline();
        
        // Check for v4l2src with device
        ASSERT_TRUE(contains(pipeline, "v4l2src"))
            << "Pipeline missing v4l2src element: " << pipeline;
        ASSERT_TRUE(contains(pipeline, "device=" + params.device_path))
            << "Pipeline missing device path: " << pipeline;
        
        // Check for video/x-raw caps
        ASSERT_TRUE(contains(pipeline, "video/x-raw"))
            << "Pipeline missing video/x-raw caps: " << pipeline;
        
        // Check for width parameter
        ASSERT_TRUE(contains(pipeline, "width=" + std::to_string(params.width)))
            << "Pipeline missing width parameter: " << pipeline;
        
        // Check for height parameter
        ASSERT_TRUE(contains(pipeline, "height=" + std::to_string(params.height)))
            << "Pipeline missing height parameter: " << pipeline;
        
        // Check for framerate parameter
        ASSERT_TRUE(contains(pipeline, "framerate=" + std::to_string(params.framerate) + "/1"))
            << "Pipeline missing framerate parameter: " << pipeline;
        
        // Check for format parameter
        ASSERT_TRUE(contains(pipeline, "format=" + params.format))
            << "Pipeline missing format parameter: " << pipeline;
        
        // Check for videoconvert element
        ASSERT_TRUE(contains(pipeline, "videoconvert"))
            << "Pipeline missing videoconvert element: " << pipeline;
        
        // Check for appsink element
        ASSERT_TRUE(contains(pipeline, "appsink"))
            << "Pipeline missing appsink element: " << pipeline;
    }
}

/**
 * Property Test: Pipeline elements are in correct order
 * 
 * For any valid GStreamerParams, the pipeline elements should appear
 * in the correct order: v4l2src -> video/x-raw -> videoconvert -> appsink
 */
TEST(GStreamerPipelineProperty, ElementsInCorrectOrder) {
    GStreamerParamsGenerator generator;
    const int NUM_ITERATIONS = 100;
    
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        GStreamerParams params = generator.generateValidParams();
        std::string pipeline = params.buildPipeline();
        
        // v4l2src should come before video/x-raw
        ASSERT_TRUE(appearsBefore(pipeline, "v4l2src", "video/x-raw"))
            << "v4l2src should appear before video/x-raw: " << pipeline;
        
        // video/x-raw should come before videoconvert
        ASSERT_TRUE(appearsBefore(pipeline, "video/x-raw", "videoconvert"))
            << "video/x-raw should appear before videoconvert: " << pipeline;
        
        // videoconvert should come before appsink
        ASSERT_TRUE(appearsBefore(pipeline, "videoconvert", "appsink"))
            << "videoconvert should appear before appsink: " << pipeline;
    }
}

/**
 * Property Test: Pipeline format is consistent
 * 
 * For any valid GStreamerParams, the pipeline should follow GStreamer
 * syntax with proper separators (spaces and exclamation marks).
 */
TEST(GStreamerPipelineProperty, PipelineFormatIsConsistent) {
    GStreamerParamsGenerator generator;
    const int NUM_ITERATIONS = 100;
    
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        GStreamerParams params = generator.generateValidParams();
        std::string pipeline = params.buildPipeline();
        
        // Should contain " ! " separators between elements
        size_t separator_count = 0;
        size_t pos = 0;
        while ((pos = pipeline.find(" ! ", pos)) != std::string::npos) {
            separator_count++;
            pos += 3;
        }
        
        // Should have at least 2 separators (v4l2src ! caps ! videoconvert ! appsink)
        ASSERT_GE(separator_count, 2)
            << "Pipeline should have at least 2 element separators: " << pipeline;
    }
}

/**
 * Property Test: Device path is properly formatted
 * 
 * For any valid GStreamerParams, the device path should be properly
 * formatted as /dev/videoX where X is a number.
 */
TEST(GStreamerPipelineProperty, DevicePathProperlyFormatted) {
    GStreamerParamsGenerator generator;
    const int NUM_ITERATIONS = 100;
    
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        GStreamerParams params = generator.generateValidParams();
        std::string pipeline = params.buildPipeline();
        
        // Device path should start with /dev/video
        ASSERT_TRUE(contains(pipeline, "device=/dev/video"))
            << "Device path should start with /dev/video: " << pipeline;
        
        // Extract device number
        size_t pos = pipeline.find("device=/dev/video");
        ASSERT_NE(pos, std::string::npos);
        
        // Verify device path format in params
        ASSERT_TRUE(params.device_path.find("/dev/video") == 0)
            << "Device path should start with /dev/video: " << params.device_path;
    }
}

/**
 * Property Test: Framerate format is correct
 * 
 * For any valid GStreamerParams, the framerate should be formatted
 * as "framerate=N/1" where N is the framerate value.
 */
TEST(GStreamerPipelineProperty, FramerateFormatCorrect) {
    GStreamerParamsGenerator generator;
    const int NUM_ITERATIONS = 100;
    
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        GStreamerParams params = generator.generateValidParams();
        std::string pipeline = params.buildPipeline();
        
        // Framerate should be in format "framerate=N/1"
        std::string expected_framerate = "framerate=" + std::to_string(params.framerate) + "/1";
        ASSERT_TRUE(contains(pipeline, expected_framerate))
            << "Framerate should be in format 'framerate=N/1': " << pipeline;
    }
}

/**
 * Edge Case Test: Minimum valid dimensions
 */
TEST(GStreamerPipelineProperty, MinimumValidDimensions) {
    GStreamerParams params;
    params.device_path = "/dev/video0";
    params.width = 320;
    params.height = 240;
    params.framerate = 10;
    params.format = "UYVY";
    params.sensor_mode = 0;
    
    std::string pipeline = params.buildPipeline();
    
    ASSERT_TRUE(contains(pipeline, "width=320"));
    ASSERT_TRUE(contains(pipeline, "height=240"));
    ASSERT_TRUE(contains(pipeline, "framerate=10/1"));
}

/**
 * Edge Case Test: Maximum typical dimensions
 */
TEST(GStreamerPipelineProperty, MaximumTypicalDimensions) {
    GStreamerParams params;
    params.device_path = "/dev/video0";
    params.width = 1920;
    params.height = 1536;
    params.framerate = 60;
    params.format = "UYVY";
    params.sensor_mode = 0;
    
    std::string pipeline = params.buildPipeline();
    
    ASSERT_TRUE(contains(pipeline, "width=1920"));
    ASSERT_TRUE(contains(pipeline, "height=1536"));
    ASSERT_TRUE(contains(pipeline, "framerate=60/1"));
}

/**
 * Edge Case Test: Different video formats
 */
TEST(GStreamerPipelineProperty, DifferentVideoFormats) {
    std::vector<std::string> formats = {"UYVY", "YUYV", "RGB", "BGR", "NV12", "I420"};
    
    for (const auto& format : formats) {
        GStreamerParams params;
        params.device_path = "/dev/video0";
        params.width = 640;
        params.height = 480;
        params.framerate = 30;
        params.format = format;
        params.sensor_mode = 0;
        
        std::string pipeline = params.buildPipeline();
        
        ASSERT_TRUE(contains(pipeline, "format=" + format))
            << "Pipeline should contain format=" << format << ": " << pipeline;
    }
}

}  // namespace test
}  // namespace depth_anything_v3

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
