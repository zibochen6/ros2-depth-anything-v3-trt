// Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// ROS 2 node for camera capture and depth estimation with point cloud publishing
// Usage: ros2 run depth_anything_v3 camera_depth_node

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <thread>
#include <chrono>

#include "depth_anything_v3/tensorrt_depth_anything.hpp"

class CameraDepthNode : public rclcpp::Node
{
public:
    CameraDepthNode() : Node("camera_depth_node")
    {
        // Declare parameters
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("model_path", "onnx/DA3METRIC-LARGE.onnx");
        this->declare_parameter("frame_id", "camera_link");
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("camera_width", 640);
        this->declare_parameter("camera_height", 480);
        
        camera_id_ = this->get_parameter("camera_id").as_int();
        model_path_ = this->get_parameter("model_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        camera_width_ = this->get_parameter("camera_width").as_int();
        camera_height_ = this->get_parameter("camera_height").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Camera ID: %d", camera_id_);
        RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", camera_width_, camera_height_);
        
        // Create publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/input/image", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/output/depth_image", 10);
        depth_colored_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/output/depth_colored", 10);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/output/point_cloud", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/camera_info", 10);
        
        // Initialize camera
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize depth estimator
        if (!initDepthEstimator()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize depth estimator");
            rclcpp::shutdown();
            return;
        }
        
        // Create timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&CameraDepthNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Camera depth node started");
    }
    
    ~CameraDepthNode()
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    bool initCamera()
    {
        RCLCPP_INFO(this->get_logger(), "Opening camera...");
        cap_.open(camera_id_);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera %d", camera_id_);
            return false;
        }
        
        // Set camera properties with fallback
        RCLCPP_INFO(this->get_logger(), "Requesting resolution: %dx%d", camera_width_, camera_height_);
        
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
        cap_.set(cv::CAP_PROP_FPS, 30);
        
        // Get actual resolution (may differ from requested)
        frame_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        frame_height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        
        if (frame_width_ != camera_width_ || frame_height_ != camera_height_) {
            RCLCPP_WARN(this->get_logger(), 
                "Camera does not support %dx%d, using %dx%d instead",
                camera_width_, camera_height_, frame_width_, frame_height_);
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
            RCLCPP_ERROR(this->get_logger(), "Cannot capture frames from camera");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "✓ Camera opened successfully: %dx%d", frame_width_, frame_height_);
        
        // Create camera info
        camera_info_ = createCameraInfo(frame_width_, frame_height_);
        
        return true;
    }
    
    bool initDepthEstimator()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing depth estimator...");
        RCLCPP_INFO(this->get_logger(), "This may take a few minutes on first run...");
        
        try {
            tensorrt_common::BuildConfig build_config;
            build_config.clip_value = 0.0f;
            build_config.calib_type_str = "MinMax";
            
            depth_estimator_ = std::make_unique<depth_anything_v3::TensorRTDepthAnything>(
                model_path_,
                "fp16",
                build_config,
                false,
                "",
                tensorrt_common::BatchConfig{1, 1, 1},
                (1ULL << 30)
            );
            
            depth_estimator_->initPreprocessBuffer(frame_width_, frame_height_);
            depth_estimator_->setSkyThreshold(0.3f);
            
            RCLCPP_INFO(this->get_logger(), "Depth estimator initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing depth estimator: %s", e.what());
            return false;
        }
    }
    
    sensor_msgs::msg::CameraInfo createCameraInfo(int width, int height)
    {
        sensor_msgs::msg::CameraInfo camera_info;
        
        const double focal_length = width / (2.0 * std::tan(60.0 * M_PI / 180.0 / 2.0));
        
        camera_info.width = width;
        camera_info.height = height;
        
        camera_info.k[0] = focal_length;
        camera_info.k[2] = width / 2.0;
        camera_info.k[4] = focal_length;
        camera_info.k[5] = height / 2.0;
        camera_info.k[8] = 1.0;
        
        camera_info.d.resize(5, 0.0);
        
        camera_info.r[0] = 1.0;
        camera_info.r[4] = 1.0;
        camera_info.r[8] = 1.0;
        
        camera_info.p[0] = focal_length;
        camera_info.p[2] = width / 2.0;
        camera_info.p[5] = focal_length;
        camera_info.p[6] = height / 2.0;
        camera_info.p[10] = 1.0;
        
        camera_info.header.frame_id = frame_id_;
        
        return camera_info;
    }
    
    cv::Mat applyDepthColormap(const cv::Mat& depth_image)
    {
        cv::Mat depth_normalized, depth_8u, colored;
        
        // Find min and max depth for better contrast
        double min_depth, max_depth;
        cv::minMaxLoc(depth_image, &min_depth, &max_depth);
        
        // Use adaptive range, but limit to reasonable values
        min_depth = std::max(0.5, min_depth);  // At least 0.5m
        max_depth = std::min(20.0, max_depth);  // At most 20m
        
        // Normalize to 0-255 range
        depth_image.convertTo(depth_normalized, CV_64F);
        depth_normalized = (depth_normalized - min_depth) / (max_depth - min_depth) * 255.0;
        depth_normalized.setTo(0, depth_normalized < 0);
        depth_normalized.setTo(255, depth_normalized > 255);
        depth_normalized.convertTo(depth_8u, CV_8U);
        
        // Apply TURBO colormap (better than JET for depth)
        cv::applyColorMap(depth_8u, colored, cv::COLORMAP_TURBO);
        
        return colored;
    }
    
    void createPointCloud(
        const cv::Mat& depth_image,
        const cv::Mat& rgb_image,
        sensor_msgs::msg::PointCloud2& cloud_msg)
    {
        const double fx = camera_info_.k[0];
        const double fy = camera_info_.k[4];
        const double cx = camera_info_.k[2];
        const double cy = camera_info_.k[5];
        
        const int height = depth_image.rows;
        const int width = depth_image.cols;
        const int downsample = 1;  // No downsampling for better quality
        
        // Setup PointCloud2 message
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = frame_id_;
        cloud_msg.height = 1;
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        
        // Count valid points
        int valid_points = 0;
        for (int v = 0; v < height; v += downsample) {
            for (int u = 0; u < width; u += downsample) {
                float depth = depth_image.at<float>(v, u);
                if (depth > 0.0f && std::isfinite(depth) && depth < 100.0f) {
                    valid_points++;
                }
            }
        }
        
        modifier.resize(valid_points);
        cloud_msg.width = valid_points;
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
        
        for (int v = 0; v < height; v += downsample) {
            for (int u = 0; u < width; u += downsample) {
                float depth = depth_image.at<float>(v, u);
                
                if (depth <= 0.0f || !std::isfinite(depth) || depth > 100.0f) {
                    continue;
                }
                
                // Camera coordinate system: X=right, Y=down, Z=forward
                // Convert to ROS coordinate system for better visualization
                *iter_z = depth;                          // Z: forward (depth)
                *iter_x = -(u - cx) * depth / fx;        // X: left (negated for correct orientation)
                *iter_y = -(v - cy) * depth / fy;        // Y: up (negated for correct orientation)
                
                if (!rgb_image.empty()) {
                    cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(v, u);
                    *iter_r = rgb[0];  // RGB image, so R is at index 0
                    *iter_g = rgb[1];
                    *iter_b = rgb[2];
                } else {
                    *iter_r = 255;
                    *iter_g = 255;
                    *iter_b = 255;
                }
                
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_r;
                ++iter_g;
                ++iter_b;
            }
        }
    }
    
    void timerCallback()
    {
        cv::Mat frame;
        cap_ >> frame;
        
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }
        
        auto stamp = this->now();
        
        // Convert to RGB
        cv::Mat frame_rgb;
        cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);
        
        // Run depth estimation
        std::vector<cv::Mat> images = {frame_rgb};
        bool success = depth_estimator_->doInference(images, camera_info_, 1, false);
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Inference failed");
            return;
        }
        
        const cv::Mat& depth_image = depth_estimator_->getDepthImage();
        
        // Publish input image
        auto input_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        input_msg->header.stamp = stamp;
        input_msg->header.frame_id = frame_id_;
        image_pub_->publish(*input_msg);
        
        // Publish depth image
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_image).toImageMsg();
        depth_msg->header.stamp = stamp;
        depth_msg->header.frame_id = frame_id_;
        depth_pub_->publish(*depth_msg);
        
        // Publish colored depth image
        cv::Mat depth_colored = applyDepthColormap(depth_image);
        auto depth_colored_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", depth_colored).toImageMsg();
        depth_colored_msg->header.stamp = stamp;
        depth_colored_msg->header.frame_id = frame_id_;
        depth_colored_pub_->publish(*depth_colored_msg);
        
        // Publish point cloud
        sensor_msgs::msg::PointCloud2 cloud_msg;
        createPointCloud(depth_image, frame_rgb, cloud_msg);
        pointcloud_pub_->publish(cloud_msg);
        
        // Publish camera info
        camera_info_.header.stamp = stamp;
        camera_info_pub_->publish(camera_info_);
    }
    
    // Parameters
    int camera_id_;
    std::string model_path_;
    std::string frame_id_;
    int frame_width_;
    int frame_height_;
    int camera_width_;
    int camera_height_;
    
    // Camera
    cv::VideoCapture cap_;
    sensor_msgs::msg::CameraInfo camera_info_;
    
    // Depth estimator
    std::unique_ptr<depth_anything_v3::TensorRTDepthAnything> depth_estimator_;
    
    // ROS 2
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_colored_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraDepthNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
