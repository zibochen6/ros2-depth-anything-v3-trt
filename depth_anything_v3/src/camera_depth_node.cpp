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
#include "depth_anything_v3/camera_factory.hpp"

class CameraDepthNode : public rclcpp::Node
{
public:
    CameraDepthNode() : Node("camera_depth_node")
    {
        // Declare parameters
        this->declare_parameter("camera_type", "standard");
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("device_path", "/dev/video0");
        this->declare_parameter("model_path", "onnx/DA3METRIC-LARGE.onnx");
        this->declare_parameter("frame_id", "camera_link");
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("camera_width", 640);
        this->declare_parameter("camera_height", 480);
        this->declare_parameter("framerate", 30);
        this->declare_parameter("format", "UYVY");
        this->declare_parameter("sensor_mode", 0);
        this->declare_parameter("downsample_factor", 1);  // New parameter for downsampling
        
        // Camera calibration parameters (optional, will use estimated values if not provided)
        // Default values are for fisheye lens calibration (1920x1536)
        this->declare_parameter("use_calibration", false);
        this->declare_parameter("fx", 824.147361);
        this->declare_parameter("fy", 823.660879);
        this->declare_parameter("cx", 958.275200);
        this->declare_parameter("cy", 767.389372);
        this->declare_parameter("k1", 1.486308);    // Fisheye D[0]
        this->declare_parameter("k2", -13.386609);  // Fisheye D[1]
        this->declare_parameter("p1", 21.409334);   // Fisheye D[2]
        this->declare_parameter("p2", 3.817858);    // Fisheye D[3]
        this->declare_parameter("k3", 0.0);
        
        camera_type_ = this->get_parameter("camera_type").as_string();
        camera_id_ = this->get_parameter("camera_id").as_int();
        device_path_ = this->get_parameter("device_path").as_string();
        model_path_ = this->get_parameter("model_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        camera_width_ = this->get_parameter("camera_width").as_int();
        camera_height_ = this->get_parameter("camera_height").as_int();
        framerate_ = this->get_parameter("framerate").as_int();
        format_ = this->get_parameter("format").as_string();
        sensor_mode_ = this->get_parameter("sensor_mode").as_int();
        downsample_factor_ = this->get_parameter("downsample_factor").as_int();
        
        // Get calibration parameters
        use_calibration_ = this->get_parameter("use_calibration").as_bool();
        calib_fx_ = this->get_parameter("fx").as_double();
        calib_fy_ = this->get_parameter("fy").as_double();
        calib_cx_ = this->get_parameter("cx").as_double();
        calib_cy_ = this->get_parameter("cy").as_double();
        calib_k1_ = this->get_parameter("k1").as_double();
        calib_k2_ = this->get_parameter("k2").as_double();
        calib_p1_ = this->get_parameter("p1").as_double();
        calib_p2_ = this->get_parameter("p2").as_double();
        calib_k3_ = this->get_parameter("k3").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Camera type: %s", camera_type_.c_str());
        if (camera_type_ == "standard") {
            RCLCPP_INFO(this->get_logger(), "Camera ID: %d", camera_id_);
        } else if (camera_type_ == "gmsl") {
            RCLCPP_INFO(this->get_logger(), "Device path: %s", device_path_.c_str());
            RCLCPP_INFO(this->get_logger(), "Format: %s", format_.c_str());
            RCLCPP_INFO(this->get_logger(), "Sensor mode: %d", sensor_mode_);
        }
        RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d @ %d fps", 
                    camera_width_, camera_height_, framerate_);
        if (downsample_factor_ > 1) {
            RCLCPP_INFO(this->get_logger(), "Downsampling: %dx (for faster inference)", downsample_factor_);
        }
        if (use_calibration_) {
            RCLCPP_INFO(this->get_logger(), "Using calibrated camera parameters:");
            RCLCPP_INFO(this->get_logger(), "  fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                        calib_fx_, calib_fy_, calib_cx_, calib_cy_);
        }
        
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
        if (camera_capture_ && camera_capture_->isOpened()) {
            camera_capture_->release();
        }
    }

private:
    bool initCamera()
    {
        RCLCPP_INFO(this->get_logger(), "Opening camera...");
        
        // Create camera configuration
        depth_anything_v3::CameraConfig config;
        config.camera_type = camera_type_;
        config.camera_id = camera_id_;
        config.device_path = device_path_;
        config.width = camera_width_;
        config.height = camera_height_;
        config.framerate = framerate_;
        config.format = format_;
        config.sensor_mode = sensor_mode_;
        
        // Create camera capture using factory
        camera_capture_ = depth_anything_v3::CameraFactory::createCamera(config);
        
        if (!camera_capture_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create camera capture instance");
            RCLCPP_ERROR(this->get_logger(), "Check camera_type parameter (must be 'standard' or 'gmsl')");
            return false;
        }
        
        // Initialize camera
        if (!camera_capture_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
            return false;
        }
        
        if (!camera_capture_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Camera is not opened");
            return false;
        }
        
        // Get actual resolution
        frame_width_ = camera_capture_->getWidth();
        frame_height_ = camera_capture_->getHeight();
        
        RCLCPP_INFO(this->get_logger(), "✓ Camera opened successfully: %dx%d", 
                    frame_width_, frame_height_);
        
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
            
            // Calculate actual inference resolution
            int inference_width = frame_width_ / downsample_factor_;
            int inference_height = frame_height_ / downsample_factor_;
            
            RCLCPP_INFO(this->get_logger(), "Inference resolution: %dx%d", inference_width, inference_height);
            
            depth_estimator_->initPreprocessBuffer(inference_width, inference_height);
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
        
        camera_info.width = width;
        camera_info.height = height;
        
        if (use_calibration_) {
            // Use calibrated parameters, scaled for current resolution
            double scale_x = static_cast<double>(width) / 1920.0;
            double scale_y = static_cast<double>(height) / 1536.0;
            
            // Scale intrinsics
            camera_info.k[0] = calib_fx_ * scale_x;  // fx
            camera_info.k[2] = calib_cx_ * scale_x;  // cx
            camera_info.k[4] = calib_fy_ * scale_y;  // fy
            camera_info.k[5] = calib_cy_ * scale_y;  // cy
            camera_info.k[8] = 1.0;
            
            // Distortion coefficients (k1, k2, p1, p2, k3)
            camera_info.d.resize(5);
            camera_info.d[0] = calib_k1_;
            camera_info.d[1] = calib_k2_;
            camera_info.d[2] = calib_p1_;
            camera_info.d[3] = calib_p2_;
            camera_info.d[4] = calib_k3_;
            
            // Rectification matrix (identity for monocular)
            camera_info.r[0] = 1.0;
            camera_info.r[4] = 1.0;
            camera_info.r[8] = 1.0;
            
            // Projection matrix
            camera_info.p[0] = calib_fx_ * scale_x;
            camera_info.p[2] = calib_cx_ * scale_x;
            camera_info.p[5] = calib_fy_ * scale_y;
            camera_info.p[6] = calib_cy_ * scale_y;
            camera_info.p[10] = 1.0;
        } else {
            // Use estimated parameters based on 60° FOV
            const double focal_length = width / (2.0 * std::tan(60.0 * M_PI / 180.0 / 2.0));
            
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
        }
        
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
        createPointCloudWithCameraInfo(depth_image, rgb_image, camera_info_, cloud_msg);
    }
    
    void createPointCloudWithCameraInfo(
        const cv::Mat& depth_image,
        const cv::Mat& rgb_image,
        const sensor_msgs::msg::CameraInfo& cam_info,
        sensor_msgs::msg::PointCloud2& cloud_msg)
    {
        const double fx = cam_info.k[0];
        const double fy = cam_info.k[4];
        const double cx = cam_info.k[2];
        const double cy = cam_info.k[5];
        
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
        
        if (!camera_capture_->read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
            return;
        }
        
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }
        
        // Downsample if requested (for faster inference)
        cv::Mat frame_for_inference = frame;
        sensor_msgs::msg::CameraInfo inference_camera_info = camera_info_;
        
        if (downsample_factor_ > 1) {
            cv::Mat downsampled;
            cv::resize(frame, downsampled, 
                      cv::Size(frame.cols / downsample_factor_, frame.rows / downsample_factor_),
                      0, 0, cv::INTER_LINEAR);
            frame_for_inference = downsampled;
            
            // Update camera info for downsampled resolution
            inference_camera_info = createCameraInfo(
                frame_for_inference.cols, 
                frame_for_inference.rows
            );
        }
        
        auto stamp = this->now();
        
        // Convert to RGB
        cv::Mat frame_rgb;
        cv::cvtColor(frame_for_inference, frame_rgb, cv::COLOR_BGR2RGB);
        
        // Run depth estimation with correct camera info
        std::vector<cv::Mat> images = {frame_rgb};
        bool success = depth_estimator_->doInference(images, inference_camera_info, 1, false);
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Inference failed");
            return;
        }
        
        const cv::Mat& depth_image = depth_estimator_->getDepthImage();
        
        // Publish input image (use downsampled frame for consistency)
        auto input_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_for_inference).toImageMsg();
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
        
        // Publish point cloud with correct camera info
        sensor_msgs::msg::PointCloud2 cloud_msg;
        createPointCloudWithCameraInfo(depth_image, frame_rgb, inference_camera_info, cloud_msg);
        pointcloud_pub_->publish(cloud_msg);
        
        // Publish camera info (use inference camera info for consistency)
        inference_camera_info.header.stamp = stamp;
        camera_info_pub_->publish(inference_camera_info);
    }
    
    // Parameters
    std::string camera_type_;
    int camera_id_;
    std::string device_path_;
    std::string model_path_;
    std::string frame_id_;
    int frame_width_;
    int frame_height_;
    int camera_width_;
    int camera_height_;
    int framerate_;
    std::string format_;
    int sensor_mode_;
    int downsample_factor_;  // Downsample factor for faster inference
    
    // Camera calibration parameters
    bool use_calibration_;
    double calib_fx_;
    double calib_fy_;
    double calib_cx_;
    double calib_cy_;
    double calib_k1_;
    double calib_k2_;
    double calib_p1_;
    double calib_p2_;
    double calib_k3_;
    
    // Camera
    std::unique_ptr<depth_anything_v3::CameraCapture> camera_capture_;
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
