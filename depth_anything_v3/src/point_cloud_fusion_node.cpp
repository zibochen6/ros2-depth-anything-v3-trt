// Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// ROS 2 node for fusing point clouds from multiple cameras
// Usage: ros2 run depth_anything_v3 point_cloud_fusion_node

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

class PointCloudFusionNode : public rclcpp::Node
{
public:
    PointCloudFusionNode() : Node("point_cloud_fusion_node")
    {
        // Declare parameters
        this->declare_parameter("input_topics", std::vector<std::string>{
            "/camera_0/point_cloud",
            "/camera_1/point_cloud",
            "/camera_2/point_cloud",
            "/camera_3/point_cloud"
        });
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("fusion_rate", 10.0);
        this->declare_parameter("timeout", 0.5);  // seconds
        
        input_topics_ = this->get_parameter("input_topics").as_string_array();
        target_frame_ = this->get_parameter("target_frame").as_string();
        double fusion_rate = this->get_parameter("fusion_rate").as_double();
        timeout_seconds_ = this->get_parameter("timeout").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Point Cloud Fusion Node");
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Fusion rate: %.1f Hz", fusion_rate);
        RCLCPP_INFO(this->get_logger(), "Timeout: %.2f seconds", timeout_seconds_);
        RCLCPP_INFO(this->get_logger(), "Input topics:");
        for (const auto& topic : input_topics_) {
            RCLCPP_INFO(this->get_logger(), "  - %s", topic.c_str());
        }
        
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create publisher for fused point cloud
        fused_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "fused_point_cloud", 10);
        
        // Create subscribers for each input topic
        for (const auto& topic : input_topics_) {
            auto callback = [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->pointCloudCallback(topic, msg);
            };
            
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, 10, callback);
            
            subscribers_.push_back(sub);
            
            // Initialize buffer entry
            cloud_buffers_[topic] = PointCloudBuffer();
        }
        
        // Create timer for fusion
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fusion_rate)),
            std::bind(&PointCloudFusionNode::fusionTimerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Point cloud fusion node started");
    }

private:
    struct PointCloudBuffer {
        sensor_msgs::msg::PointCloud2::SharedPtr cloud;
        rclcpp::Time timestamp;
        std::string source_frame;
    };
    
    void pointCloudCallback(const std::string& topic, 
                           const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Store point cloud in buffer
        cloud_buffers_[topic].cloud = msg;
        cloud_buffers_[topic].timestamp = this->now();
        cloud_buffers_[topic].source_frame = msg->header.frame_id;
    }
    
    void fusionTimerCallback()
    {
        // Remove stale point clouds
        removeStalePointClouds();
        
        // Collect and transform point clouds
        std::vector<sensor_msgs::msg::PointCloud2> transformed_clouds;
        
        for (const auto& [topic, buffer] : cloud_buffers_) {
            if (!buffer.cloud) {
                continue;  // No data received yet
            }
            
            // Transform point cloud to target frame
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            if (transformPointCloud(buffer.cloud, transformed_cloud)) {
                transformed_clouds.push_back(transformed_cloud);
            }
        }
        
        if (transformed_clouds.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No point clouds available for fusion");
            return;
        }
        
        // Fuse point clouds
        sensor_msgs::msg::PointCloud2 fused_cloud;
        fusePointClouds(transformed_clouds, fused_cloud);
        
        // Publish fused point cloud
        fused_cloud_pub_->publish(fused_cloud);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Published fused point cloud with %d points from %zu cameras",
            fused_cloud.width, transformed_clouds.size());
    }
    
    void removeStalePointClouds()
    {
        auto now = this->now();
        auto timeout = rclcpp::Duration::from_seconds(timeout_seconds_);
        
        for (auto& [topic, buffer] : cloud_buffers_) {
            if (buffer.cloud && (now - buffer.timestamp) > timeout) {
                RCLCPP_INFO(this->get_logger(), 
                    "Removing stale point cloud from %s (age: %.2f s)",
                    topic.c_str(), (now - buffer.timestamp).seconds());
                buffer.cloud.reset();
            }
        }
    }
    
    bool transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                            sensor_msgs::msg::PointCloud2& output_cloud)
    {
        try {
            // Look up transform
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                target_frame_,
                input_cloud->header.frame_id,
                tf2::TimePointZero,
                tf2::durationFromSec(0.1));
            
            // Transform point cloud
            tf2::doTransform(*input_cloud, output_cloud, transform);
            
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Failed to transform point cloud from %s to %s: %s",
                input_cloud->header.frame_id.c_str(),
                target_frame_.c_str(),
                ex.what());
            return false;
        }
    }
    
    void fusePointClouds(const std::vector<sensor_msgs::msg::PointCloud2>& input_clouds,
                        sensor_msgs::msg::PointCloud2& output_cloud)
    {
        if (input_clouds.empty()) {
            return;
        }
        
        // Count total points
        size_t total_points = 0;
        for (const auto& cloud : input_clouds) {
            total_points += cloud.width * cloud.height;
        }
        
        // Setup output cloud
        output_cloud.header.stamp = this->now();
        output_cloud.header.frame_id = target_frame_;
        output_cloud.height = 1;
        output_cloud.width = total_points;
        output_cloud.is_dense = false;
        output_cloud.is_bigendian = false;
        
        // Setup fields (xyz + rgb)
        sensor_msgs::PointCloud2Modifier modifier(output_cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(total_points);
        
        // Create iterators for output cloud
        sensor_msgs::PointCloud2Iterator<float> out_x(output_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(output_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(output_cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_r(output_cloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_g(output_cloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_b(output_cloud, "b");
        
        // Copy points from all input clouds
        for (const auto& cloud : input_clouds) {
            sensor_msgs::PointCloud2ConstIterator<float> in_x(cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> in_y(cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> in_z(cloud, "z");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> in_r(cloud, "r");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> in_g(cloud, "g");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> in_b(cloud, "b");
            
            for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z, ++in_r, ++in_g, ++in_b,
                 ++out_x, ++out_y, ++out_z, ++out_r, ++out_g, ++out_b) {
                *out_x = *in_x;
                *out_y = *in_y;
                *out_z = *in_z;
                *out_r = *in_r;
                *out_g = *in_g;
                *out_b = *in_b;
            }
        }
    }
    
    // Parameters
    std::vector<std::string> input_topics_;
    std::string target_frame_;
    double timeout_seconds_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ROS 2
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    
    // Point cloud buffers
    std::map<std::string, PointCloudBuffer> cloud_buffers_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
