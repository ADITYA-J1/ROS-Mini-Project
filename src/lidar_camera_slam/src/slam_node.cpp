#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

class SLAMNode : public rclcpp::Node {
public:
    SLAMNode() : Node("slam_node") {
        // Subscribers
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SLAMNode::lidar_callback, this, _1));

        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&SLAMNode::camera_callback, this, _1));

        // Publisher for the map
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        // Timer for publishing the map periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SLAMNode::publish_map, this)
        );

        RCLCPP_INFO(this->get_logger(), "SLAM Node Initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::OccupancyGrid map_;

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received LiDAR data with %lu points", msg->ranges.size());

        // Simulated mapping logic
        map_.header.stamp = this->get_clock()->now();
        map_.header.frame_id = "map";
        map_.info.resolution = 0.1;  // 10 cm resolution
        map_.info.width = 100;       // 10 meters
        map_.info.height = 100;      // 10 meters
        map_.info.origin.position.x = -5.0;
        map_.info.origin.position.y = -5.0;

        map_.data.resize(map_.info.width * map_.info.height, -1);  // Unknown cells

        // Fill map with dummy data for demonstration
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range > msg->range_min && range < msg->range_max) {
                int x = static_cast<int>((range * std::cos(i * msg->angle_increment)) / map_.info.resolution) + map_.info.width / 2;
                int y = static_cast<int>((range * std::sin(i * msg->angle_increment)) / map_.info.resolution) + map_.info.height / 2;

                if (x >= 0 && x < static_cast<int>(map_.info.width) &&
                    y >= 0 && y < static_cast<int>(map_.info.height)) {
                    map_.data[y * map_.info.width + x] = 100;  // Occupied cell
                }
            }
        }
    }

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Camera image with resolution %ux%u",
                    msg->width, msg->height);
    }

    void publish_map() {
        RCLCPP_INFO(this->get_logger(), "Publishing map...");
        map_pub_->publish(map_);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMNode>());
    rclcpp::shutdown();
    return 0;
}

