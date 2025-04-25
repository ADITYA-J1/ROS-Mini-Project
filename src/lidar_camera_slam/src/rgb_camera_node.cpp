#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class RGBCameraNode : public rclcpp::Node
{
public:
    RGBCameraNode() : Node("rgb_camera_node")
    {
        // Create subscribers
        image_sub_ = image_transport::create_subscription(
            this,
            "/camera/image_raw",
            std::bind(&RGBCameraNode::imageCallback, this, std::placeholders::_1),
            "raw",
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud",  // Adjust this topic name to match your LiDAR point cloud topic
            10,
            std::bind(&RGBCameraNode::pointcloudCallback, this, std::placeholders::_1)
        );

        // Create publisher for colored point cloud
        colored_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/colored_point_cloud",
            10
        );

        RCLCPP_INFO(this->get_logger(), "RGB Camera Node has been initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image;
            image_timestamp_ = msg->header.stamp;
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (current_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image available yet");
            return;
        }

        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        // Process the point cloud and add color information
        // This is a simplified version - you'll need to implement the actual color mapping
        // based on your specific requirements and camera-LiDAR calibration
        for (auto& point : *cloud) {
            // Example: Set all points to red for testing
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*cloud, colored_cloud_msg);
        colored_cloud_msg.header = msg->header;

        // Publish the colored point cloud
        colored_pointcloud_pub_->publish(colored_cloud_msg);
    }

    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pointcloud_pub_;
    
    cv::Mat current_image_;
    builtin_interfaces::msg::Time image_timestamp_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 