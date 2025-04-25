#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <filesystem>
#include <fstream>

using std::placeholders::_1;

class ObjectDetectionNode : public rclcpp::Node {
public:
    ObjectDetectionNode() : Node("object_detection_node") {
        // Get package path
        std::string package_path = std::filesystem::current_path().string();
        std::string models_path = package_path + "/src/lidar_camera_slam/models/";

        // Load YOLO model
        std::string model_path = models_path + "yolov4.weights";
        std::string config_path = models_path + "yolov4.cfg";
        std::string classes_path = models_path + "coco.names";

        RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Loading classes from: %s", classes_path.c_str());

        net_ = cv::dnn::readNetFromDarknet(config_path, model_path);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        // Load class names
        std::ifstream ifs(classes_path);
        std::string line;
        while (std::getline(ifs, line)) {
            classes_.push_back(line);
        }

        // Subscriber for camera images
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ObjectDetectionNode::camera_callback, this, _1));

        // Publisher for detected objects
        detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "/detected_objects", 10);

        RCLCPP_INFO(this->get_logger(), "Object Detection Node Initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
    cv::dnn::Net net_;
    std::vector<std::string> classes_;

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // Prepare input blob
            cv::Mat blob = cv::dnn::blobFromImage(frame, 1/255.0, cv::Size(416, 416), 
                                                cv::Scalar(0,0,0), true, false);
            net_.setInput(blob);

            // Forward pass
            std::vector<cv::Mat> outs;
            net_.forward(outs, getOutputsNames());

            // Process detections
            vision_msgs::msg::Detection2DArray detections;
            detections.header = msg->header;

            for (const auto& out : outs) {
                float* data = (float*)out.data;
                for (int i = 0; i < out.rows; ++i, data += out.cols) {
                    cv::Mat scores = out.row(i).colRange(5, out.cols);
                    cv::Point classIdPoint;
                    double confidence;
                    cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);

                    if (confidence > 0.5) {
                        vision_msgs::msg::Detection2D detection;
                        vision_msgs::msg::BoundingBox2D bbox;

                        int centerX = (int)(data[0] * frame.cols);
                        int centerY = (int)(data[1] * frame.rows);
                        int width = (int)(data[2] * frame.cols);
                        int height = (int)(data[3] * frame.rows);

                        bbox.center.position.x = centerX;
                        bbox.center.position.y = centerY;
                        bbox.size_x = width;
                        bbox.size_y = height;

                        detection.bbox = bbox;
                        detection.results.resize(1);
                        auto& hypothesis = detection.results[0];
                        hypothesis.hypothesis.class_id = classes_[classIdPoint.x];
                        hypothesis.hypothesis.score = confidence;

                        detections.detections.push_back(detection);
                    }
                }
            }

            detections_pub_->publish(detections);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
        }
    }

    std::vector<std::string> getOutputsNames() {
        static std::vector<std::string> names;
        if (names.empty()) {
            std::vector<int> outLayers = net_.getUnconnectedOutLayers();
            std::vector<std::string> layersNames = net_.getLayerNames();
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i) {
                names[i] = layersNames[outLayers[i] - 1];
            }
        }
        return names;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
} 