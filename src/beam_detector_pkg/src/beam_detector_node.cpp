/**
 * @file beam_detector_node.cpp
 * @brief ROS 2 node for beam detection using YOLO11-OBB model
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "beam_detector_pkg/YOLO11-OBB.hpp"

// Custom message for beam detection results
struct BeamDetectionMsg {
    std_msgs::msg::Header header;
    std::vector<cv::Point2f> horizontal_beams;
    std::vector<cv::Point2f> vertical_beams;
    float scale_factor;
};

std::vector<cv::Point2f> get_obb_corners(const OrientedBoundingBox& box) {
    float cx = box.x, cy = box.y, w = box.width, h = box.height, angle = box.angle;
    float cos_a = cos(angle), sin_a = sin(angle);
    float w2 = w / 2, h2 = h / 2;
    
    std::vector<cv::Point2f> corners(4);
    corners[0] = cv::Point2f(cx + (-w2 * cos_a - -h2 * sin_a), cy + (-w2 * sin_a + -h2 * cos_a));
    corners[1] = cv::Point2f(cx + (w2 * cos_a - -h2 * sin_a), cy + (w2 * sin_a + -h2 * cos_a));
    corners[2] = cv::Point2f(cx + (w2 * cos_a - h2 * sin_a), cy + (w2 * sin_a + h2 * cos_a));
    corners[3] = cv::Point2f(cx + (-w2 * cos_a - h2 * sin_a), cy + (-w2 * sin_a + h2 * cos_a));
    
    return corners;
}

class BeamTracker {
public:
    BeamTracker() : reference_beam_distance(-1), scale_factor(1.0) {}
    
    void update_scale(float current_beam_distance) {
        if (reference_beam_distance < 0) {
            reference_beam_distance = current_beam_distance;
            scale_factor = 1.0;
        } else {
            scale_factor = reference_beam_distance / current_beam_distance;
        }
    }
    
    struct BeamSet {
        cv::Point2f top_horizontal;
        cv::Point2f bottom_horizontal;
        std::vector<cv::Point2f> verticals;
        bool has_top = false;
        bool has_bottom = false;
    };
    
    BeamSet extract_beams(const std::vector<Detection>& results, float conf_threshold) {
        BeamSet beams;
        std::vector<cv::Point2f> horizontal_centroids;
        std::vector<cv::Point2f> vertical_centroids;
        
        for (const auto& detection : results) {
            if (detection.conf < conf_threshold) continue;
            
            std::vector<cv::Point2f> corners = get_obb_corners(detection.box);
            float cx = 0, cy = 0;
            for (const auto& corner : corners) {
                cx += corner.x;
                cy += corner.y;
            }
            cx /= corners.size();
            cy /= corners.size();
            
            if (detection.classId == 0) {  // Vertical
                vertical_centroids.push_back(cv::Point2f(cx, cy));
            } else if (detection.classId == 1) {  // Horizontal
                horizontal_centroids.push_back(cv::Point2f(cx, cy));
            }
        }
        
        // Sort horizontal beams by y-coordinate (top to bottom)
        std::sort(horizontal_centroids.begin(), horizontal_centroids.end(), 
                 [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        
        // Take top and bottom horizontal beams
        if (horizontal_centroids.size() >= 1) {
            beams.top_horizontal = horizontal_centroids[0];
            beams.has_top = true;
            
            if (horizontal_centroids.size() >= 2) {
                beams.bottom_horizontal = horizontal_centroids.back();
                beams.has_bottom = true;
                
                // Calculate beam distance for scale correction
                float beam_distance = beams.bottom_horizontal.y - beams.top_horizontal.y;
                update_scale(beam_distance);
            }
        }
        
        // Take up to 2 vertical beams
        beams.verticals = vertical_centroids;
        if (beams.verticals.size() > 2) {
            beams.verticals.resize(2);
        }
        
        return beams;
    }
    
    float get_scale_factor() const { return scale_factor; }
    
private:
    float reference_beam_distance;
    float scale_factor;
};

void draw_tracked_lines(cv::Mat& image, const BeamTracker::BeamSet& beams, float scale_factor) {
    int h = image.rows, w = image.cols;
    cv::Scalar color(0, 255, 0);  // Green color
    
    // Draw horizontal beams
    if (beams.has_top) {
        cv::line(image, cv::Point(0, beams.top_horizontal.y), cv::Point(w, beams.top_horizontal.y), color, 2);
        cv::putText(image, "H_TOP", cv::Point(beams.top_horizontal.x, beams.top_horizontal.y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
    
    if (beams.has_bottom) {
        cv::line(image, cv::Point(0, beams.bottom_horizontal.y), cv::Point(w, beams.bottom_horizontal.y), color, 2);
        cv::putText(image, "H_BOT", cv::Point(beams.bottom_horizontal.x, beams.bottom_horizontal.y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
    
    // Draw vertical beams
    for (size_t i = 0; i < beams.verticals.size(); i++) {
        const auto& v = beams.verticals[i];
        cv::line(image, cv::Point(v.x, 0), cv::Point(v.x, h), color, 2);
        cv::putText(image, "V" + std::to_string(i), cv::Point(v.x, v.y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
    
    // Show scale factor
    cv::putText(image, "Scale: " + std::to_string(scale_factor).substr(0, 4), cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

class BeamDetectorNode : public rclcpp::Node {
public:
    BeamDetectorNode() : Node("beam_detector_node") {
        // Declare parameters
        this->declare_parameter("model_path", "/workspace/models/best.onnx");
        this->declare_parameter("labels_path", "/workspace/models/Dota.names");
        this->declare_parameter("conf_threshold", 0.5);
        this->declare_parameter("use_gpu", true);
        this->declare_parameter("input_topic", "/camera/camera/color/image_raw");
        this->declare_parameter("output_topic", "/beam_detector/image");
        
        // Get parameters
        std::string model_path = this->get_parameter("model_path").as_string();
        std::string labels_path = this->get_parameter("labels_path").as_string();
        conf_threshold_ = this->get_parameter("conf_threshold").as_double();
        bool use_gpu = this->get_parameter("use_gpu").as_bool();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        // Initialize YOLO detector
        try {
            detector_ = std::make_unique<YOLO11OBBDetector>(model_path, labels_path, use_gpu);
            RCLCPP_INFO(this->get_logger(), "YOLO detector initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO detector: %s", e.what());
            return;
        }
        
        // Create subscriber and publisher
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&BeamDetectorNode::image_callback, this, std::placeholders::_1));
        
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "Beam detector node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // Run detection
            std::vector<Detection> results = detector_->detect(image, conf_threshold_);
            
            // Extract and track beams
            BeamTracker::BeamSet current_beams = tracker_.extract_beams(results, conf_threshold_);
            
            // Draw results on image
            draw_tracked_lines(image, current_beams, tracker_.get_scale_factor());
            
            // Convert back to ROS message and publish
            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = image;
            
            image_pub_->publish(*out_msg.toImageMsg());
            
            // Log detection info
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Detected %zu objects, H-beams: %s/%s, V-beams: %zu, Scale: %.3f",
                results.size(),
                current_beams.has_top ? "TOP" : "---",
                current_beams.has_bottom ? "BOT" : "---",
                current_beams.verticals.size(),
                tracker_.get_scale_factor());
                
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
        }
    }
    
    std::unique_ptr<YOLO11OBBDetector> detector_;
    BeamTracker tracker_;
    float conf_threshold_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BeamDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}