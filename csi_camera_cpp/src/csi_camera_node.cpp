#include "csi_camera_cpp/csi_camera_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <sstream>
#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>


using namespace std::chrono_literals;

namespace csi_camera_cpp
{

// GStreamer pipeline function
std::string CSICameraNode::gstreamer_pipeline_string()
{
    // Pipeline exactly matching the provided simple_camera.cpp example
    std::stringstream ss;
    ss << "nvarguscamerasrc sensor-id=" << sensor_id_ << " ! "
       << "video/x-raw(memory:NVMM), width=(int)" << capture_width_ << ", height=(int)" << capture_height_ << ", framerate=(fraction)" << framerate_ << "/1 ! "
       << "nvvidconv flip-method=" << flip_method_ << " ! "
       << "video/x-raw, width=(int)" << display_width_ << ", height=(int)" << display_height_ << ", format=(string)BGRx ! "
       << "videoconvert ! video/x-raw, format=(string)BGR ! "
       << "appsink max-buffers=1 drop=true sync=false"; // Re-added low-latency appsink properties
    RCLCPP_INFO(this->get_logger(), "Using optimized appsink GStreamer pipeline: %s", ss.str().c_str());
    return ss.str();
}

CSICameraNode::CSICameraNode(const rclcpp::NodeOptions & options)
: Node("csi_camera_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing CSICameraNode...");
    declare_parameters();
    load_parameters();
    setup_camera();
    fill_camera_info();
    setup_publisher();
    setup_timer();
    RCLCPP_INFO(this->get_logger(), "CSICameraNode initialized.");
}

CSICameraNode::~CSICameraNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down CSICameraNode...");
    if (timer_) {
        timer_->cancel();
    }
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "Camera capture released.");
    }
    RCLCPP_INFO(this->get_logger(), "CSICameraNode shutdown complete.");
}

void CSICameraNode::declare_parameters()
{
    this->declare_parameter<int>("sensor_id", 0);
    this->declare_parameter<int>("capture_width", 1920);
    this->declare_parameter<int>("capture_height", 1080);
    this->declare_parameter<int>("display_width", 960);
    this->declare_parameter<int>("display_height", 540);
    this->declare_parameter<int>("framerate", 30);
    this->declare_parameter<int>("flip_method", 0);
    this->declare_parameter<double>("publish_rate", 30.0);
    this->declare_parameter<std::string>("frame_id", "camera_frame");
}

void CSICameraNode::load_parameters()
{
    this->get_parameter("sensor_id", sensor_id_);
    this->get_parameter("capture_width", capture_width_);
    this->get_parameter("capture_height", capture_height_);
    this->get_parameter("display_width", display_width_);
    this->get_parameter("display_height", display_height_);
    this->get_parameter("framerate", framerate_);
    this->get_parameter("flip_method", flip_method_);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("frame_id", frame_id_);
}

void CSICameraNode::setup_camera()
{
    std::string pipeline = gstreamer_pipeline_string();
    RCLCPP_INFO(this->get_logger(), "Using GStreamer pipeline: %s", pipeline.c_str());

    RCLCPP_INFO(this->get_logger(), "Attempting to open camera...");
    cap_.open(pipeline, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        std::string err_msg = "Error: Unable to open GStreamer pipeline. Check pipeline string and camera connection.";
        RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
        throw std::runtime_error(err_msg);
    } else {
        RCLCPP_INFO(this->get_logger(), "Successfully opened camera using GStreamer pipeline.");
    }
}

void CSICameraNode::setup_publisher()
{
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();
    qos.durability_volatile();

    // publishers for Image and CameraInfo
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);

    RCLCPP_INFO(this->get_logger(), "Publishing Image messages to %s", image_pub_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing CameraInfo messages to %s", info_pub_->get_topic_name());
}

void CSICameraNode::setup_timer()
{
    if (publish_rate_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "Publish rate is <= 0, timer will not be created.");
        return;
    }
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(timer_period, std::bind(&CSICameraNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Timer created with period: %.4f s (%.1f Hz)", timer_period.count(), publish_rate_);
}

void CSICameraNode::fill_camera_info()
{
    // default CameraInfo message
    camera_info_msg_.header.frame_id = frame_id_;
    camera_info_msg_.height = display_height_;
    camera_info_msg_.width = display_width_;
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.d.resize(5, 0.0);
    camera_info_msg_.k.fill(0.0);
    camera_info_msg_.k[0] = display_width_;  // fx ~ width
    camera_info_msg_.k[4] = display_height_; // fy ~ height
    camera_info_msg_.k[2] = display_width_ / 2.0; // cx
    camera_info_msg_.k[5] = display_height_ / 2.0; // cy
    camera_info_msg_.k[8] = 1.0;
    camera_info_msg_.r.fill(0.0);
    camera_info_msg_.r[0] = camera_info_msg_.r[4] = camera_info_msg_.r[8] = 1.0;
    camera_info_msg_.p.fill(0.0);
    camera_info_msg_.p[0] = camera_info_msg_.k[0]; // fx
    camera_info_msg_.p[5] = camera_info_msg_.k[4]; // fy
    camera_info_msg_.p[2] = camera_info_msg_.k[2]; // cx
    camera_info_msg_.p[6] = camera_info_msg_.k[5]; // cy
    camera_info_msg_.p[10] = 1.0;
    RCLCPP_INFO(this->get_logger(), "Default CameraInfo");
}


void CSICameraNode::timer_callback()
{
    if (!cap_.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "Timer callback called but camera not open");
        return;
    }

    if (!cap_.read(frame_)) {
        RCLCPP_WARN(this->get_logger(), "cap.read() failed");
        return;
    }

    if (frame_.empty()) {
         RCLCPP_WARN(this->get_logger(), "Frame empty");
         return;
    }

    // OpenCV image -> ROS Image msg
    auto now = this->get_clock()->now();
    std_msgs::msg::Header header;
    header.stamp = now;
    header.frame_id = frame_id_;

    // unique_ptr for image msg
    auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg(*image_msg);

    // unique_ptr for camera info msg
    auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_msg_);
    info_msg->header.stamp = now; // Update timestamp to match image

    // pub msgs IFF have subscribers
    if (image_pub_->get_subscription_count() > 0) {
        image_pub_->publish(std::move(image_msg));
    }
    if (info_pub_->get_subscription_count() > 0) {
        info_pub_->publish(std::move(info_msg));
    }
}

} // namespace csi_camera_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(csi_camera_cpp::CSICameraNode)
