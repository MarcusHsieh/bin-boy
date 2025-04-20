#include "csi_camera_cpp/csi_camera_node.hpp" // Fix the include

#include <rclcpp_components/register_node_macro.hpp> // For composable nodes
#include <chrono>
#include <sstream> // For std::stringstream

using namespace std::chrono_literals;

namespace csi_camera_cpp
{

// GStreamer pipeline function (similar to the C++ example)
std::string CSICameraNode::gstreamer_pipeline_string()
{
    std::stringstream ss;
    ss << "nvarguscamerasrc sensor-id=" << sensor_id_ << " ! "
       << "video/x-raw(memory:NVMM), width=(int)" << capture_width_ << ", height=(int)" << capture_height_ << ", framerate=(fraction)" << framerate_ << "/1 ! "
       << "nvvidconv flip-method=" << flip_method_ << " ! "
       // Remove explicit width/height here, let videoconvert handle final format
       << "video/x-raw, format=(string)BGRx ! "
       << "videoconvert ! "
       << "video/x-raw, format=(string)BGR ! appsink max-buffers=1 sync=false"; // Keep low-latency appsink settings
    return ss.str();
}

CSICameraNode::CSICameraNode(const rclcpp::NodeOptions & options)
: Node("csi_camera_node", options) // Pass options for composable nodes
{
    RCLCPP_INFO(this->get_logger(), "Initializing CSICameraNode...");
    declare_parameters();
    load_parameters();
    setup_camera();
    setup_publisher();
    fill_camera_info(); // Initialize camera info
    setup_timer();
    RCLCPP_INFO(this->get_logger(), "CSICameraNode initialized successfully.");
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
    // Add parameter for camera info URL if needed later
    // this->declare_parameter<std::string>("camera_info_url", "");
}

void CSICameraNode::load_parameters()
{
    this->get_parameter("sensor_id", sensor_id_);
    this->get_parameter("capture_width", capture_width_);
    this->get_parameter("capture_height", capture_height_);
    this->get_parameter("display_width", display_width_); // Corrected ->
    this->get_parameter("display_height", display_height_); // Corrected ->
    this->get_parameter("framerate", framerate_); // Corrected ->
    this->get_parameter("flip_method", flip_method_); // Corrected ->
    this->get_parameter("publish_rate", publish_rate_); // Corrected ->
    this->get_parameter("frame_id", frame_id_); // Corrected ->
}

void CSICameraNode::setup_camera()
{
    std::string pipeline = gstreamer_pipeline_string();
    RCLCPP_INFO(this->get_logger(), "Using GStreamer pipeline: %s", pipeline.c_str());

    RCLCPP_INFO(this->get_logger(), "Attempting to open camera...");
    cap_.open(pipeline, cv::CAP_GSTREAMER);

    // Add a small delay and check if opened
    rclcpp::sleep_for(2s); // Use rclcpp sleep

    if (!cap_.isOpened()) {
        // Throw an exception instead of shutting down directly
        std::string err_msg = "Error: Unable to open camera with GStreamer pipeline. Check pipeline string and camera connection.";
        RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
        throw std::runtime_error(err_msg);
    } else {
        RCLCPP_INFO(this->get_logger(), "Successfully opened camera using GStreamer pipeline.");
    }
}

void CSICameraNode::setup_publisher()
{
    // Use image_transport to create publisher
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw");
    RCLCPP_INFO(this->get_logger(), "Publishing to %s", camera_pub_.getTopic().c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing CameraInfo to %s", camera_pub_.getInfoTopic().c_str());
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
    // Create a default CameraInfo message.
    // Ideally, load this from a calibration file using camera_info_manager
    camera_info_msg_.header.frame_id = frame_id_;
    camera_info_msg_.height = display_height_; // Use display height for info
    camera_info_msg_.width = display_width_;   // Use display width for info
    // Set default distortion model and parameters (pinhole, all zeros)
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.d.resize(5, 0.0);
    // Set default camera matrix (K) - identity for now
    camera_info_msg_.k.fill(0.0);
    camera_info_msg_.k[0] = display_width_;  // fx approx width
    camera_info_msg_.k[4] = display_height_; // fy approx height
    camera_info_msg_.k[2] = display_width_ / 2.0; // cx
    camera_info_msg_.k[5] = display_height_ / 2.0; // cy
    camera_info_msg_.k[8] = 1.0;
    // Set default rectification matrix (R) - identity
    camera_info_msg_.r.fill(0.0);
    camera_info_msg_.r[0] = camera_info_msg_.r[4] = camera_info_msg_.r[8] = 1.0;
    // Set default projection matrix (P) - K [I|0] for now
    camera_info_msg_.p.fill(0.0);
    camera_info_msg_.p[0] = camera_info_msg_.k[0]; // fx
    camera_info_msg_.p[5] = camera_info_msg_.k[4]; // fy
    camera_info_msg_.p[2] = camera_info_msg_.k[2]; // cx
    camera_info_msg_.p[6] = camera_info_msg_.k[5]; // cy
    camera_info_msg_.p[10] = 1.0;

    RCLCPP_INFO(this->get_logger(), "Generated default CameraInfo.");
}


void CSICameraNode::timer_callback()
{
    if (!cap_.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "Timer callback called but camera is not open.");
        return;
    }

    if (!cap_.read(frame_)) {
        RCLCPP_WARN(this->get_logger(), "cap.read() failed.");
        return;
    }

    if (frame_.empty()) {
         RCLCPP_WARN(this->get_logger(), "Frame is empty.");
         return;
    }

    // Convert OpenCV image to ROS Image message
    auto now = this->get_clock()->now();
    std_msgs::msg::Header header;
    header.stamp = now;
    header.frame_id = frame_id_;

    // Use cv_bridge to convert cv::Mat to sensor_msgs::msg::Image
    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg();

    // Update timestamp in CameraInfo message
    camera_info_msg_.header.stamp = now;

    // Publish image and camera info
    camera_pub_.publish(image_msg, std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_msg_));

    // Optional: Log periodically
    // static int frame_count = 0;
    // if (frame_count++ % 60 == 0) {
    //     RCLCPP_INFO(this->get_logger(), "Published frame %d", frame_count);
    // }
}

} // namespace csi_camera_cpp

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(csi_camera_cpp::CSICameraNode)
