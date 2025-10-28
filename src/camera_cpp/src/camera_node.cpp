#include "camera_cpp/camera_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <sstream>
#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

namespace camera_cpp
{

// CSI GStreamer pipeline (using nvarguscamerasrc for Jetson CSI cameras)
std::string CameraNode::gstreamer_pipeline_csi()
{
    // Simplified pipeline: capture, resize, convert to BGR
    // Cropping for vignetting removal will be done in OpenCV (more reliable)
    std::stringstream ss;
    ss << "nvarguscamerasrc sensor-id=" << sensor_id_ << " wbmode=" << awb_mode_ << " ! "
       << "video/x-raw(memory:NVMM), width=(int)" << capture_width_ << ", height=(int)" << capture_height_ << ", framerate=(fraction)" << framerate_ << "/1 ! "
       << "nvvidconv flip-method=" << flip_method_ << " ! "
       << "video/x-raw, width=(int)" << display_width_ << ", height=(int)" << display_height_ << ", format=(string)BGRx ! "
       << "videoconvert ! video/x-raw, format=(string)BGR ! "
       << "appsink max-buffers=1 drop=true sync=false";
    RCLCPP_INFO(this->get_logger(), "CSI GStreamer pipeline: %s", ss.str().c_str());
    return ss.str();
}

// USB GStreamer pipeline (using v4l2src for USB cameras)
std::string CameraNode::gstreamer_pipeline_usb()
{
    std::stringstream ss;
    ss << "v4l2src device=" << device_path_ << " ! "
       << "video/x-raw, width=(int)" << capture_width_ << ", height=(int)" << capture_height_ << ", framerate=(fraction)" << framerate_ << "/1 ! "
       << "videoconvert ! "
       << "videoscale ! "
       << "video/x-raw, width=(int)" << display_width_ << ", height=(int)" << display_height_ << ", format=(string)BGR ! "
       << "appsink max-buffers=1 drop=true sync=false";
    RCLCPP_INFO(this->get_logger(), "USB GStreamer pipeline: %s", ss.str().c_str());
    return ss.str();
}

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing CameraNode...");
    declare_parameters();
    load_parameters();
    setup_camera();
    fill_camera_info();
    setup_publisher();
    setup_timer();
    RCLCPP_INFO(this->get_logger(), "CameraNode initialized.");
}

CameraNode::~CameraNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down CameraNode...");
    if (timer_) {
        timer_->cancel();
    }
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "Camera capture released.");
    }
    RCLCPP_INFO(this->get_logger(), "CameraNode shutdown complete.");
}

void CameraNode::declare_parameters()
{
    // Camera type parameter (required)
    this->declare_parameter<std::string>("camera_type", "csi");

    // Common parameters
    this->declare_parameter<int>("capture_width", 1280);
    this->declare_parameter<int>("capture_height", 720);
    this->declare_parameter<int>("display_width", 640);
    this->declare_parameter<int>("display_height", 480);
    this->declare_parameter<int>("framerate", 15);
    this->declare_parameter<double>("publish_rate", 15.0);
    this->declare_parameter<std::string>("frame_id", "camera_frame");

    // CSI-specific parameters
    this->declare_parameter<int>("sensor_id", 0);
    this->declare_parameter<int>("flip_method", 0);
    this->declare_parameter<int>("awb_mode", 1);           // 1=auto white balance
    this->declare_parameter<double>("crop_left", 0.00);    // Calibrated: no vignetting crop needed
    this->declare_parameter<double>("crop_right", 0.00);   // Calibrated: no vignetting crop needed
    this->declare_parameter<double>("crop_top", 0.00);     // Calibrated: no vignetting crop needed
    this->declare_parameter<double>("crop_bottom", 0.00);  // Calibrated: no vignetting crop needed

    // Lens distortion correction parameters (calibrated for Waveshare IMX219-200)
    this->declare_parameter<bool>("enable_distortion_correction", true);   // Enable barrel correction
    this->declare_parameter<bool>("use_center_crop_only", false);          // Use mathematical correction
    this->declare_parameter<double>("center_crop_percentage", 0.70);       // Keep center 70% (if center crop mode)
    this->declare_parameter<double>("barrel_distortion_k1", -0.130);       // Calibrated k1 for IMX219-200 lens

    // USB-specific parameters
    this->declare_parameter<int>("device_id", 0);
}

void CameraNode::load_parameters()
{
    // Load camera type
    std::string camera_type_str;
    this->get_parameter("camera_type", camera_type_str);

    if (camera_type_str == "csi") {
        camera_type_ = CameraType::CSI;
        RCLCPP_INFO(this->get_logger(), "Camera type: CSI");
    } else if (camera_type_str == "usb") {
        camera_type_ = CameraType::USB;
        RCLCPP_INFO(this->get_logger(), "Camera type: USB");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid camera_type '%s'. Must be 'csi' or 'usb'. Defaulting to 'csi'.", camera_type_str.c_str());
        camera_type_ = CameraType::CSI;
    }

    // Load common parameters
    this->get_parameter("capture_width", capture_width_);
    this->get_parameter("capture_height", capture_height_);
    this->get_parameter("display_width", display_width_);
    this->get_parameter("display_height", display_height_);
    this->get_parameter("framerate", framerate_);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("frame_id", frame_id_);

    // Load type-specific parameters
    if (camera_type_ == CameraType::CSI) {
        this->get_parameter("sensor_id", sensor_id_);
        this->get_parameter("flip_method", flip_method_);
        this->get_parameter("awb_mode", awb_mode_);

        double crop_l, crop_r, crop_t, crop_b;
        this->get_parameter("crop_left", crop_l);
        this->get_parameter("crop_right", crop_r);
        this->get_parameter("crop_top", crop_t);
        this->get_parameter("crop_bottom", crop_b);
        crop_left_ = static_cast<float>(crop_l);
        crop_right_ = static_cast<float>(crop_r);
        crop_top_ = static_cast<float>(crop_t);
        crop_bottom_ = static_cast<float>(crop_b);

        // Load distortion correction parameters
        this->get_parameter("enable_distortion_correction", enable_distortion_correction_);
        this->get_parameter("use_center_crop_only", use_center_crop_only_);
        double center_crop_pct, k1;
        this->get_parameter("center_crop_percentage", center_crop_pct);
        this->get_parameter("barrel_distortion_k1", k1);
        center_crop_percentage_ = static_cast<float>(center_crop_pct);
        barrel_distortion_k1_ = static_cast<float>(k1);

        RCLCPP_INFO(this->get_logger(), "CSI parameters - sensor_id: %d, flip_method: %d, awb_mode: %d",
                    sensor_id_, flip_method_, awb_mode_);
        RCLCPP_INFO(this->get_logger(), "CSI crop - L:%.2f R:%.2f T:%.2f B:%.2f",
                    crop_left_, crop_right_, crop_top_, crop_bottom_);
        RCLCPP_INFO(this->get_logger(), "Distortion correction: %s, Center crop only: %s (%.1f%%), k1: %.3f",
                    enable_distortion_correction_ ? "enabled" : "disabled",
                    use_center_crop_only_ ? "yes" : "no",
                    center_crop_percentage_ * 100.0f, barrel_distortion_k1_);

        // Initialize camera matrix and distortion coefficients for correction
        if (enable_distortion_correction_ && !use_center_crop_only_) {
            // Camera matrix (approximate for 200° FOV lens)
            camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
            camera_matrix_.at<double>(0, 0) = display_width_ * 0.5;  // fx (focal length in pixels)
            camera_matrix_.at<double>(1, 1) = display_height_ * 0.5; // fy
            camera_matrix_.at<double>(0, 2) = display_width_ / 2.0;  // cx (principal point)
            camera_matrix_.at<double>(1, 2) = display_height_ / 2.0; // cy

            // Distortion coefficients: [k1, k2, p1, p2, k3]
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
            dist_coeffs_.at<double>(0) = barrel_distortion_k1_;  // k1 (radial distortion)
            dist_coeffs_.at<double>(1) = 0.0;                     // k2
            dist_coeffs_.at<double>(2) = 0.0;                     // p1 (tangential)
            dist_coeffs_.at<double>(3) = 0.0;                     // p2
            dist_coeffs_.at<double>(4) = 0.0;                     // k3

            // Precompute undistortion maps for performance
            cv::Size image_size(display_width_, display_height_);
            cv::initUndistortRectifyMap(
                camera_matrix_, dist_coeffs_, cv::Mat(),
                camera_matrix_, image_size,
                CV_32FC1, map1_, map2_);

            RCLCPP_INFO(this->get_logger(), "Undistortion maps initialized for barrel correction");
        }
    } else {
        this->get_parameter("device_id", device_id_);
        device_path_ = "/dev/video" + std::to_string(device_id_);
        RCLCPP_INFO(this->get_logger(), "USB parameters - device_id: %d, device_path: %s", device_id_, device_path_.c_str());
    }
}

void CameraNode::setup_camera()
{
    std::string pipeline;

    if (camera_type_ == CameraType::CSI) {
        pipeline = gstreamer_pipeline_csi();
        RCLCPP_INFO(this->get_logger(), "Attempting to open CSI camera with GStreamer...");
        cap_.open(pipeline, cv::CAP_GSTREAMER);
    } else {
        // For USB cameras, we can use either GStreamer or direct V4L2
        // Using GStreamer for consistency with CSI approach
        pipeline = gstreamer_pipeline_usb();
        RCLCPP_INFO(this->get_logger(), "Attempting to open USB camera with GStreamer...");
        cap_.open(pipeline, cv::CAP_GSTREAMER);

        // If GStreamer fails for USB, try direct V4L2 as fallback
        if (!cap_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "GStreamer failed for USB camera, trying direct V4L2...");
            cap_.open(device_id_, cv::CAP_V4L2);

            if (cap_.isOpened()) {
                // Set properties for V4L2 capture
                // MJPEG format requires proper decoding - let OpenCV handle it
                cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, capture_width_);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height_);
                cap_.set(cv::CAP_PROP_FPS, framerate_);
                // Keep CAP_PROP_CONVERT_RGB enabled for proper MJPEG decoding
                RCLCPP_INFO(this->get_logger(), "Successfully opened USB camera with V4L2 (MJPEG mode)");
            }
        }
    }

    if (!cap_.isOpened()) {
        std::string err_msg = "Error: Unable to open camera. Check camera connection and parameters.";
        RCLCPP_ERROR(this->get_logger(), "%s", err_msg.c_str());
        throw std::runtime_error(err_msg);
    } else {
        RCLCPP_INFO(this->get_logger(), "Successfully opened camera.");
    }
}

void CameraNode::setup_publisher()
{
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();
    qos.durability_volatile();

    // Publishers for Image and CameraInfo
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);

    RCLCPP_INFO(this->get_logger(), "Publishing Image messages to %s", image_pub_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing CameraInfo messages to %s", info_pub_->get_topic_name());
}

void CameraNode::setup_timer()
{
    if (publish_rate_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "Publish rate is <= 0, timer will not be created.");
        return;
    }
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(timer_period, std::bind(&CameraNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Timer created with period: %.4f s (%.1f Hz)", timer_period.count(), publish_rate_);
}

void CameraNode::fill_camera_info()
{
    // Default CameraInfo message
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
    RCLCPP_INFO(this->get_logger(), "Default CameraInfo created");
}

void CameraNode::timer_callback()
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

    // Resize frame if using V4L2 directly (GStreamer handles this automatically)
    if (camera_type_ == CameraType::USB &&
        (frame_.cols != display_width_ || frame_.rows != display_height_)) {
        cv::resize(frame_, frame_, cv::Size(display_width_, display_height_));
    }

    // Crop vignetting for CSI camera (ultra-wide lens)
    if (camera_type_ == CameraType::CSI &&
        (crop_left_ > 0.0f || crop_right_ > 0.0f || crop_top_ > 0.0f || crop_bottom_ > 0.0f)) {

        int crop_left_px = static_cast<int>(frame_.cols * crop_left_);
        int crop_right_px = static_cast<int>(frame_.cols * crop_right_);
        int crop_top_px = static_cast<int>(frame_.rows * crop_top_);
        int crop_bottom_px = static_cast<int>(frame_.rows * crop_bottom_);

        int cropped_width = frame_.cols - crop_left_px - crop_right_px;
        int cropped_height = frame_.rows - crop_top_px - crop_bottom_px;

        if (cropped_width > 0 && cropped_height > 0) {
            cv::Rect crop_roi(crop_left_px, crop_top_px, cropped_width, cropped_height);
            frame_ = frame_(crop_roi).clone();  // Crop and clone to make continuous
        }
    }

    // Apply lens distortion correction for CSI camera (200° FOV fisheye lens)
    if (camera_type_ == CameraType::CSI && enable_distortion_correction_) {
        if (use_center_crop_only_) {
            apply_center_crop(frame_);  // Just use center portion (no distortion there)
        } else {
            correct_barrel_distortion(frame_);  // Apply mathematical correction
        }
    }

    // OpenCV image -> ROS Image msg
    auto now = this->get_clock()->now();
    std_msgs::msg::Header header;
    header.stamp = now;
    header.frame_id = frame_id_;

    // unique_ptr for image msg (BGR8 for color processing)
    auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg(*image_msg);

    // unique_ptr for camera info msg
    auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_msg_);
    info_msg->header.stamp = now; // Update timestamp to match image

    // Publish msgs IFF have subscribers
    if (image_pub_->get_subscription_count() > 0) {
        image_pub_->publish(std::move(image_msg));
    }
    if (info_pub_->get_subscription_count() > 0) {
        info_pub_->publish(std::move(info_msg));
    }
}

void CameraNode::correct_barrel_distortion(cv::Mat& frame)
{
    // Apply barrel distortion correction using precomputed undistortion maps
    if (!map1_.empty() && !map2_.empty()) {
        cv::Mat corrected;
        cv::remap(frame, corrected, map1_, map2_, cv::INTER_LINEAR);
        frame = corrected;
    }
}

void CameraNode::apply_center_crop(cv::Mat& frame)
{
    // Crop to center portion only (where distortion is minimal)
    if (center_crop_percentage_ <= 0.0f || center_crop_percentage_ >= 1.0f) {
        return;  // Invalid percentage, skip
    }

    int new_width = static_cast<int>(frame.cols * center_crop_percentage_);
    int new_height = static_cast<int>(frame.rows * center_crop_percentage_);

    if (new_width > 0 && new_height > 0 && new_width < frame.cols && new_height < frame.rows) {
        int x_offset = (frame.cols - new_width) / 2;
        int y_offset = (frame.rows - new_height) / 2;

        cv::Rect center_roi(x_offset, y_offset, new_width, new_height);
        cv::Mat center_cropped = frame(center_roi).clone();

        // Resize back to original dimensions to maintain consistent output size
        cv::resize(center_cropped, frame, cv::Size(frame.cols, frame.rows));
    }
}

} // namespace camera_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(camera_cpp::CameraNode)
