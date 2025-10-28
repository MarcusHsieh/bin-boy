#ifndef CAMERA_CPP__UNIFIED_CAMERA_NODE_HPP_
#define CAMERA_CPP__UNIFIED_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

namespace camera_cpp
{

enum class CameraType {
    CSI,
    USB
};

class CameraNode : public rclcpp::Node
{
public:
    explicit CameraNode(const rclcpp::NodeOptions & options);
    virtual ~CameraNode();

private:
    // Camera type
    CameraType camera_type_;

    // Common parameters
    int capture_width_;
    int capture_height_;
    int display_width_;
    int display_height_;
    int framerate_;
    double publish_rate_;
    std::string frame_id_;

    // CSI-specific parameters
    int sensor_id_;
    int flip_method_;
    int awb_mode_;           // Auto white balance mode (0=off, 1=auto, 5=daylight, etc.)
    float crop_left_;        // Crop percentage from left (0.0-1.0)
    float crop_right_;       // Crop percentage from right (0.0-1.0)
    float crop_top_;         // Crop percentage from top (0.0-1.0)
    float crop_bottom_;      // Crop percentage from bottom (0.0-1.0)

    // Lens distortion correction parameters
    bool enable_distortion_correction_;  // Enable barrel/fisheye correction
    bool use_center_crop_only_;          // Use only center portion (least distortion)
    float center_crop_percentage_;       // If center crop, what percentage to keep (0.0-1.0)
    float barrel_distortion_k1_;         // Radial distortion coefficient k1

    // USB-specific parameters
    int device_id_;
    std::string device_path_;

    // Helper methods
    std::string gstreamer_pipeline_csi();
    std::string gstreamer_pipeline_usb();
    void correct_barrel_distortion(cv::Mat& frame);
    void apply_center_crop(cv::Mat& frame);

    // OpenCV
    cv::VideoCapture cap_;
    cv::Mat frame_;
    cv::Mat camera_matrix_;       // For distortion correction
    cv::Mat dist_coeffs_;         // Distortion coefficients
    cv::Mat map1_, map2_;         // Undistortion maps (precomputed for speed)

    // ROS
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    void declare_parameters();
    void load_parameters();
    void setup_camera();
    void setup_publisher();
    void setup_timer();
    void timer_callback();
    void fill_camera_info();
};

} // namespace camera_cpp

#endif // CAMERA_CPP__UNIFIED_CAMERA_NODE_HPP_
