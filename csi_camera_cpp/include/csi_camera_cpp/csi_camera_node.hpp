#ifndef CSI_CAMERA_CPP__CSI_CAMERA_NODE_HPP_
#define CSI_CAMERA_CPP__CSI_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

namespace csi_camera_cpp
{

class CSICameraNode : public rclcpp::Node
{
public:
    explicit CSICameraNode(const rclcpp::NodeOptions & options);
    virtual ~CSICameraNode();

private:
    // parameters
    int sensor_id_;
    int capture_width_;
    int capture_height_;
    int display_width_;
    int display_height_;
    int framerate_;
    int flip_method_;
    double publish_rate_;
    std::string frame_id_;

    std::string gstreamer_pipeline_string();

    // opencv
    cv::VideoCapture cap_;
    cv::Mat frame_;

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

} // namespace csi_camera_cpp

#endif // CSI_CAMERA_CPP__CSI_CAMERA_NODE_HPP_
