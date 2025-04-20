#ifndef CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_
#define CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>

namespace csi_camera_cpp
{

class ImageViewerNode : public rclcpp::Node
{
public:
    explicit ImageViewerNode(const rclcpp::NodeOptions & options);
    virtual ~ImageViewerNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::string window_name_ = "CSI Camera Viewer (C++)";

    void image_callback(sensor_msgs::msg::Image::UniquePtr msg);
};

} // namespace csi_camera_cpp

#endif // CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_
