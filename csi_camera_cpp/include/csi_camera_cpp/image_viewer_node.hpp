#ifndef CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_
#define CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp> // Use image_transport for subscription
#include <opencv2/opencv.hpp>
#include <memory> // For std::shared_ptr

namespace csi_camera_cpp
{

class ImageViewerNode : public rclcpp::Node
{
public:
    explicit ImageViewerNode(const rclcpp::NodeOptions & options);
    virtual ~ImageViewerNode();

private:
    // Use image_transport::Subscriber for potential transport options later
    image_transport::Subscriber sub_;
    std::string window_name_ = "CSI Camera Viewer (C++)";

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
};

} // namespace csi_camera_cpp

#endif // CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_
