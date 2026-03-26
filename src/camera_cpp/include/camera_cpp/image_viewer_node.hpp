#ifndef CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_
#define CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <memory>

namespace camera_cpp
{

class ImageViewerNode : public rclcpp::Node
{
public:
    explicit ImageViewerNode(const rclcpp::NodeOptions & options);
    ~ImageViewerNode();

private:
    void image_callback(sensor_msgs::msg::Image::UniquePtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    std::string window_name_;
    bool window_active_;

    // Cropping parameters
    bool enable_crop_;
    int crop_x_;
    int crop_y_;
    int crop_width_;
    int crop_height_;
};

} // namespace camera_cpp

#endif // CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_
