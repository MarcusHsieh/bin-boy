#ifndef CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_
#define CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <memory>

// Forward declare cv::Mat if only used in .cpp to reduce compile times by not including full opencv headers here
// namespace cv { class Mat; } // Not strictly necessary here as we don't have cv::Mat members

namespace csi_camera_cpp
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

} // namespace csi_camera_cpp

#endif // CSI_CAMERA_CPP__IMAGE_VIEWER_NODE_HPP_