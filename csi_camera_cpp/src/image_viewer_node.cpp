#include "csi_camera_cpp/image_viewer_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <memory>

namespace csi_camera_cpp
{

ImageViewerNode::ImageViewerNode(const rclcpp::NodeOptions & options)
: Node("image_viewer_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing ImageViewerNode...");

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();
    qos.durability_volatile();

    // subscription expecting unique_ptr messages
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        qos,
        std::bind(&ImageViewerNode::image_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", sub_->get_topic_name());
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "ImageViewerNode initialized.");
}

ImageViewerNode::~ImageViewerNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down ImageViewerNode...");
    cv::destroyWindow(window_name_);
    RCLCPP_INFO(this->get_logger(), "ImageViewerNode shutdown complete.");
}

// callback receives unique_ptr
void ImageViewerNode::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null message pointer in callback.");
        return;
    }

    try {
        // ROS Image msg -> OpenCV image (oCvCopy) w/ dereferenced pointer
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow(window_name_, cv_ptr->image);

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        return;
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        return;
    }

    // closing
    int key = cv::waitKey(1);
    if (key != -1 && (key == 'q' || key == 27)) { // q or ESC
         RCLCPP_INFO(this->get_logger(), "Close key pressed, shutting down viewer.");
         cv::destroyWindow(window_name_);
    }
}

} // namespace csi_camera_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(csi_camera_cpp::ImageViewerNode)
