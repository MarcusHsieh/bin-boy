#include "csi_camera_cpp/image_viewer_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <opencv2/highgui.hpp> // For cv::imshow, cv::waitKey, etc.
#include <cv_bridge/cv_bridge.h> // For image conversion

namespace csi_camera_cpp
{

ImageViewerNode::ImageViewerNode(const rclcpp::NodeOptions & options)
: Node("image_viewer_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing ImageViewerNode...");

    // Create subscription using image_transport
    // Use default QoS for now, can customize later if needed
    // Using "raw" transport explicitly
    sub_ = image_transport::create_subscription(
        this, "image_raw", // Topic name relative to node namespace/name
        std::bind(&ImageViewerNode::image_callback, this, std::placeholders::_1),
        "raw",
        rmw_qos_profile_sensor_data); // Use sensor data QoS profile for potentially lower latency

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", sub_.getTopic().c_str());
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "ImageViewerNode initialized.");
}

ImageViewerNode::~ImageViewerNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down ImageViewerNode...");
    cv::destroyWindow(window_name_);
    RCLCPP_INFO(this->get_logger(), "ImageViewerNode shutdown complete.");
}

void ImageViewerNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    try {
        // Convert ROS Image message to OpenCV image.
        // Use toCvShare to avoid copying if possible (though display might force copy)
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

        // Display image
        cv::imshow(window_name_, cv_ptr->image);

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        return; // Return on error before waitKey
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV display/processing error: %s", e.what());
        return; // Return on error before waitKey
    }

    // Ensure waitKey is called *after* imshow and outside the main try block
    int key = cv::waitKey(1); // Use waitKey(1) for minimal delay

    // Check if window was closed or 'q'/ESC pressed
    // Use getWindowProperty check first as waitKey might return -1 if no key is pressed
    if (cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) < 1 || (key != -1 && (key == 'q' || key == 27))) {
         RCLCPP_INFO(this->get_logger(), "Close request received, shutting down viewer.");
         cv::destroyWindow(window_name_);
         // Signal main thread to exit or shutdown node cleanly
         // For simplicity, we rely on Ctrl+C in the launch terminal for now
         // Or could call rclcpp::shutdown();
         // Or potentially just stop the timer/subscription if the node should continue running
    }
    // Removed duplicated catch blocks
}

} // namespace csi_camera_cpp

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(csi_camera_cpp::ImageViewerNode)
