#include "csi_camera_cpp/image_viewer_node.hpp" 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc.hpp> 

#include <string>
#include <memory> 
#include <algorithm> 

// Default values for parameters
const std::string DEFAULT_VIEWER_WINDOW_NAME = "Image Viewer";
const bool DEFAULT_ENABLE_VIEWER_CROP = false; 
const int DEFAULT_VIEWER_CROP_X = 0;
const int DEFAULT_VIEWER_CROP_Y = 0;
const int DEFAULT_VIEWER_CROP_WIDTH = 0;  // 0 means use full width from x_offset
const int DEFAULT_VIEWER_CROP_HEIGHT = 0; // 0 means full height from y_offset


namespace csi_camera_cpp
{

ImageViewerNode::ImageViewerNode(const rclcpp::NodeOptions & options)
: Node("image_viewer_node", options),
  window_active_(true) // Assume window is active initially
{
    RCLCPP_INFO(this->get_logger(), "Initializing ImageViewerNode...");

    // Declare and load parameters for cropping and window name
    this->declare_parameter<std::string>("window_name", DEFAULT_VIEWER_WINDOW_NAME);
    this->declare_parameter<bool>("enable_crop", DEFAULT_ENABLE_VIEWER_CROP);
    this->declare_parameter<int>("crop_x", DEFAULT_VIEWER_CROP_X);
    this->declare_parameter<int>("crop_y", DEFAULT_VIEWER_CROP_Y);
    this->declare_parameter<int>("crop_width", DEFAULT_VIEWER_CROP_WIDTH);
    this->declare_parameter<int>("crop_height", DEFAULT_VIEWER_CROP_HEIGHT);

    this->get_parameter("window_name", window_name_);
    this->get_parameter("enable_crop", enable_crop_);
    this->get_parameter("crop_x", crop_x_);
    this->get_parameter("crop_y", crop_y_);
    this->get_parameter("crop_width", crop_width_);
    this->get_parameter("crop_height", crop_height_);

    RCLCPP_INFO(this->get_logger(), "Window name: '%s'", window_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Enable crop: %s", enable_crop_ ? "true" : "false");
    if (enable_crop_) {
        RCLCPP_INFO(this->get_logger(), "Crop ROI (x,y,w,h): %d, %d, %d, %d",
                    crop_x_, crop_y_, crop_width_, crop_height_);
    }

    // QoS: KeepLast(1) and best_effort can make it feel more responsive if camera is fast
    rclcpp::QoS image_qos(rclcpp::KeepLast(1));
    image_qos.best_effort();
    image_qos.durability_volatile();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", // Topic to subscribe to
        image_qos,
        std::bind(&ImageViewerNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", subscription_->get_topic_name());

    if (rclcpp::ok()) {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "OpenCV window '%s' created.", window_name_.c_str());
    } else {
        window_active_ = false;
    }
    RCLCPP_INFO(this->get_logger(), "ImageViewerNode initialized.");
}

ImageViewerNode::~ImageViewerNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down ImageViewerNode...");
    if (window_active_) {
        cv::destroyWindow(window_name_);
        RCLCPP_INFO(this->get_logger(), "OpenCV window '%s' destroyed.", window_name_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "ImageViewerNode shutdown complete.");
}

void ImageViewerNode::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
    if (!window_active_ || !rclcpp::ok()) {
        return;
    }

    if (!msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received null message pointer in image_callback.");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    if (!cv_ptr || cv_ptr->image.empty()) {
        RCLCPP_WARN(this->get_logger(), "cv_bridge conversion resulted in null or empty image.");
        return;
    }
    cv::Mat full_frame = cv_ptr->image;
    cv::Mat frame_to_display;

    if (enable_crop_) {
        // Validate and apply crop
        int effective_crop_x = crop_x_;
        int effective_crop_y = crop_y_;
        int effective_crop_width = crop_width_;
        int effective_crop_height = crop_height_;

        // If width/height are 0 or too large, adjust them to fit from x,y
        if (effective_crop_width <= 0 || (effective_crop_x + effective_crop_width) > full_frame.cols) {
            effective_crop_width = full_frame.cols - effective_crop_x;
        }
        if (effective_crop_height <= 0 || (effective_crop_y + effective_crop_height) > full_frame.rows) {
            effective_crop_height = full_frame.rows - effective_crop_y;
        }

        // Final check for validity of the calculated ROI
        if (effective_crop_x >= 0 && effective_crop_y >= 0 &&
            effective_crop_width > 0 && effective_crop_height > 0 &&
            (effective_crop_x + effective_crop_width) <= full_frame.cols &&
            (effective_crop_y + effective_crop_height) <= full_frame.rows)
        {
            cv::Rect roi(effective_crop_x, effective_crop_y, effective_crop_width, effective_crop_height);
            frame_to_display = full_frame(roi); // This creates a view. For imshow, it's fine.
                                                // If you were to modify frame_to_display AND wanted to keep full_frame original,
                                                // you'd use full_frame(roi).clone();
            RCLCPP_DEBUG(this->get_logger(), "Displaying cropped frame: %dx%d", frame_to_display.cols, frame_to_display.rows);
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Invalid crop parameters for viewer. Displaying full frame. Full: %dx%d, Crop x,y,w,h: %d,%d,%d,%d",
                full_frame.cols, full_frame.rows, crop_x_, crop_y_, crop_width_, crop_height_);
            frame_to_display = full_frame; // Fallback to full frame
        }
    } else {
        frame_to_display = full_frame; // No cropping
    }

    if (frame_to_display.empty()) {
        RCLCPP_WARN(this->get_logger(), "Frame to display is empty. Not showing.");
        return;
    }

    try {
        cv::imshow(window_name_, frame_to_display);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV imshow exception: %s. Window might have been closed.", e.what());
        window_active_ = false; // Assume window is gone if imshow fails
        return;
    }

    int key = cv::waitKey(1);
    if (key != -1) {
        if (key == 'q' || key == 27) { // q or ESC
            RCLCPP_INFO(this->get_logger(), "Quit key pressed. Closing window '%s'.", window_name_.c_str());
            cv::destroyWindow(window_name_);
            window_active_ = false;
            // To stop the node itself:
            // if (rclcpp::ok()) { rclcpp::shutdown(); }
        }
    }
}

} // namespace csi_camera_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(csi_camera_cpp::ImageViewerNode)