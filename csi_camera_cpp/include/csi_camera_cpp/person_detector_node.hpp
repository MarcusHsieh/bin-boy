#ifndef CSI_CAMERA_CPP__PERSON_DETECTOR_NODE_HPP_
#define CSI_CAMERA_CPP__PERSON_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <memory>
#include <string>
#include <vector>

namespace csi_camera_cpp
{

class PersonDetectorNode : public rclcpp::Node
{
public:
    explicit PersonDetectorNode(const rclcpp::NodeOptions & options);
    virtual ~PersonDetectorNode() = default;

private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // opencv DNN model
    cv::dnn::Net net_;
    const float confidence_threshold_ = 0.5;
    const int person_class_id_ = 15; // person

    // parameters
    std::string model_proto_path_;
    std::string model_weights_path_;
    bool publish_annotated_image_;
    int detection_frame_skip_;

    // internal state
    size_t frame_counter_ = 0;

    void declare_parameters();
    void load_parameters();
    void load_model();
    void image_callback(sensor_msgs::msg::Image::UniquePtr msg);
};

} // namespace csi_camera_cpp

#endif // CSI_CAMERA_CPP__PERSON_DETECTOR_NODE_HPP_
