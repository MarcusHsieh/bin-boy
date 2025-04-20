#include "csi_camera_cpp/person_detector_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace csi_camera_cpp
{

PersonDetectorNode::PersonDetectorNode(const rclcpp::NodeOptions & options)
: Node("person_detector_node", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "Initializing PersonDetectorNode...");

    declare_parameters();
    load_parameters();
    load_model();

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();
    qos.durability_volatile();

    // subscribe to raw images
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        qos,
        std::bind(&PersonDetectorNode::image_callback, this, std::placeholders::_1));

    // publish detections
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
        "person_detections", qos);

    // publish annotated image
    if (publish_annotated_image_) {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "person_detections/image", qos);
        RCLCPP_INFO(this->get_logger(), "Publishing annotated detection images to %s", image_pub_->get_topic_name());
    }

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", subscription_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing detections to %s", detection_pub_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "PersonDetectorNode initialized.");
}

void PersonDetectorNode::declare_parameters()
{
    std::string default_proto = "install/csi_camera_cpp/share/csi_camera_cpp/models/MobileNetSSD_deploy.prototxt";
    std::string default_weights = "install/csi_camera_cpp/share/csi_camera_cpp/models/MobileNetSSD_deploy.caffemodel";

    this->declare_parameter<std::string>("model_proto_path", default_proto);
    this->declare_parameter<std::string>("model_weights_path", default_weights);
    this->declare_parameter<bool>("publish_annotated_image", true);
    this->declare_parameter<int>("detection_frame_skip", 0);
}

void PersonDetectorNode::load_parameters()
{
    this->get_parameter("model_proto_path", model_proto_path_);
    this->get_parameter("model_weights_path", model_weights_path_);
    this->get_parameter("publish_annotated_image", publish_annotated_image_);
    this->get_parameter("detection_frame_skip", detection_frame_skip_);
    RCLCPP_INFO(this->get_logger(), "Detection frame skip set to: %d", detection_frame_skip_);
}

void PersonDetectorNode::load_model()
{
    std::string package_share_path;
    try {
        package_share_path = ament_index_cpp::get_package_share_directory("csi_camera_cpp");
        RCLCPP_INFO(this->get_logger(), "Package share path: %s", package_share_path.c_str());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error finding package share directory: %s", e.what());
        throw;
    }

    std::string abs_proto_path;
    std::string abs_weights_path;

    if (model_proto_path_.rfind("/", 0) == 0) {
        abs_proto_path = model_proto_path_;
    } else {
        abs_proto_path = package_share_path + "/models/MobileNetSSD_deploy.prototxt";
    }

    if (model_weights_path_.rfind("/", 0) == 0) {
        abs_weights_path = model_weights_path_;
    } else {
        abs_weights_path = package_share_path + "/models/MobileNetSSD_deploy.caffemodel";
    }

    RCLCPP_INFO(this->get_logger(), "Attempting to load model proto: %s", abs_proto_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Attempting to load model weights: %s", abs_weights_path.c_str());

    try {
        net_ = cv::dnn::readNetFromCaffe(abs_proto_path, abs_weights_path);
        if (net_.empty()) {
             throw std::runtime_error("DNN network is empty after loading model files.");
        }
        RCLCPP_INFO(this->get_logger(), "DNN model loaded successfully.");

        // cuda enable
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        RCLCPP_INFO(this->get_logger(), "Attempting to use CUDA backend for DNN.");

    } catch (const cv::Exception& e) {
        std::string err_msg = "Failed to load DNN model: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
        throw std::runtime_error(err_msg);
    }
}

void PersonDetectorNode::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
    frame_counter_++;
    if (!msg) return;

    // frame skip
    bool skip_detection = (detection_frame_skip_ > 0) && (frame_counter_ % (detection_frame_skip_ + 1) != 0);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        // ROS Image msg -> OpenCV image
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty frame.");
        return;
    }

    vision_msgs::msg::Detection2DArray detections_msg;
    detections_msg.header = msg->header;

    //if not skipping
    if (!skip_detection)
    {
        // blob for DNN input
        cv::Mat blob = cv::dnn::blobFromImage(frame, 0.007843, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5), true, false);
        net_.setInput(blob);

        // inference
        cv::Mat detections_mat = net_.forward();

        // process detections
        cv::Mat detection_data(detections_mat.size[2], detections_mat.size[3], CV_32F, detections_mat.ptr<float>());

        for (int i = 0; i < detection_data.rows; ++i) {
            float confidence = detection_data.at<float>(i, 2);

            if (confidence > confidence_threshold_) {
                int class_id = static_cast<int>(detection_data.at<float>(i, 1));

                // check: detected class == 'person'
                if (class_id == person_class_id_) {
                    int left = static_cast<int>(detection_data.at<float>(i, 3) * frame.cols);
                    int top = static_cast<int>(detection_data.at<float>(i, 4) * frame.rows);
                    int right = static_cast<int>(detection_data.at<float>(i, 5) * frame.cols);
                    int bottom = static_cast<int>(detection_data.at<float>(i, 6) * frame.rows);

                    // bbox
                    left = std::max(0, left);
                    top = std::max(0, top);
                    right = std::min(frame.cols - 1, right);
                    bottom = std::min(frame.rows - 1, bottom);
                    int width = right - left;
                    int height = bottom - top;

                    if (width > 0 && height > 0) {
                        vision_msgs::msg::Detection2D detection;
                        detection.header = msg->header;

                        detection.bbox.center.x = left + width / 2.0;
                        detection.bbox.center.y = top + height / 2.0;
                        detection.bbox.size_x = width;
                        detection.bbox.size_y = height;

                        // class + score
                        vision_msgs::msg::ObjectHypothesisWithPose hypothesis_with_pose;
                        hypothesis_with_pose.id = "person";
                        hypothesis_with_pose.score = confidence;
                        detection.results.push_back(hypothesis_with_pose);

                        detections_msg.detections.push_back(detection);

                        // annotate
                        if (publish_annotated_image_) {
                            cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);
                            std::string label = cv::format("Person: %.2f", confidence);
                            int baseLine;
                            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                            top = std::max(top, labelSize.height);
                            cv::rectangle(frame, cv::Point(left, top - labelSize.height),
                                          cv::Point(left + labelSize.width, top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
                            cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                        }
                    }
                }
            }
        }

        // publish if person found
        if (!detections_msg.detections.empty()) {
            detection_pub_->publish(detections_msg);
        }
    } // End if(!skip_detection)

    // publish if annotated image enabled
    if (publish_annotated_image_ && (!skip_detection || !detections_msg.detections.empty())) {
        auto annotated_image_msg = std::make_unique<sensor_msgs::msg::Image>();
        cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg(*annotated_image_msg);
        image_pub_->publish(std::move(annotated_image_msg));
    }
}

} // namespace csi_camera_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(csi_camera_cpp::PersonDetectorNode)
