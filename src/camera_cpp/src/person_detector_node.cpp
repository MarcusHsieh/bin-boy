#include "camera_cpp/person_detector_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>

namespace camera_cpp
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

    // publish camera performance metrics (for dashboard)
    performance_pub_ = this->create_publisher<bin_boy_interfaces::msg::CameraPerformance>(
        "camera/performance", qos);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", subscription_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing detections to %s", detection_pub_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing camera performance to %s", performance_pub_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "PersonDetectorNode initialized.");
}

void PersonDetectorNode::declare_parameters()
{
    std::string default_engine = "";
    std::string default_onnx = "";

    this->declare_parameter<std::string>("model_engine_path", default_engine);
    this->declare_parameter<std::string>("model_onnx_path", default_onnx);
    this->declare_parameter<bool>("use_tensorrt", true);
    this->declare_parameter<bool>("publish_annotated_image", true);
    this->declare_parameter<int>("detection_frame_skip", 0);
    this->declare_parameter<double>("confidence_threshold", 0.5);
    this->declare_parameter<bool>("debug_logging", false);
}

void PersonDetectorNode::load_parameters()
{
    this->get_parameter("model_engine_path", model_engine_path_);
    this->get_parameter("model_onnx_path", model_onnx_path_);
    this->get_parameter("use_tensorrt", use_tensorrt_);
    this->get_parameter("publish_annotated_image", publish_annotated_image_);
    this->get_parameter("detection_frame_skip", detection_frame_skip_);
    this->get_parameter("debug_logging", debug_logging_);

    double conf_threshold;
    this->get_parameter("confidence_threshold", conf_threshold);
    confidence_threshold_ = static_cast<float>(conf_threshold);

    RCLCPP_INFO(this->get_logger(), "TensorRT enabled: %s", use_tensorrt_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Confidence threshold: %.2f", confidence_threshold_);
    RCLCPP_INFO(this->get_logger(), "Detection frame skip set to: %d", detection_frame_skip_);
    RCLCPP_INFO(this->get_logger(), "Debug logging: %s", debug_logging_ ? "enabled" : "disabled");
}

void PersonDetectorNode::load_model()
{
    if (!use_tensorrt_) {
        RCLCPP_WARN(this->get_logger(), "TensorRT is disabled. No inference will be performed.");
        return;
    }

    trt_inference_ = std::make_unique<TensorRTInference>();

    std::string package_share_path;
    try {
        package_share_path = ament_index_cpp::get_package_share_directory("camera_cpp");
        RCLCPP_INFO(this->get_logger(), "Package share path: %s", package_share_path.c_str());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error finding package share directory: %s", e.what());
        throw;
    }

    // Determine absolute paths
    std::string abs_engine_path;
    std::string abs_onnx_path;

    if (model_engine_path_.empty()) {
        abs_engine_path = package_share_path + "/models/yolov5n_fp16.trt";
    } else if (model_engine_path_.rfind("/", 0) == 0) {
        abs_engine_path = model_engine_path_;
    } else {
        abs_engine_path = package_share_path + "/models/" + model_engine_path_;
    }

    if (model_onnx_path_.empty()) {
        abs_onnx_path = package_share_path + "/models/yolov5n.onnx";
    } else if (model_onnx_path_.rfind("/", 0) == 0) {
        abs_onnx_path = model_onnx_path_;
    } else {
        abs_onnx_path = package_share_path + "/models/" + model_onnx_path_;
    }

    RCLCPP_INFO(this->get_logger(), "TensorRT engine path: %s", abs_engine_path.c_str());
    RCLCPP_INFO(this->get_logger(), "ONNX model path: %s", abs_onnx_path.c_str());

    // Try to load pre-built engine first
    bool engine_loaded = false;
    std::ifstream engine_file(abs_engine_path);
    if (engine_file.good()) {
        engine_file.close();
        RCLCPP_INFO(this->get_logger(), "Loading pre-built YOLOv5n TensorRT engine...");
        engine_loaded = trt_inference_->loadEngine(abs_engine_path, ModelType::YOLOV5);
    }

    // If engine not found or failed to load, try to build from ONNX
    if (!engine_loaded) {
        RCLCPP_INFO(this->get_logger(), "Pre-built engine not found. Checking for ONNX model...");
        std::ifstream onnx_file(abs_onnx_path);
        if (onnx_file.good()) {
            onnx_file.close();
            RCLCPP_INFO(this->get_logger(), "Building YOLOv5n TensorRT engine from ONNX (this may take several minutes)...");
            RCLCPP_INFO(this->get_logger(), "This is a one-time process. Future runs will be instant!");
            engine_loaded = trt_inference_->buildEngineFromONNX(abs_onnx_path, abs_engine_path, true, ModelType::YOLOV5);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Neither TensorRT engine nor ONNX model found!");
            RCLCPP_ERROR(this->get_logger(), "Please provide either:");
            RCLCPP_ERROR(this->get_logger(), "  1. Pre-built TensorRT engine: %s", abs_engine_path.c_str());
            RCLCPP_ERROR(this->get_logger(), "  2. ONNX model file: %s", abs_onnx_path.c_str());
            RCLCPP_ERROR(this->get_logger(), "\nTo download YOLOv5n model:");
            RCLCPP_ERROR(this->get_logger(), "  cd ~/bin-boy/src/camera_cpp/scripts");
            RCLCPP_ERROR(this->get_logger(), "  python3 download_yolov5n.py");
            throw std::runtime_error("No valid model files found for TensorRT inference");
        }
    }

    if (!engine_loaded || !trt_inference_->isReady()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize TensorRT inference engine");
        throw std::runtime_error("TensorRT initialization failed");
    }

    RCLCPP_INFO(this->get_logger(), "YOLOv5n TensorRT inference engine initialized successfully!");
    RCLCPP_INFO(this->get_logger(), "GPU acceleration is now active!");
    RCLCPP_INFO(this->get_logger(), "Model: YOLOv5n (Nano)");

}

void PersonDetectorNode::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
    frame_counter_++;
    if (!msg) return;

    // frame skip
    bool skip_detection = (detection_frame_skip_ > 0) && (frame_counter_ % (detection_frame_skip_ + 1) != 0);

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat frame;

    try {
        // ROS Image msg -> OpenCV image (primary: BGR8 for color tracking)
        if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
            cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
            frame = cv_ptr->image;
        } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
            // Legacy support: convert mono8 to BGR if needed
            cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO8);
            cv::cvtColor(cv_ptr->image, frame, cv::COLOR_GRAY2BGR);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        return;
    }

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty frame.");
        return;
    }

    vision_msgs::msg::Detection2DArray detections_msg;
    detections_msg.header = msg->header;

    //if not skipping and TensorRT is initialized
    if (!skip_detection && trt_inference_ && trt_inference_->isReady())
    {
        // TensorRT inference
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Detection> detections = trt_inference_->infer(frame, confidence_threshold_, person_class_id_);
        auto end = std::chrono::high_resolution_clock::now();

        float inference_time = std::chrono::duration<float, std::milli>(end - start).count();
        float fps = 1000.0f / inference_time;

        // Publish performance metrics to dashboard
        bin_boy_interfaces::msg::CameraPerformance perf_msg;
        perf_msg.header = msg->header;
        perf_msg.inference_time_ms = inference_time;
        perf_msg.fps = fps;
        perf_msg.gpu_detections = static_cast<int32_t>(detections.size());
        perf_msg.frame_number = frame_counter_;
        performance_pub_->publish(perf_msg);

        // Log inference time periodically (every 30 frames) - only if debug logging enabled
        if (debug_logging_ && frame_counter_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "TensorRT Inference time: %.2f ms (%.1f FPS) | GPU Detections: %zu",
                inference_time, fps, detections.size());
        }

        // Process detections
        for (const auto& det : detections) {
            vision_msgs::msg::Detection2D detection;
            detection.header = msg->header;

            detection.bbox.center.x = det.bbox.x + det.bbox.width / 2.0;
            detection.bbox.center.y = det.bbox.y + det.bbox.height / 2.0;
            detection.bbox.size_x = det.bbox.width;
            detection.bbox.size_y = det.bbox.height;

            // class + score
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis_with_pose;
            hypothesis_with_pose.id = "person";
            hypothesis_with_pose.score = det.confidence;
            detection.results.push_back(hypothesis_with_pose);

            detections_msg.detections.push_back(detection);

            // annotate
            if (publish_annotated_image_) {
                cv::rectangle(frame, det.bbox, cv::Scalar(0, 255, 0), 2);
                std::string label = cv::format("Person: %.2f", det.confidence);
                int baseLine;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                int top = std::max(det.bbox.y, labelSize.height);
                cv::rectangle(frame, cv::Point(det.bbox.x, top - labelSize.height),
                              cv::Point(det.bbox.x + labelSize.width, top + baseLine),
                              cv::Scalar(255, 255, 255), cv::FILLED);
                cv::putText(frame, label, cv::Point(det.bbox.x, top),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
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

} // namespace camera_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(camera_cpp::PersonDetectorNode)
