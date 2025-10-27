#ifndef CSI_CAMERA_CPP__TENSORRT_INFERENCE_HPP_
#define CSI_CAMERA_CPP__TENSORRT_INFERENCE_HPP_

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

namespace csi_camera_cpp
{

// Detection structure for output
struct Detection
{
    int class_id;
    float confidence;
    cv::Rect bbox;
};

// TensorRT Logger
class Logger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char* msg) noexcept override
    {
        // Only log warnings and errors
        if (severity <= Severity::kWARNING)
            std::cout << "[TensorRT] " << msg << std::endl;
    }
};

// Model type enumeration
enum class ModelType {
    MOBILENET_SSD,  // MobileNet-SSD (original)
    YOLOV5          // YOLOv5 (recommended)
};

class TensorRTInference
{
public:
    TensorRTInference();
    ~TensorRTInference();

    // Load pre-built TensorRT engine from file
    bool loadEngine(const std::string& engine_path, ModelType model_type = ModelType::YOLOV5);

    // Build TensorRT engine from ONNX model
    bool buildEngineFromONNX(const std::string& onnx_path,
                             const std::string& engine_path,
                             bool use_fp16 = true,
                             ModelType model_type = ModelType::YOLOV5);

    // Perform inference on an image
    std::vector<Detection> infer(const cv::Mat& image,
                                 float confidence_threshold = 0.5,
                                 int target_class_id = 0); // 0 = person in COCO (YOLO), 15 in SSD

    // Check if engine is loaded and ready
    bool isReady() const { return context_ != nullptr; }

    // Get last inference time in milliseconds
    float getLastInferenceTime() const { return last_inference_time_ms_; }

    // Set model type
    void setModelType(ModelType type) { model_type_ = type; }

private:
    Logger logger_;
    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;

    // Input/output dimensions
    int input_h_;
    int input_w_;
    int input_c_;
    int input_size_;      // Total input size in bytes
    int output_size_;     // Total output size in bytes

    // CUDA buffers
    void* buffers_[2];    // Input and output buffers on GPU
    cudaStream_t stream_;

    // Preprocessing buffer (host side)
    std::vector<float> input_buffer_;
    std::vector<float> output_buffer_;

    // Performance tracking
    float last_inference_time_ms_;

    // Model type
    ModelType model_type_;

    // Letterbox padding info (for YOLO)
    float scale_;
    int pad_w_;
    int pad_h_;

    // Helper methods
    bool allocateBuffers();
    void freeBuffers();
    void preprocessImage(const cv::Mat& image, float* input_buffer);
    void preprocessImageYOLO(const cv::Mat& image, float* input_buffer);
    void preprocessImageSSD(const cv::Mat& image, float* input_buffer);

    std::vector<Detection> postprocessDetections(const float* output_buffer,
                                                  int image_width,
                                                  int image_height,
                                                  float confidence_threshold,
                                                  int target_class_id);
    std::vector<Detection> postprocessYOLO(const float* output_buffer,
                                           int image_width,
                                           int image_height,
                                           float confidence_threshold,
                                           int target_class_id);
    std::vector<Detection> postprocessSSD(const float* output_buffer,
                                          int image_width,
                                          int image_height,
                                          float confidence_threshold,
                                          int target_class_id);

    // Non-Maximum Suppression for YOLO
    std::vector<Detection> nms(std::vector<Detection>& detections, float nms_threshold = 0.45f);
    float iou(const cv::Rect& box1, const cv::Rect& box2);
};

} // namespace csi_camera_cpp

#endif // CSI_CAMERA_CPP__TENSORRT_INFERENCE_HPP_
