#include "csi_camera_cpp/tensorrt_inference.hpp"
#include <chrono>
#include <algorithm>

namespace csi_camera_cpp
{

TensorRTInference::TensorRTInference()
    : runtime_(nullptr)
    , engine_(nullptr)
    , context_(nullptr)
    , input_h_(640)  // YOLOv5 default (was 300 for SSD)
    , input_w_(640)
    , input_c_(3)
    , input_size_(0)
    , output_size_(0)
    , last_inference_time_ms_(0.0f)
    , model_type_(ModelType::YOLOV5)
    , scale_(1.0f)
    , pad_w_(0)
    , pad_h_(0)
{
    buffers_[0] = nullptr;
    buffers_[1] = nullptr;

    // Create CUDA stream
    cudaStreamCreate(&stream_);
}

TensorRTInference::~TensorRTInference()
{
    freeBuffers();

    if (context_)
        delete context_;
    if (engine_)
        delete engine_;
    if (runtime_)
        delete runtime_;

    cudaStreamDestroy(stream_);
}

bool TensorRTInference::loadEngine(const std::string& engine_path, ModelType model_type)
{
    model_type_ = model_type;

    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file.good()) {
        std::cerr << "[TensorRT] Failed to open engine file: " << engine_path << std::endl;
        return false;
    }

    // Read engine file
    engine_file.seekg(0, std::ifstream::end);
    size_t size = engine_file.tellg();
    engine_file.seekg(0, std::ifstream::beg);

    std::vector<char> engine_data(size);
    engine_file.read(engine_data.data(), size);
    engine_file.close();

    // Deserialize engine
    runtime_ = nvinfer1::createInferRuntime(logger_);
    if (!runtime_) {
        std::cerr << "[TensorRT] Failed to create runtime" << std::endl;
        return false;
    }

    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
    if (!engine_) {
        std::cerr << "[TensorRT] Failed to deserialize engine" << std::endl;
        return false;
    }

    context_ = engine_->createExecutionContext();
    if (!context_) {
        std::cerr << "[TensorRT] Failed to create execution context" << std::endl;
        return false;
    }

    // Get input dimensions from engine
    // For MobileNet-SSD: input is "data" or binding index 0
    nvinfer1::Dims input_dims = engine_->getBindingDimensions(0);
    if (input_dims.nbDims == 4) {  // NCHW format
        input_c_ = input_dims.d[1];
        input_h_ = input_dims.d[2];
        input_w_ = input_dims.d[3];
    } else if (input_dims.nbDims == 3) {  // CHW format
        input_c_ = input_dims.d[0];
        input_h_ = input_dims.d[1];
        input_w_ = input_dims.d[2];
    }

    std::cout << "[TensorRT] Input dimensions: " << input_c_ << "x"
              << input_h_ << "x" << input_w_ << std::endl;

    // Allocate buffers
    if (!allocateBuffers()) {
        std::cerr << "[TensorRT] Failed to allocate buffers" << std::endl;
        return false;
    }

    std::cout << "[TensorRT] Engine loaded successfully from: " << engine_path << std::endl;
    return true;
}

bool TensorRTInference::buildEngineFromONNX(const std::string& onnx_path,
                                            const std::string& engine_path,
                                            bool use_fp16,
                                            ModelType model_type)
{
    model_type_ = model_type;

    // Create builder
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(logger_);
    if (!builder) {
        std::cerr << "[TensorRT] Failed to create builder" << std::endl;
        return false;
    }

    // Create network definition
    const auto explicit_batch = 1U << static_cast<uint32_t>(
        nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(explicit_batch);
    if (!network) {
        std::cerr << "[TensorRT] Failed to create network" << std::endl;
        delete builder;
        return false;
    }

    // Create ONNX parser
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger_);
    if (!parser) {
        std::cerr << "[TensorRT] Failed to create ONNX parser" << std::endl;
        delete network;
        delete builder;
        return false;
    }

    // Parse ONNX file
    bool parsed = parser->parseFromFile(onnx_path.c_str(),
        static_cast<int>(nvinfer1::ILogger::Severity::kWARNING));
    if (!parsed) {
        std::cerr << "[TensorRT] Failed to parse ONNX file: " << onnx_path << std::endl;
        for (int i = 0; i < parser->getNbErrors(); ++i) {
            std::cerr << "[TensorRT] Error: " << parser->getError(i)->desc() << std::endl;
        }
        delete parser;
        delete network;
        delete builder;
        return false;
    }

    // Build engine configuration
    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
    if (!config) {
        std::cerr << "[TensorRT] Failed to create builder config" << std::endl;
        delete parser;
        delete network;
        delete builder;
        return false;
    }

    // Set max workspace size (512 MB)
    config->setMaxWorkspaceSize(512 * 1024 * 1024);

    // Enable FP16 if supported and requested
    if (use_fp16 && builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        std::cout << "[TensorRT] FP16 mode enabled" << std::endl;
    }

    // Build engine
    std::cout << "[TensorRT] Building engine from ONNX (this may take a few minutes)..." << std::endl;
    nvinfer1::IHostMemory* plan = builder->buildSerializedNetwork(*network, *config);
    if (!plan) {
        std::cerr << "[TensorRT] Failed to build engine" << std::endl;
        delete config;
        delete parser;
        delete network;
        delete builder;
        return false;
    }

    // Get runtime and deserialize engine
    runtime_ = nvinfer1::createInferRuntime(logger_);
    engine_ = runtime_->deserializeCudaEngine(plan->data(), plan->size());

    // Save engine to file
    std::ofstream engine_file(engine_path, std::ios::binary);
    if (engine_file.good()) {
        engine_file.write(static_cast<const char*>(plan->data()), plan->size());
        std::cout << "[TensorRT] Engine saved to: " << engine_path << std::endl;
    }
    delete plan;

    if (!engine_) {
        std::cerr << "[TensorRT] Failed to deserialize engine" << std::endl;
        delete config;
        delete parser;
        delete network;
        delete builder;
        return false;
    }

    // Create execution context
    context_ = engine_->createExecutionContext();
    if (!context_) {
        std::cerr << "[TensorRT] Failed to create execution context" << std::endl;
        delete config;
        delete parser;
        delete network;
        delete builder;
        return false;
    }

    // Cleanup
    delete config;
    delete parser;
    delete network;
    delete builder;

    // Allocate buffers
    if (!allocateBuffers()) {
        std::cerr << "[TensorRT] Failed to allocate buffers" << std::endl;
        return false;
    }

    std::cout << "[TensorRT] Engine built successfully!" << std::endl;
    return true;
}

bool TensorRTInference::allocateBuffers()
{
    if (!engine_) return false;

    // Calculate input size from stored dimensions
    input_size_ = input_c_ * input_h_ * input_w_ * sizeof(float);

    // Get output size (binding 1)
    nvinfer1::Dims output_dims = engine_->getBindingDimensions(1);
    output_size_ = 1;  // Start with 1

    std::cout << "[TensorRT] Output dimensions: [";
    for (int i = 0; i < output_dims.nbDims; ++i) {
        std::cout << output_dims.d[i];
        if (i < output_dims.nbDims - 1) std::cout << ", ";
        output_size_ *= output_dims.d[i];
    }
    std::cout << "]" << std::endl;
    output_size_ *= sizeof(float);

    std::cout << "[TensorRT] Input size: " << input_size_ << " bytes" << std::endl;
    std::cout << "[TensorRT] Output size: " << output_size_ << " bytes ("
              << (output_size_ / sizeof(float)) << " floats)" << std::endl;

    // Allocate GPU buffers
    cudaMalloc(&buffers_[0], input_size_);
    cudaMalloc(&buffers_[1], output_size_);

    // Allocate CPU buffers
    input_buffer_.resize(input_size_ / sizeof(float));
    output_buffer_.resize(output_size_ / sizeof(float));

    return true;
}

void TensorRTInference::freeBuffers()
{
    if (buffers_[0]) {
        cudaFree(buffers_[0]);
        buffers_[0] = nullptr;
    }
    if (buffers_[1]) {
        cudaFree(buffers_[1]);
        buffers_[1] = nullptr;
    }
}

void TensorRTInference::preprocessImage(const cv::Mat& image, float* input_buffer)
{
    if (model_type_ == ModelType::YOLOV5) {
        preprocessImageYOLO(image, input_buffer);
    } else {
        preprocessImageSSD(image, input_buffer);
    }
}

void TensorRTInference::preprocessImageYOLO(const cv::Mat& image, float* input_buffer)
{
    // YOLO uses letterbox preprocessing to maintain aspect ratio
    int img_w = image.cols;
    int img_h = image.rows;

    // Calculate scale to fit image in input size while maintaining aspect ratio
    scale_ = std::min(static_cast<float>(input_w_) / img_w,
                      static_cast<float>(input_h_) / img_h);

    int new_w = static_cast<int>(img_w * scale_);
    int new_h = static_cast<int>(img_h * scale_);

    // Calculate padding
    pad_w_ = (input_w_ - new_w) / 2;
    pad_h_ = (input_h_ - new_h) / 2;

    // Resize image
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h));

    // Create letterbox image with gray padding
    cv::Mat letterbox(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(letterbox(cv::Rect(pad_w_, pad_h_, new_w, new_h)));

    // Convert BGR to RGB
    cv::Mat rgb;
    cv::cvtColor(letterbox, rgb, cv::COLOR_BGR2RGB);

    // Normalize to [0, 1] and convert to NCHW format
    cv::Mat float_img;
    rgb.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    // Convert HWC to CHW (planar format)
    std::vector<cv::Mat> channels(3);
    cv::split(float_img, channels);

    int channel_size = input_h_ * input_w_;
    for (int c = 0; c < 3; ++c) {
        memcpy(input_buffer + c * channel_size,
               channels[c].data,
               channel_size * sizeof(float));
    }
}

void TensorRTInference::preprocessImageSSD(const cv::Mat& image, float* input_buffer)
{
    // MobileNet-SSD preprocessing (original implementation)
    // Resize image to network input size
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(input_w_, input_h_));

    // Convert BGR to RGB
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    // Convert to float and normalize (0-255 to 0-1, then mean subtraction)
    // MobileNet-SSD uses mean subtraction: (pixel / 255.0 - 0.5) * 2.0
    // Which is equivalent to: (pixel - 127.5) / 127.5
    cv::Mat float_img;
    rgb.convertTo(float_img, CV_32FC3, 1.0 / 127.5, -1.0);

    // Convert HWC to CHW (planar format)
    std::vector<cv::Mat> channels(3);
    cv::split(float_img, channels);

    int channel_size = input_h_ * input_w_;
    for (int c = 0; c < 3; ++c) {
        memcpy(input_buffer + c * channel_size,
               channels[c].data,
               channel_size * sizeof(float));
    }
}

std::vector<Detection> TensorRTInference::postprocessDetections(const float* output_buffer,
                                                                 int image_width,
                                                                 int image_height,
                                                                 float confidence_threshold,
                                                                 int target_class_id)
{
    if (model_type_ == ModelType::YOLOV5) {
        return postprocessYOLO(output_buffer, image_width, image_height, confidence_threshold, target_class_id);
    } else {
        return postprocessSSD(output_buffer, image_width, image_height, confidence_threshold, target_class_id);
    }
}

std::vector<Detection> TensorRTInference::postprocessSSD(const float* output_buffer,
                                                          int image_width,
                                                          int image_height,
                                                          float confidence_threshold,
                                                          int target_class_id)
{
    std::vector<Detection> detections;

    // MobileNet-SSD output format: [1, 1, N, 7]
    // Each detection: [image_id, label, confidence, xmin, ymin, xmax, ymax]

    // The output is typically structured as: num_detections followed by detection data
    // For MobileNet-SSD, the structure is [1, 1, 100, 7] (up to 100 detections)

    int num_detections = output_buffer_.size() / 7;  // Simplified assumption
    if (num_detections > 100) num_detections = 100;  // Cap at reasonable limit

    for (int i = 0; i < num_detections; ++i) {
        int base_idx = i * 7;

        // Check if we're still within valid data
        if (base_idx + 6 >= static_cast<int>(output_buffer_.size())) break;

        float confidence = output_buffer[base_idx + 2];

        if (confidence > confidence_threshold) {
            int class_id = static_cast<int>(output_buffer[base_idx + 1]);

            // Filter for target class (person = 15 in COCO, but in MobileNet-SSD it's often 15)
            if (class_id == target_class_id) {
                float xmin = output_buffer[base_idx + 3];
                float ymin = output_buffer[base_idx + 4];
                float xmax = output_buffer[base_idx + 5];
                float ymax = output_buffer[base_idx + 6];

                // Convert normalized coordinates to pixel coordinates
                int left = static_cast<int>(xmin * image_width);
                int top = static_cast<int>(ymin * image_height);
                int right = static_cast<int>(xmax * image_width);
                int bottom = static_cast<int>(ymax * image_height);

                // Clamp to image bounds
                left = std::max(0, std::min(left, image_width - 1));
                top = std::max(0, std::min(top, image_height - 1));
                right = std::max(0, std::min(right, image_width - 1));
                bottom = std::max(0, std::min(bottom, image_height - 1));

                int width = right - left;
                int height = bottom - top;

                if (width > 0 && height > 0) {
                    Detection det;
                    det.class_id = class_id;
                    det.confidence = confidence;
                    det.bbox = cv::Rect(left, top, width, height);
                    detections.push_back(det);
                }
            }
        }
    }

    return detections;
}

std::vector<Detection> TensorRTInference::infer(const cv::Mat& image,
                                                float confidence_threshold,
                                                int target_class_id)
{
    if (!context_ || image.empty()) {
        return std::vector<Detection>();
    }

    auto start = std::chrono::high_resolution_clock::now();

    // Preprocess image
    preprocessImage(image, input_buffer_.data());

    // Copy input to GPU
    cudaMemcpyAsync(buffers_[0], input_buffer_.data(), input_size_,
                    cudaMemcpyHostToDevice, stream_);

    // Run inference
    context_->enqueueV2(buffers_, stream_, nullptr);

    // Copy output to CPU
    cudaMemcpyAsync(output_buffer_.data(), buffers_[1], output_size_,
                    cudaMemcpyDeviceToHost, stream_);

    // Synchronize stream
    cudaStreamSynchronize(stream_);

    auto end = std::chrono::high_resolution_clock::now();
    last_inference_time_ms_ = std::chrono::duration<float, std::milli>(end - start).count();

    // Postprocess detections
    return postprocessDetections(output_buffer_.data(),
                                image.cols,
                                image.rows,
                                confidence_threshold,
                                target_class_id);
}

std::vector<Detection> TensorRTInference::postprocessYOLO(const float* output_buffer,
                                                           int image_width,
                                                           int image_height,
                                                           float confidence_threshold,
                                                           int target_class_id)
{
    std::vector<Detection> detections;
    detections.reserve(100);  // Pre-allocate for typical number of detections

    // YOLOv5 output format can be:
    // Older: [1, 25200, 85] = [batch, boxes, (x,y,w,h,obj,classes...)]
    // Newer: [1, 84, 8400] = [batch, (x,y,w,h,classes...), boxes] - NO separate objectness!
    // The newer format uses max class score as confidence directly

    const int total_elements = output_buffer_.size();

    // Auto-detect format based on total size
    int num_boxes, num_features;
    bool is_transposed;

    // Static variables to cache format detection (only detect once)
    static bool format_detected = false;
    static int cached_num_boxes = 0;
    static int cached_num_features = 0;
    static bool cached_is_transposed = false;
    static bool cached_has_objectness = false;

    if (!format_detected) {
        // First time: detect format
        if (total_elements == 8400 * 84) {
            // Newer YOLOv5 format: [1, 84, 8400]
            cached_num_boxes = 8400;
            cached_num_features = 84;  // 4 bbox + 80 classes (NO objectness!)
            cached_is_transposed = true;
            cached_has_objectness = false;
            std::cout << "[TensorRT] Detected YOLOv5 format: [1, 84, 8400] - Transposed, no objectness" << std::endl;
        } else if (total_elements == 25200 * 85) {
            // Older YOLOv5 format: could be [1, 25200, 85] or [1, 85, 25200]
            cached_num_boxes = 25200;
            cached_num_features = 85;
            cached_has_objectness = true;

            // Detect if transposed by checking if values look like scores
            float val_idx_4 = output_buffer[4];
            float val_idx_100800 = output_buffer[4 * 25200];  // 4*25200

            if (val_idx_100800 >= 0.0f && val_idx_100800 <= 1.0f &&
                (val_idx_4 < 0.0f || val_idx_4 > 1.0f)) {
                cached_is_transposed = true;
                std::cout << "[TensorRT] Detected YOLOv5 format: [1, 85, 25200] - Transposed" << std::endl;
            } else {
                cached_is_transposed = false;
                std::cout << "[TensorRT] Detected YOLOv5 format: [1, 25200, 85] - Standard" << std::endl;
            }
        } else {
            std::cerr << "[TensorRT] ERROR: Unknown output format! Total elements: " << total_elements << std::endl;
            return detections;
        }
        format_detected = true;
    }

    // Use cached values
    num_boxes = cached_num_boxes;
    num_features = cached_num_features;
    is_transposed = cached_is_transposed;

    const bool has_objectness = cached_has_objectness;

    for (int i = 0; i < num_boxes; ++i) {
        float center_x, center_y, width, height;
        float person_class_score;
        float confidence;

        if (is_transposed) {
            // Format: [1, features, boxes] - data is interleaved by feature
            center_x = output_buffer[0 * num_boxes + i];
            center_y = output_buffer[1 * num_boxes + i];
            width = output_buffer[2 * num_boxes + i];
            height = output_buffer[3 * num_boxes + i];

            if (has_objectness) {
                // [1, 85, boxes]: 4 bbox + objectness + 80 classes
                float objectness = output_buffer[4 * num_boxes + i];
                person_class_score = output_buffer[(5 + target_class_id) * num_boxes + i];
                confidence = objectness * person_class_score;
            } else {
                // [1, 84, boxes]: 4 bbox + 80 classes (NO objectness)
                person_class_score = output_buffer[(4 + target_class_id) * num_boxes + i];
                confidence = person_class_score;  // Class score IS the confidence
            }
        } else {
            // Format: [1, boxes, features] - data is contiguous per box
            int base_idx = i * num_features;
            center_x = output_buffer[base_idx + 0];
            center_y = output_buffer[base_idx + 1];
            width = output_buffer[base_idx + 2];
            height = output_buffer[base_idx + 3];

            if (has_objectness) {
                float objectness = output_buffer[base_idx + 4];
                person_class_score = output_buffer[base_idx + 5 + target_class_id];
                confidence = objectness * person_class_score;
            } else {
                person_class_score = output_buffer[base_idx + 4 + target_class_id];
                confidence = person_class_score;
            }
        }

        // Filter by confidence threshold
        if (confidence < confidence_threshold) continue;

        // Sanity check on values - coordinates should be reasonable
        if (center_x < 0 || center_x > 1000 || center_y < 0 || center_y > 1000 ||
            width < 0 || width > 1000 || height < 0 || height > 1000) {
            continue;  // Skip invalid boxes
        }

        // Convert from letterbox coordinates to original image coordinates
        // Remove padding first, then unscale
        center_x = (center_x - static_cast<float>(pad_w_)) / scale_;
        center_y = (center_y - static_cast<float>(pad_h_)) / scale_;
        width = width / scale_;
        height = height / scale_;

        // Convert from center format to corner format
        float x1 = center_x - width * 0.5f;
        float y1 = center_y - height * 0.5f;

        // Clamp to image bounds
        x1 = std::max(0.0f, std::min(x1, static_cast<float>(image_width - 1)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(image_height - 1)));
        width = std::max(1.0f, std::min(width, static_cast<float>(image_width) - x1));
        height = std::max(1.0f, std::min(height, static_cast<float>(image_height) - y1));

        // Additional sanity check - skip tiny or huge boxes
        if (width < 10 || height < 10 || width > image_width || height > image_height) {
            continue;
        }

        Detection det;
        det.class_id = target_class_id;
        det.confidence = confidence;
        det.bbox = cv::Rect(static_cast<int>(x1), static_cast<int>(y1),
                           static_cast<int>(width), static_cast<int>(height));
        detections.push_back(det);
    }

    // Apply Non-Maximum Suppression (only if we have detections)
    if (detections.empty()) return detections;
    return nms(detections, 0.45f);
}

std::vector<Detection> TensorRTInference::nms(std::vector<Detection>& detections, float nms_threshold)
{
    if (detections.empty()) return detections;

    // Sort by confidence (descending)
    std::sort(detections.begin(), detections.end(),
              [](const Detection& a, const Detection& b) {
                  return a.confidence > b.confidence;
              });

    std::vector<Detection> result;
    std::vector<bool> suppressed(detections.size(), false);

    for (size_t i = 0; i < detections.size(); ++i) {
        if (suppressed[i]) continue;

        result.push_back(detections[i]);

        // Suppress overlapping boxes
        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (suppressed[j]) continue;

            float overlap = iou(detections[i].bbox, detections[j].bbox);
            if (overlap > nms_threshold) {
                suppressed[j] = true;
            }
        }
    }

    return result;
}

float TensorRTInference::iou(const cv::Rect& box1, const cv::Rect& box2)
{
    // Calculate intersection
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    int intersection_width = std::max(0, x2 - x1);
    int intersection_height = std::max(0, y2 - y1);
    int intersection_area = intersection_width * intersection_height;

    // Calculate union
    int box1_area = box1.width * box1.height;
    int box2_area = box2.width * box2.height;
    int union_area = box1_area + box2_area - intersection_area;

    if (union_area == 0) return 0.0f;

    return static_cast<float>(intersection_area) / static_cast<float>(union_area);
}

} // namespace csi_camera_cpp
