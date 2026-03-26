#!/usr/bin/env python3
"""
Download and prepare YOLOv5n ONNX model for TensorRT inference on Jetson Nano.

This script downloads the YOLOv5n model and exports it to ONNX format
optimized for TensorRT with FP16 precision.

Usage:
    python3 download_yolov5n.py
"""

import os
import sys
import subprocess

def install_dependencies():
    """Install required packages."""
    print("=== Installing Dependencies ===\n")

    packages = [
        'torch',
        'torchvision',
        'ultralytics',
        'onnx'
    ]

    for package in packages:
        try:
            __import__(package)
            print(f"✓ {package} already installed")
        except ImportError:
            print(f"Installing {package}...")
            subprocess.run([sys.executable, "-m", "pip", "install", package, "-q"], check=True)
            print(f"✓ {package} installed")

def download_and_export_yolov5n(output_dir):
    """Download YOLOv5n and export to ONNX."""
    print("\n=== Downloading YOLOv5n Model ===\n")

    try:
        import torch
        from ultralytics import YOLO

        # Download YOLOv5n pretrained model
        print("Downloading YOLOv5n pretrained weights...")
        model = YOLO('yolov5n.pt')
        print("✓ Model downloaded successfully")

        # Export to ONNX format optimized for TensorRT
        print("\nExporting to ONNX format...")
        print("Input size: 640x640")
        print("Dynamic batch: False (optimized for single inference)")
        print("Simplify: True (optimized graph)")

        onnx_path = model.export(
            format='onnx',
            imgsz=640,
            simplify=True,
            dynamic=False,
            opset=12
        )

        print(f"✓ ONNX model exported: {onnx_path}")

        # Move to models directory
        import shutil
        dest_path = os.path.join(output_dir, 'yolov5n.onnx')
        shutil.move(onnx_path, dest_path)
        print(f"✓ Model moved to: {dest_path}")

        return dest_path

    except Exception as e:
        print(f"✗ Error: {e}")
        return None

def verify_onnx_model(onnx_path):
    """Verify the ONNX model is valid."""
    print("\n=== Verifying ONNX Model ===\n")

    try:
        import onnx

        model = onnx.load(onnx_path)
        onnx.checker.check_model(model)
        print("✓ ONNX model is valid")

        # Print model info
        print("\nModel Information:")
        print(f"  IR Version: {model.ir_version}")
        print(f"  Producer: {model.producer_name}")

        # Input info
        input_info = model.graph.input[0]
        print(f"\nInput:")
        print(f"  Name: {input_info.name}")
        dims = [d.dim_value for d in input_info.type.tensor_type.shape.dim]
        print(f"  Shape: {dims}")

        # Output info
        print(f"\nOutputs: {len(model.graph.output)}")
        for i, output in enumerate(model.graph.output):
            print(f"  Output {i}: {output.name}")
            dims = [d.dim_value for d in output.type.tensor_type.shape.dim]
            print(f"    Shape: {dims}")

        return True

    except Exception as e:
        print(f"✗ Verification failed: {e}")
        return False

def create_model_info(output_dir):
    """Create a model info file."""
    info_path = os.path.join(output_dir, 'yolov5n_info.txt')

    info_content = """YOLOv5n Model Information
========================

Model: YOLOv5n (Nano)
Architecture: YOLOv5
Input Size: 640x640x3
Format: ONNX (Opset 12)
Precision: FP32 (will be converted to FP16 by TensorRT)

Classes: 80 (COCO dataset)
Person Class ID: 0

Performance (Jetson Nano with TensorRT FP16):
- Expected Inference Time: 10-20ms
- Expected FPS: 40-50
- Expected mAP50: ~45%

Preprocessing:
- Input: BGR image (any size)
- Resize: 640x640 (letterbox with padding)
- Normalization: /255.0 (0-1 range)
- Format: NCHW (batch, channels, height, width)

Output Format:
- Shape: [1, 25200, 85]
  - 25200 = number of detection boxes
  - 85 = [x, y, w, h, objectness, class0_prob, ..., class79_prob]

- Box format: [center_x, center_y, width, height] (normalized 0-1)
- Confidence threshold: 0.5 (recommended)
- NMS IoU threshold: 0.45 (recommended)

Post-processing:
1. Filter by objectness score (> 0.5)
2. Filter by person class confidence (> 0.5)
3. Apply NMS (Non-Maximum Suppression)
4. Convert box coordinates to pixel values

Advantages over MobileNet-SSD:
- 2x better accuracy (45% vs 23% mAP50)
- Better small object detection
- Better occlusion handling
- Modern architecture (2021)
- Active development and support

Trade-offs:
- Slightly slower (10-20ms vs 5-15ms)
- Still real-time on Jetson Nano (40+ FPS)
"""

    with open(info_path, 'w') as f:
        f.write(info_content)

    print(f"\n✓ Model info saved to: {info_path}")

def main():
    """Main function."""
    print("=" * 60)
    print("YOLOv5n ONNX Model Downloader for Jetson Nano")
    print("=" * 60)

    # Determine output directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(os.path.dirname(script_dir), 'models')

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)
        print(f"✓ Created models directory: {models_dir}")

    print(f"\nOutput directory: {models_dir}")

    # Install dependencies
    install_dependencies()

    # Download and export model
    onnx_path = download_and_export_yolov5n(models_dir)

    if not onnx_path:
        print("\n" + "=" * 60)
        print("✗ FAILED to download/export model")
        print("=" * 60)
        return False

    # Verify model
    if not verify_onnx_model(onnx_path):
        print("\n" + "=" * 60)
        print("⚠ WARNING: Model verification failed")
        print("=" * 60)
        return False

    # Create info file
    create_model_info(models_dir)

    # Success
    print("\n" + "=" * 60)
    print("✓ SUCCESS!")
    print("=" * 60)
    print(f"\nYOLOv5n ONNX model ready at:")
    print(f"  {onnx_path}")
    print(f"\nModel size: {os.path.getsize(onnx_path) / (1024*1024):.2f} MB")
    print("\nNext steps:")
    print("  1. Build the package: colcon build --packages-select csi_camera_cpp")
    print("  2. Launch detector: ros2 launch csi_camera_cpp csi_camera_ipc.launch.py run_detector:=true")
    print("  3. TensorRT will automatically build optimized engine on first run (~5-10 min)")
    print("\nExpected performance:")
    print("  - Inference time: 10-20ms")
    print("  - FPS: 40-50")
    print("  - Accuracy: 2x better than MobileNet-SSD")

    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
