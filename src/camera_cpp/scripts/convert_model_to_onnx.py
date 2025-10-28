#!/usr/bin/env python3
"""
Script to convert MobileNet-SSD Caffe model to ONNX format for TensorRT.

This script uses MMdnn (Multi-framework deep neural network conversion tool)
or directly attempts ONNX conversion if available.

Usage:
    python3 convert_model_to_onnx.py

Requirements:
    pip3 install mmdnn
    or
    pip3 install caffe2pytorch torch onnx

Note: If conversion fails, you can download a pre-trained ONNX MobileNet-SSD model
from ONNX Model Zoo or use a PyTorch SSD model converted to ONNX.
"""

import os
import sys
import subprocess

def check_and_install_dependencies():
    """Check if required packages are installed."""
    try:
        import onnx
        print("✓ ONNX is installed")
    except ImportError:
        print("✗ ONNX not found. Installing...")
        subprocess.run([sys.executable, "-m", "pip", "install", "onnx"])

    # Check for conversion tools
    try:
        import mmdnn
        print("✓ MMdnn is installed")
        return "mmdnn"
    except ImportError:
        print("✗ MMdnn not found")
        print("\nTo install MMdnn:")
        print("  pip3 install mmdnn")
        print("\nAlternatively, you can:")
        print("  1. Use PyTorch to convert: pip3 install torch torchvision")
        print("  2. Download pre-converted ONNX model from ONNX Model Zoo")
        return None

def convert_with_mmdnn(prototxt_path, caffemodel_path, output_onnx_path):
    """Convert Caffe model to ONNX using MMdnn."""
    print("\n=== Converting with MMdnn ===")

    # Step 1: Convert Caffe to IR (Intermediate Representation)
    ir_pb_path = output_onnx_path.replace('.onnx', '.pb')
    ir_npy_path = output_onnx_path.replace('.onnx', '.npy')

    print("Step 1: Converting Caffe to IR...")
    cmd = [
        'mmconvert',
        '-sf', 'caffe',
        '-in', caffemodel_path,
        '-iw', caffemodel_path,
        '--inNodeName', 'data',
        '-df', 'onnx',
        '-om', output_onnx_path
    ]

    try:
        subprocess.run(cmd, check=True)
        print(f"✓ Successfully converted to ONNX: {output_onnx_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Conversion failed: {e}")
        return False

def download_pretrained_onnx_model(output_path):
    """Download a pre-trained MobileNet-SSD ONNX model."""
    print("\n=== Downloading pre-trained ONNX model ===")
    print("Note: This downloads a compatible SSD MobileNet model")

    # ONNX Model Zoo URL for SSD MobileNet
    # Note: This is a simplified approach - user may need to adapt
    url = "https://github.com/onnx/models/raw/main/vision/object_detection_segmentation/ssd-mobilenetv1/model/ssd_mobilenet_v1_12.onnx"

    try:
        import urllib.request
        print(f"Downloading from ONNX Model Zoo...")
        urllib.request.urlretrieve(url, output_path)
        print(f"✓ Downloaded model to: {output_path}")
        return True
    except Exception as e:
        print(f"✗ Download failed: {e}")
        print("\nManual download instructions:")
        print(f"  1. Visit: https://github.com/onnx/models")
        print(f"  2. Download SSD MobileNet ONNX model")
        print(f"  3. Save to: {output_path}")
        return False

def main():
    """Main conversion function."""
    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(os.path.dirname(script_dir), 'models')

    prototxt_path = os.path.join(models_dir, 'MobileNetSSD_deploy.prototxt')
    caffemodel_path = os.path.join(models_dir, 'MobileNetSSD_deploy.caffemodel')
    output_onnx_path = os.path.join(models_dir, 'mobilenet_ssd.onnx')

    print("=== MobileNet-SSD Caffe to ONNX Converter ===\n")
    print(f"Input Caffe model: {caffemodel_path}")
    print(f"Output ONNX model: {output_onnx_path}")

    # Check if Caffe model exists
    if not os.path.exists(caffemodel_path):
        print(f"\n✗ Caffe model not found at: {caffemodel_path}")
        return False

    # Check dependencies
    converter = check_and_install_dependencies()

    success = False

    if converter == "mmdnn":
        success = convert_with_mmdnn(prototxt_path, caffemodel_path, output_onnx_path)
    else:
        print("\n=== Alternative Options ===")
        print("\nOption 1: Install conversion tools")
        print("  pip3 install mmdnn")
        print("  Then run this script again")

        print("\nOption 2: Download pre-trained ONNX model")
        response = input("\nWould you like to download a pre-trained ONNX model? (y/n): ")
        if response.lower() == 'y':
            success = download_pretrained_onnx_model(output_onnx_path)

    if success:
        print("\n" + "="*50)
        print("✓ SUCCESS!")
        print("="*50)
        print(f"\nONNX model is ready at: {output_onnx_path}")
        print("\nNext steps:")
        print("  1. The model will be automatically converted to TensorRT engine on first run")
        print("  2. Launch your detector node:")
        print("     ros2 launch csi_camera_cpp csi_camera_ipc.launch.py run_detector:=true")
        print("\nThe TensorRT engine will be built automatically and saved for future use.")
    else:
        print("\n" + "="*50)
        print("⚠ MANUAL SETUP REQUIRED")
        print("="*50)
        print("\nPlease provide an ONNX model manually:")
        print(f"  Location: {output_onnx_path}")
        print("\nOptions:")
        print("  1. Convert using PyTorch:")
        print("     - Load MobileNet-SSD in PyTorch")
        print("     - Export to ONNX using torch.onnx.export()")
        print("  2. Download from ONNX Model Zoo:")
        print("     - https://github.com/onnx/models")
        print("  3. Use a different object detection model (YOLO, SSD, etc.)")

    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
