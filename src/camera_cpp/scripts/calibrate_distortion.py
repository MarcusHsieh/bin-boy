#!/usr/bin/env python3
"""
Interactive Distortion Calibration Tool for CSI Camera
Allows real-time adjustment of barrel distortion correction parameters.

Usage:
    ros2 run camera_cpp calibrate_distortion.py

Controls:
    - Use trackbars to adjust parameters in real-time
    - Press 'q' to quit and print final values
    - Press 's' to save current settings to file
    - Press 'r' to reset to defaults
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import threading


class DistortionCalibrator(Node):
    def __init__(self):
        super().__init__('distortion_calibrator')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        # Create subscriber to camera feed
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Calibration parameters (stored as integers for trackbar, converted later)
        self.params = {
            'k1': 25,           # -0.25 default (stored as 25, divided by 100)
            'k2': 0,            # Second-order distortion
            'crop_percent': 70, # Center crop percentage
            'mode': 0,          # 0=barrel correction, 1=center crop, 2=no correction
            'crop_left': 5,     # Edge crop percentage
            'crop_right': 5,
            'crop_top': 5,
            'crop_bottom': 5,
        }

        # Camera matrix (will be updated based on image size)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.map1 = None
        self.map2 = None

        # Create OpenCV window and trackbars
        self.window_name = 'Distortion Calibration Tool'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)

        # Create trackbars
        cv2.createTrackbar('Mode (0=Barrel, 1=Center, 2=None)', self.window_name,
                          self.params['mode'], 2, self.on_mode_change)
        cv2.createTrackbar('k1 x100 (barrel)', self.window_name,
                          self.params['k1'], 60, self.on_k1_change)
        cv2.createTrackbar('k2 x100 (2nd order)', self.window_name,
                          self.params['k2'], 30, self.on_k2_change)
        cv2.createTrackbar('Center Crop %', self.window_name,
                          self.params['crop_percent'], 100, self.on_crop_percent_change)
        cv2.createTrackbar('Edge Crop L %', self.window_name,
                          self.params['crop_left'], 30, self.on_crop_left_change)
        cv2.createTrackbar('Edge Crop R %', self.window_name,
                          self.params['crop_right'], 30, self.on_crop_right_change)
        cv2.createTrackbar('Edge Crop T %', self.window_name,
                          self.params['crop_top'], 30, self.on_crop_top_change)
        cv2.createTrackbar('Edge Crop B %', self.window_name,
                          self.params['crop_bottom'], 30, self.on_crop_bottom_change)

        self.get_logger().info('Distortion Calibrator started!')
        self.get_logger().info('Waiting for images on /image_raw...')
        self.get_logger().info('Controls: q=quit, s=save, r=reset')

    def on_mode_change(self, val):
        self.params['mode'] = val
        self.get_logger().info(f'Mode: {["Barrel Correction", "Center Crop", "No Correction"][val]}')

    def on_k1_change(self, val):
        self.params['k1'] = val
        self.update_distortion_maps()

    def on_k2_change(self, val):
        self.params['k2'] = val
        self.update_distortion_maps()

    def on_crop_percent_change(self, val):
        self.params['crop_percent'] = max(10, val)  # Minimum 10%

    def on_crop_left_change(self, val):
        self.params['crop_left'] = val

    def on_crop_right_change(self, val):
        self.params['crop_right'] = val

    def on_crop_top_change(self, val):
        self.params['crop_top'] = val

    def on_crop_bottom_change(self, val):
        self.params['crop_bottom'] = val

    def update_distortion_maps(self):
        """Recompute undistortion maps when k1/k2 changes"""
        if self.camera_matrix is None:
            return

        k1 = -self.params['k1'] / 100.0  # Convert to negative decimal
        k2 = -self.params['k2'] / 100.0

        self.dist_coeffs = np.array([k1, k2, 0.0, 0.0, 0.0], dtype=np.float64)

        height, width = self.latest_frame.shape[:2] if self.latest_frame is not None else (480, 640)

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None,
            self.camera_matrix, (width, height),
            cv2.CV_32FC1
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Initialize camera matrix on first frame
            if self.camera_matrix is None:
                height, width = frame.shape[:2]
                self.camera_matrix = np.array([
                    [width * 0.5, 0, width / 2.0],
                    [0, height * 0.5, height / 2.0],
                    [0, 0, 1]
                ], dtype=np.float64)
                self.update_distortion_maps()
                self.get_logger().info(f'Initialized for {width}x{height} images')

            # Thread-safe frame update
            with self.frame_lock:
                self.latest_frame = frame.copy()

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

    def apply_edge_crop(self, frame):
        """Crop edges to remove vignetting"""
        h, w = frame.shape[:2]

        crop_l = int(w * self.params['crop_left'] / 100.0)
        crop_r = int(w * self.params['crop_right'] / 100.0)
        crop_t = int(h * self.params['crop_top'] / 100.0)
        crop_b = int(h * self.params['crop_bottom'] / 100.0)

        if crop_l + crop_r >= w or crop_t + crop_b >= h:
            return frame  # Invalid crop

        return frame[crop_t:h-crop_b, crop_l:w-crop_r].copy()

    def apply_barrel_correction(self, frame):
        """Apply barrel distortion correction"""
        if self.map1 is None or self.map2 is None:
            return frame

        corrected = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
        return corrected

    def apply_center_crop(self, frame):
        """Use only center portion of image"""
        h, w = frame.shape[:2]
        crop_pct = self.params['crop_percent'] / 100.0

        new_w = int(w * crop_pct)
        new_h = int(h * crop_pct)

        if new_w <= 0 or new_h <= 0:
            return frame

        x_offset = (w - new_w) // 2
        y_offset = (h - new_h) // 2

        cropped = frame[y_offset:y_offset+new_h, x_offset:x_offset+new_w].copy()

        # Resize back to original size
        return cv2.resize(cropped, (w, h))

    def process_frame(self, frame):
        """Apply selected correction to frame"""
        mode = self.params['mode']

        # Always apply edge crop first
        processed = self.apply_edge_crop(frame)

        # Apply selected distortion correction
        if mode == 0:  # Barrel correction
            processed = self.apply_barrel_correction(processed)
        elif mode == 1:  # Center crop only
            processed = self.apply_center_crop(processed)
        # mode == 2: No correction, just edge crop

        # Add text overlay with current settings
        self.add_info_overlay(processed)

        return processed

    def add_info_overlay(self, frame):
        """Add text showing current parameters"""
        mode_names = ['Barrel Correction', 'Center Crop Only', 'No Correction']
        k1 = -self.params['k1'] / 100.0
        k2 = -self.params['k2'] / 100.0

        # Create semi-transparent overlay
        overlay = frame.copy()
        h, w = frame.shape[:2]
        cv2.rectangle(overlay, (5, 5), (550, 230), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        # Add text
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_pos = 25
        line_height = 22

        cv2.putText(frame, f'Mode: {mode_names[self.params["mode"]]}',
                   (10, y_pos), font, 0.6, (0, 255, 0), 2)
        y_pos += line_height

        if self.params['mode'] == 0:  # Barrel correction
            # Show both slider value and actual negative value
            cv2.putText(frame, f'k1 = {k1:.3f}  (slider: {self.params["k1"]}/100)',
                       (10, y_pos), font, 0.6, (0, 255, 255), 2)
            y_pos += line_height
            cv2.putText(frame, f'k2 = {k2:.3f}  (slider: {self.params["k2"]}/100)',
                       (10, y_pos), font, 0.6, (0, 255, 255), 2)
            y_pos += line_height
            # Add hint
            cv2.putText(frame, 'Tip: Lines curve OUT -> increase slider',
                       (10, y_pos), font, 0.45, (100, 255, 100), 1)
            y_pos += line_height
            cv2.putText(frame, '     Lines curve IN -> decrease slider',
                       (10, y_pos), font, 0.45, (100, 255, 100), 1)
            y_pos += line_height
        elif self.params['mode'] == 1:  # Center crop
            cv2.putText(frame, f'Center Crop: {self.params["crop_percent"]}%',
                       (10, y_pos), font, 0.6, (0, 255, 255), 2)
            y_pos += line_height

        cv2.putText(frame, f'Edge Crop: L{self.params["crop_left"]}% R{self.params["crop_right"]}%',
                   (10, y_pos), font, 0.5, (200, 200, 200), 1)
        y_pos += line_height
        cv2.putText(frame, f'           T{self.params["crop_top"]}% B{self.params["crop_bottom"]}%',
                   (10, y_pos), font, 0.5, (200, 200, 200), 1)
        y_pos += line_height * 2

        cv2.putText(frame, 'q: Quit | s: Save | r: Reset',
                   (10, y_pos), font, 0.5, (255, 255, 255), 1)

    def print_final_values(self):
        """Print final parameter values for launch file"""
        k1 = -self.params['k1'] / 100.0
        k2 = -self.params['k2'] / 100.0

        print('\n' + '='*60)
        print('FINAL CALIBRATION VALUES')
        print('='*60)
        print('\nLaunch file parameters:\n')

        if self.params['mode'] == 0:  # Barrel correction
            print(f'enable_distortion_correction:=true \\')
            print(f'use_center_crop_only:=false \\')
            print(f'barrel_distortion_k1:={k1:.3f} \\')
            if k2 != 0:
                print(f'# Note: k2 not yet implemented, current: {k2:.3f}')
        elif self.params['mode'] == 1:  # Center crop
            print(f'enable_distortion_correction:=true \\')
            print(f'use_center_crop_only:=true \\')
            print(f'center_crop_percentage:={self.params["crop_percent"]/100.0:.2f} \\')
        else:  # No correction
            print(f'enable_distortion_correction:=false \\')

        print(f'crop_left:={self.params["crop_left"]/100.0:.2f} \\')
        print(f'crop_right:={self.params["crop_right"]/100.0:.2f} \\')
        print(f'crop_top:={self.params["crop_top"]/100.0:.2f} \\')
        print(f'crop_bottom:={self.params["crop_bottom"]/100.0:.2f}')

        print('\n' + '='*60 + '\n')

    def save_to_file(self):
        """Save current settings to file"""
        filename = '/home/jetson/bin-boy/camera_calibration.txt'
        k1 = -self.params['k1'] / 100.0
        k2 = -self.params['k2'] / 100.0

        with open(filename, 'w') as f:
            f.write('# Camera Distortion Calibration Settings\n')
            f.write(f'# Generated by calibrate_distortion.py\n\n')
            f.write(f'mode: {self.params["mode"]} # 0=barrel, 1=center, 2=none\n')
            f.write(f'k1: {k1:.3f}\n')
            f.write(f'k2: {k2:.3f}\n')
            f.write(f'center_crop_percentage: {self.params["crop_percent"]/100.0:.2f}\n')
            f.write(f'crop_left: {self.params["crop_left"]/100.0:.2f}\n')
            f.write(f'crop_right: {self.params["crop_right"]/100.0:.2f}\n')
            f.write(f'crop_top: {self.params["crop_top"]/100.0:.2f}\n')
            f.write(f'crop_bottom: {self.params["crop_bottom"]/100.0:.2f}\n')

        self.get_logger().info(f'Settings saved to {filename}')
        print(f'\nSettings saved to {filename}\n')

    def reset_to_defaults(self):
        """Reset all parameters to defaults"""
        self.params = {
            'k1': 25,
            'k2': 0,
            'crop_percent': 70,
            'mode': 0,
            'crop_left': 5,
            'crop_right': 5,
            'crop_top': 5,
            'crop_bottom': 5,
        }

        # Update trackbars
        cv2.setTrackbarPos('Mode (0=Barrel, 1=Center, 2=None)', self.window_name, self.params['mode'])
        cv2.setTrackbarPos('k1 x100 (barrel)', self.window_name, self.params['k1'])
        cv2.setTrackbarPos('k2 x100 (2nd order)', self.window_name, self.params['k2'])
        cv2.setTrackbarPos('Center Crop %', self.window_name, self.params['crop_percent'])
        cv2.setTrackbarPos('Edge Crop L %', self.window_name, self.params['crop_left'])
        cv2.setTrackbarPos('Edge Crop R %', self.window_name, self.params['crop_right'])
        cv2.setTrackbarPos('Edge Crop T %', self.window_name, self.params['crop_top'])
        cv2.setTrackbarPos('Edge Crop B %', self.window_name, self.params['crop_bottom'])

        self.update_distortion_maps()
        self.get_logger().info('Reset to default values')

    def spin_thread(self):
        """ROS2 spin in separate thread"""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        """Main loop - display corrected frames"""
        # Start ROS2 spinning in separate thread
        ros_thread = threading.Thread(target=self.spin_thread, daemon=True)
        ros_thread.start()

        self.get_logger().info('Starting display loop...')

        try:
            while self.running and rclpy.ok():
                # Get latest frame (thread-safe)
                frame = None
                with self.frame_lock:
                    if self.latest_frame is not None:
                        frame = self.latest_frame.copy()

                # Process and display frame
                if frame is not None:
                    display_frame = self.process_frame(frame)
                    cv2.imshow(self.window_name, display_frame)
                else:
                    # Show waiting message if no frames yet
                    blank = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(blank, 'Waiting for camera images...',
                               (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.imshow(self.window_name, blank)

                # Handle keyboard input (MUST be in main thread for OpenCV)
                key = cv2.waitKey(30) & 0xFF  # 30ms = ~33 fps max
                if key == ord('q'):
                    self.get_logger().info('Quitting...')
                    self.print_final_values()
                    self.running = False
                    break
                elif key == ord('s'):
                    self.save_to_file()
                elif key == ord('r'):
                    self.reset_to_defaults()

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
            self.print_final_values()

        finally:
            self.running = False
            cv2.destroyAllWindows()
            ros_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)

    calibrator = DistortionCalibrator()

    print('\n' + '='*60)
    print('INTERACTIVE DISTORTION CALIBRATION TOOL')
    print('='*60)
    print('\nInstructions:')
    print('  1. Point camera at something with STRAIGHT LINES')
    print('     (doorframe, table edge, wall corner)')
    print('  2. Adjust sliders until lines appear straight')
    print('  3. Press "s" to save settings')
    print('  4. Press "q" to quit and print launch file values')
    print('  5. Press "r" to reset to defaults')
    print('\nSlider Guide:')
    print('  - Mode: 0=Barrel Correction, 1=Center Crop, 2=None')
    print('  - k1: Increase if lines curve outward (barrel)')
    print('  - k1: Decrease if lines curve inward (over-corrected)')
    print('  - Center Crop: Use less % for straighter lines')
    print('  - Edge Crop: Remove vignetting from corners')
    print('='*60 + '\n')

    try:
        calibrator.run()
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
