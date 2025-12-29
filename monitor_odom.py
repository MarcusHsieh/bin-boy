#!/usr/bin/env python3
"""
Simple odometry monitor for manual calibration
Shows distance traveled from starting point
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.start_x = None
        self.start_y = None
        self.start_theta = None

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Get yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_theta = theta
            print(f"\nStarting position set: x={x:.4f}, y={y:.4f}, theta={math.degrees(theta):.1f}°")
            print("\n" + "="*60)
            print("NOW DRIVE THE ROBOT - Commands in another terminal:")
            print("="*60)
            print("\n# Drive forward 0.2 m/s:")
            print('ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"')
            print("\n# Stop (Ctrl+C in cmd_vel terminal)")
            print("\n# Rotate 0.3 rad/s:")
            print('ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"')
            print("\n" + "="*60 + "\n")
        else:
            dx = x - self.start_x
            dy = y - self.start_y
            dist = math.sqrt(dx*dx + dy*dy)
            dtheta = theta - self.start_theta

            # Normalize angle
            while dtheta > math.pi:
                dtheta -= 2*math.pi
            while dtheta < -math.pi:
                dtheta += 2*math.pi

            print(f"\rDistance: {dist:.4f}m | dx: {dx:+.4f}m | dy: {dy:+.4f}m | Rotation: {math.degrees(dtheta):+.1f}°   ", end='', flush=True)

def main(args=None):
    rclpy.init(args=args)

    print("\n" + "="*60)
    print("MANUAL ODOMETRY CALIBRATION")
    print("="*60)
    print("\nThis monitors odometry distance traveled.")
    print("\nSteps:")
    print("  1. Mark robot's current position with tape")
    print("  2. Drive robot in another terminal (commands shown below)")
    print("  3. Stop when you want to measure")
    print("  4. Mark final position with tape")
    print("  5. Press Ctrl+C here")
    print("  6. Measure actual distance with tape measure")
    print("  7. Compare to odometry reading")
    print("\n" + "="*60)

    monitor = OdomMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("MEASUREMENT STOPPED")
        print("="*60)
        print("\nNow measure ACTUAL distance with tape measure.")
        print("\nCalculate correction:")
        print("  correction_factor = actual_distance / odometry_distance")
        print("  new_wheel_radius = 0.07 * correction_factor")
        print("\nEdit: src/bin_boy_control/launch/kiwi_drive.launch.py")
        print("Change 'wheel_radius' parameter to new value")
        print("="*60 + "\n")

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
