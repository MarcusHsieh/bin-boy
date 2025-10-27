#!/usr/bin/env python3
"""
Test Script: Sensor Validation
Monitors all sensors and validates they're publishing correct data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
import time


class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Sensor data received flags
        self.sensors_status = {
            'lidar': {'received': False, 'count': 0, 'last_time': None},
            'camera': {'received': False, 'count': 0, 'last_time': None},
            'camera_info': {'received': False, 'count': 0, 'last_time': None},
            'imu': {'received': False, 'count': 0, 'last_time': None},
            'odom': {'received': False, 'count': 0, 'last_time': None},
        }

        # Create subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # Timer for status reporting
        self.status_timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info('Sensor Validator started')
        self.get_logger().info('Monitoring: LIDAR, Camera, Camera Info, IMU, Odometry')
        self.get_logger().info('')

    def lidar_callback(self, msg):
        """Validate LIDAR data"""
        self.sensors_status['lidar']['received'] = True
        self.sensors_status['lidar']['count'] += 1
        self.sensors_status['lidar']['last_time'] = self.get_clock().now()

        # Validate data
        if self.sensors_status['lidar']['count'] == 1:
            self.get_logger().info(f'LIDAR: {len(msg.ranges)} points, '
                                   f'range [{msg.range_min:.2f}, {msg.range_max:.2f}]m, '
                                   f'angle [{msg.angle_min:.2f}, {msg.angle_max:.2f}] rad')

    def camera_callback(self, msg):
        """Validate camera data"""
        self.sensors_status['camera']['received'] = True
        self.sensors_status['camera']['count'] += 1
        self.sensors_status['camera']['last_time'] = self.get_clock().now()

        if self.sensors_status['camera']['count'] == 1:
            self.get_logger().info(f'Camera: {msg.width}x{msg.height}, '
                                   f'encoding={msg.encoding}')

    def camera_info_callback(self, msg):
        """Validate camera info"""
        self.sensors_status['camera_info']['received'] = True
        self.sensors_status['camera_info']['count'] += 1
        self.sensors_status['camera_info']['last_time'] = self.get_clock().now()

        if self.sensors_status['camera_info']['count'] == 1:
            self.get_logger().info(f'Camera Info: {msg.width}x{msg.height}, '
                                   f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}')

    def imu_callback(self, msg):
        """Validate IMU data"""
        self.sensors_status['imu']['received'] = True
        self.sensors_status['imu']['count'] += 1
        self.sensors_status['imu']['last_time'] = self.get_clock().now()

        if self.sensors_status['imu']['count'] == 1:
            has_orientation = msg.orientation_covariance[0] != -1.0
            self.get_logger().info(f'IMU: orientation={has_orientation}, '
                                   f'accel=[{msg.linear_acceleration.x:.2f}, '
                                   f'{msg.linear_acceleration.y:.2f}, '
                                   f'{msg.linear_acceleration.z:.2f}] m/s^2')

    def odom_callback(self, msg):
        """Validate odometry data"""
        self.sensors_status['odom']['received'] = True
        self.sensors_status['odom']['count'] += 1
        self.sensors_status['odom']['last_time'] = self.get_clock().now()

        if self.sensors_status['odom']['count'] == 1:
            self.get_logger().info(f'Odometry: frame={msg.header.frame_id}, '
                                   f'child_frame={msg.child_frame_id}')

    def print_status(self):
        """Print sensor status summary"""
        now = self.get_clock().now()

        status_lines = []
        all_ok = True

        for sensor_name, status in self.sensors_status.items():
            if status['received']:
                # Calculate Hz
                if status['count'] > 0:
                    # Estimate Hz based on count (rough estimate)
                    hz_estimate = status['count'] / (
                        (now - status['last_time']).nanoseconds / 1e9 + 1.0
                    ) if status['last_time'] else 0

                    # Check if recent (within last 2 seconds)
                    time_since = (now - status['last_time']).nanoseconds / 1e9
                    is_recent = time_since < 2.0

                    status_str = f"{'✓' if is_recent else '✗'} {sensor_name:12s}: " \
                                 f"{status['count']:4d} msgs, " \
                                 f"{time_since:5.2f}s ago"
                    status_lines.append(status_str)

                    if not is_recent:
                        all_ok = False
                else:
                    status_lines.append(f"✗ {sensor_name:12s}: no data")
                    all_ok = False
            else:
                status_lines.append(f"✗ {sensor_name:12s}: never received")
                all_ok = False

        # Print status
        self.get_logger().info('--- Sensor Status ---')
        for line in status_lines:
            self.get_logger().info(line)

        if all_ok:
            self.get_logger().info('Status: ALL SENSORS OK')
        else:
            self.get_logger().warn('Status: SOME SENSORS MISSING OR DELAYED')
        self.get_logger().info('')

    def run_test(self, duration=10):
        """Run sensor validation test for specified duration"""
        self.get_logger().info(f'Running sensor test for {duration} seconds...')
        self.get_logger().info('Waiting for sensor data...')
        self.get_logger().info('')

        # Spin for duration
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Final report
        self.get_logger().info('')
        self.get_logger().info('=== FINAL SENSOR REPORT ===')
        all_ok = True
        for sensor_name, status in self.sensors_status.items():
            if status['received'] and status['count'] > 0:
                self.get_logger().info(f'✓ {sensor_name}: {status["count"]} messages received')
            else:
                self.get_logger().error(f'✗ {sensor_name}: NO DATA')
                all_ok = False

        self.get_logger().info('')
        if all_ok:
            self.get_logger().info('=== ALL SENSORS VALIDATED ===')
        else:
            self.get_logger().error('=== SOME SENSORS FAILED ===')


def main(args=None):
    rclpy.init(args=args)

    validator = SensorValidator()

    try:
        validator.run_test(duration=10)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
