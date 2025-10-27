#!/usr/bin/env python3
"""
ROS2 Node for MPU-6050 IMU
Publishes sensor_msgs/Imu data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time
import math


class MPU6050:
    """Driver for MPU-6050 IMU via I2C"""

    # MPU6050 Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE = 0x38

    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    TEMP_OUT_H = 0x41

    # I2C address (default, can be 0x69 with AD0 high)
    MPU6050_ADDR = 0x68

    def __init__(self, bus_number=1, address=0x68):
        """Initialize MPU-6050 on I2C bus"""
        self.bus = smbus2.SMBus(bus_number)
        self.address = address

        # Wake up MPU6050 (it starts in sleep mode)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)

        # Set sample rate (1kHz / (1 + SMPLRT_DIV))
        # For 100Hz: 1000/(1+9) = 100
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 9)

        # Configure DLPF (Digital Low Pass Filter)
        # DLPF_CFG = 3: Bandwidth ~44Hz for accel and gyro
        self.bus.write_byte_data(self.address, self.CONFIG, 3)

        # Gyro config: ±500°/s range
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x08)

        # Accel config: ±4g range
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x08)

        # Scaling factors
        # Gyro: 500°/s range = 65.5 LSB/(°/s)
        self.gyro_scale = 500.0 / 32768.0  # deg/s per LSB
        # Accel: 4g range = 8192 LSB/g
        self.accel_scale = 4.0 / 32768.0  # g per LSB
        self.gravity = 9.80665  # m/s^2

    def read_word(self, reg):
        """Read a 16-bit signed word from registers"""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low

        # Convert to signed
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    def read_accel(self):
        """Read accelerometer data (m/s^2)"""
        ax = self.read_word(self.ACCEL_XOUT_H) * self.accel_scale * self.gravity
        ay = self.read_word(self.ACCEL_XOUT_H + 2) * self.accel_scale * self.gravity
        az = self.read_word(self.ACCEL_XOUT_H + 4) * self.accel_scale * self.gravity
        return ax, ay, az

    def read_gyro(self):
        """Read gyroscope data (rad/s)"""
        gx = self.read_word(self.GYRO_XOUT_H) * self.gyro_scale * (math.pi / 180.0)
        gy = self.read_word(self.GYRO_XOUT_H + 2) * self.gyro_scale * (math.pi / 180.0)
        gz = self.read_word(self.GYRO_XOUT_H + 4) * self.gyro_scale * (math.pi / 180.0)
        return gx, gy, gz

    def read_temp(self):
        """Read temperature (°C)"""
        temp_raw = self.read_word(self.TEMP_OUT_H)
        temp_c = (temp_raw / 340.0) + 36.53
        return temp_c


class MPU6050Node(Node):
    """ROS2 node for MPU-6050 IMU"""

    def __init__(self):
        super().__init__('mpu6050_node')

        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')

        # Get parameters
        bus_number = self.get_parameter('i2c_bus').value
        address = self.get_parameter('i2c_address').value
        publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        self.get_logger().info(f'Initializing MPU-6050 on I2C bus {bus_number}, address 0x{address:02X}')

        # Initialize hardware
        try:
            self.imu = MPU6050(bus_number, address)
            self.get_logger().info('MPU-6050 initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU-6050: {e}')
            raise

        # Create publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        self.get_logger().info('MPU-6050 Node initialized')

    def publish_imu_data(self):
        """Read IMU and publish data"""
        try:
            # Read sensor data
            ax, ay, az = self.imu.read_accel()
            gx, gy, gz = self.imu.read_gyro()

            # Create and publish message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Linear acceleration
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            # Angular velocity
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            # Orientation is not available (no magnetometer)
            # Set orientation covariance to -1 to indicate unavailable
            msg.orientation_covariance[0] = -1.0

            # Set covariance values (from MPU-6050 datasheet)
            # Gyroscope: ±0.05 deg/s/sqrt(Hz) noise density
            # At 100Hz: std = 0.05 * sqrt(100) = 0.5 deg/s = 0.0087 rad/s
            gyro_var = (0.0087) ** 2
            msg.angular_velocity_covariance[0] = gyro_var
            msg.angular_velocity_covariance[4] = gyro_var
            msg.angular_velocity_covariance[8] = gyro_var

            # Accelerometer: ±400 µg/sqrt(Hz) noise density (4g range)
            # At 100Hz: std = 400e-6 * 9.81 * sqrt(100) = 0.039 m/s^2
            accel_var = (0.039) ** 2
            msg.linear_acceleration_covariance[0] = accel_var
            msg.linear_acceleration_covariance[4] = accel_var
            msg.linear_acceleration_covariance[8] = accel_var

            self.imu_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Failed to read IMU data: {e}')

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down MPU-6050 node')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MPU6050Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
