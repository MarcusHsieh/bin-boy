#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import sys
import math

class TwistToEsp32Serial(Node):

    def __init__(self):
        super().__init__('twist_to_esp32_serial')


        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.07)  # meters wheel radius
        self.declare_parameter('wheel_separation', 0.165) # meters dist from center to wheel center
        self.declare_parameter('max_motor_rpm', 70.0)      # motor = 70rpm
        self.declare_parameter('min_abs_speed_percent', 5) # min speed req
        self.declare_parameter('cmd_timeout_duration', 1.0) # sec

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.min_speed_threshold = self.get_parameter('min_abs_speed_percent').get_parameter_value().integer_value
        max_motor_rpm_value = self.get_parameter('max_motor_rpm').get_parameter_value().double_value

        self.max_motor_rps = max_motor_rpm_value / 60.0
        self.max_wheel_lin_vel = self.max_motor_rps * 2.0 * math.pi * self.wheel_radius

        self.get_logger().info(f"Connecting to ESP32 on {self.serial_port_name} at {self.baud_rate} baud.")
        self.get_logger().info(f"Kiwi Params: Wheel Radius={self.wheel_radius:.3f}m, Wheel Sep={self.wheel_separation:.3f}m, Max Wheel Lin Vel={self.max_wheel_lin_vel:.3f} m/s")
        
        #serial
        self.serial_connection = None
        try:
            self.serial_connection = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1.0)
            time.sleep(2.0) # Allow ESP32 bootup
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port_name}: {e}")
            rclpy.shutdown(); sys.exit(1)
        except Exception as e:
            self.get_logger().error(f"Unexpected serial setup error: {e}")
            rclpy.shutdown(); sys.exit(1)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.last_cmd_time = self.get_clock().now()
        self.timeout_timer = self.create_timer(0.2, self.check_cmd_timeout)

        self.get_logger().info("Twist to ESP32 Serial node started.")

        # motor angles (M1 front, M2 backleft, M3 backright)
        # angles measured CCW from positive X axis (forward)
        self.motor_angles_rad = [
            math.radians(90.0),  # Motor 1 (front right) -> -vx + omega*R
            math.radians(210.0), # Motor 2 (back) -> 0.5*vx - 0.866*vy + omega*R
            math.radians(330.0)  # Motor 3 (front left) -> 0.5*vx + 0.866*vy + omega*R
        ]
        # still need to tune based on motor placement + orientation...

    def cmd_vel_callback(self, msg: Twist):
        """Receives Twist command, calculates motor speeds, sends serial commands."""
        vx = msg.linear.x
        vy = msg.linear.y 
        omega_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

        # kiwi drive
        # calc target linear speed for each wheel
        wheel_lin_vel = [0.0] * 3
        for i in range(3):
             wheel_lin_vel[i] = -vx * math.sin(self.motor_angles_rad[i]) \
                                + vy * math.cos(self.motor_angles_rad[i]) \
                                + omega_z * self.wheel_separation

        # max wheel speed magnitude calc
        max_abs_vel = 0.0
        for vel in wheel_lin_vel:
            if abs(vel) > max_abs_vel:
                max_abs_vel = abs(vel)

        if max_abs_vel > self.max_wheel_lin_vel:
            scale_factor = self.max_wheel_lin_vel / max_abs_vel
            for i in range(3):
                wheel_lin_vel[i] *= scale_factor

        for i in range(3):
            motor_num = i + 1
            speed_percent_float = (wheel_lin_vel[i] / self.max_wheel_lin_vel) * 100.0

            # deadband
            if abs(speed_percent_float) < self.min_speed_threshold:
                speed_percent = 0
                direction_char = 'S'
            elif speed_percent_float > 0:
                speed_percent = int(speed_percent_float)
                direction_char = 'F'
            else:
                speed_percent = int(abs(speed_percent_float))
                direction_char = 'R'

            if speed_percent > 100: speed_percent = 100
            if speed_percent < 0: speed_percent = 0

            command = f"M{motor_num}:{direction_char}:{speed_percent}%"
            self.send_serial_command(command)

    def check_cmd_timeout(self):
         """Checks if commands haven't been received recently and stops motors."""
         now = self.get_clock().now()
         duration_since_last_cmd = (now - self.last_cmd_time).nanoseconds / 1e9

         if duration_since_last_cmd > self.get_parameter('cmd_timeout_duration').get_parameter_value().double_value:
              if self.last_cmd_time.nanoseconds != 0:
                   self.get_logger().warn("Command timeout exceeded. Stopping motors.")
                   self.stop_all_motors()
                   self.last_cmd_time = now


    def stop_all_motors(self):
         """Sends stop command to all motors."""
         for i in range(3):
              motor_num = i + 1
              command = f"M{motor_num}:S:0%"
              self.send_serial_command(command)
              # time.sleep(0.005)

    def send_serial_command(self, command: str):
        """Sends a command string over the serial port."""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(command.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected serial write error: {e}")
        else:
            self.get_logger().warn(f"Serial connection unavailable. Cannot send: {command}")

    def read_serial_data(self):
        """Optional: Reads ESP32 feedback."""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().info(f"ESP32 >> {line}")
            except Exception:
                 pass


    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info("Stopping motors before shutdown.")
        self.stop_all_motors()
        time.sleep(0.1)
        if self.serial_connection and self.serial_connection.is_open:
            self.get_logger().info("Closing serial port.")
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TwistToEsp32Serial()
        if node.serial_connection:
             import threading
             read_thread = threading.Thread(target=lambda: \
                  [node.read_serial_data() or time.sleep(0.05) for _ in iter(int, 1)], daemon=True)
             read_thread.start()
             rclpy.spin(node)
        else:
             print("Node init failed (Serial). Shutting down.", file=sys.stderr)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received.")
    except Exception as e:
         if node: node.get_logger().fatal(f"Unhandled exception: {e}", exc_info=True)
         else: print(f"Unhandled exception before node creation: {e}", file=sys.stderr)
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()