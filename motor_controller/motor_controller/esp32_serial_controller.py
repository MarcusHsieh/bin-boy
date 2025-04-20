#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import sys

class Esp32SerialController(Node):

    def __init__(self):
        super().__init__('esp32_serial_controller')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.get_logger().info(f"Attempting to connect to ESP32 on {self.serial_port_name} at {self.baud_rate} baud.")

        self.serial_connection = None
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            time.sleep(2.0)
            self.get_logger().info("Successfully connected to serial port.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port_name}: {e}")
            self.get_logger().error("Node will shut down. Check port name/permissions and ESP32 connection.")
            rclpy.shutdown()
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during serial setup: {e}")
            rclpy.shutdown()
            sys.exit(1)

        #subscriber
        self.subscription = self.create_subscription(
            String,
            '/motor_commands',
            self.command_callback,
            10)
        self.get_logger().info(f"Subscribed to /motor_commands topic.")

        self.serial_read_timer = self.create_timer(0.1, self.read_serial_data)


    def command_callback(self, msg):
        """Receives a command string and sends it to the ESP32."""
        command = msg.data
        self.get_logger().info(f"Received command: '{command}'")

        if not command.startswith('M') or not command.endswith('%') or command.count(':') != 2:
             self.get_logger().warn(f"Command '{command}' format may be invalid. Sending anyway.")

        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(command.encode('utf-8'))
                self.get_logger().info(f"Sent: '{command}'")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred during serial write: {e}")
        else:
            self.get_logger().warn("Serial connection is not available. Cannot send command.")

    def read_serial_data(self):
        """Periodically reads any data sent back from the ESP32 (for debugging)."""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().info(f"ESP32 >> {line}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
            except UnicodeDecodeError:
                self.get_logger().warn("Received non-UTF8 data from ESP32.")
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred during serial read: {e}")


    def destroy_node(self):
        """Clean up resources."""
        if self.serial_connection and self.serial_connection.is_open:
            self.get_logger().info("Closing serial port.")
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Esp32SerialController()
        if node.serial_connection:
             rclpy.spin(node)
        else:
             print("Node initialization failed (Serial connection). Shutting down.", file=sys.stderr)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down.")
    except Exception as e:
         if node:
             node.get_logger().fatal(f"Unhandled exception in main spin: {e}", exc_info=True)
         else:
             print(f"Unhandled exception before node creation: {e}", file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()


if __name__ == '__main__':
    main()