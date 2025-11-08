#!/usr/bin/env python3
"""
Simple Keyboard Controller for Kiwi Drive Robot
No installation required - works immediately!

Controls:
  w/s - Forward/Backward
  a/d - Strafe Left/Right
  q/e - Rotate Left/Right
  x   - Stop
  SPACE - Emergency stop
  +/- - Increase/Decrease speed
  CTRL+C - Quit
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleKeyboardController(Node):
    def __init__(self):
        super().__init__('simple_keyboard_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Speed settings
        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.3  # rad/s
        self.speed_increment = 0.05

        self.get_logger().info('Simple Keyboard Controller Started!')
        self.get_logger().info('='*60)
        self.print_instructions()

    def print_instructions(self):
        print('\n' + '='*60)
        print('KIWI DRIVE KEYBOARD CONTROL')
        print('='*60)
        print('Movement Controls:')
        print('  w - Forward')
        print('  s - Backward')
        print('  a - Strafe Left')
        print('  d - Strafe Right')
        print('  q - Rotate Counter-Clockwise')
        print('  e - Rotate Clockwise')
        print()
        print('Combined (Hold multiple keys):')
        print('  w+a - Forward-Left diagonal')
        print('  w+d - Forward-Right diagonal')
        print('  w+q - Forward while rotating left')
        print('  etc...')
        print()
        print('Speed Control:')
        print('  + - Increase speed')
        print('  - - Decrease speed')
        print()
        print('Safety:')
        print('  x or SPACE - Stop')
        print('  CTRL+C - Quit')
        print('='*60)
        print(f'Current Speed: Linear={self.linear_speed:.2f} m/s, Angular={self.angular_speed:.2f} rad/s')
        print('='*60)
        print('\nReady! Press keys to move...\n')

    def get_key(self):
        """Get a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def publish_velocity(self, vx, vy, omega):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        self.publisher.publish(msg)

    def stop(self):
        """Send stop command"""
        self.publish_velocity(0.0, 0.0, 0.0)
        self.get_logger().info('STOP')

    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()

                vx = 0.0
                vy = 0.0
                omega = 0.0

                # Movement keys
                if key == 'w':
                    vx = self.linear_speed
                    self.get_logger().info(f'Forward: {vx:.2f} m/s')
                elif key == 's':
                    vx = -self.linear_speed
                    self.get_logger().info(f'Backward: {vx:.2f} m/s')
                elif key == 'a':
                    vy = self.linear_speed
                    self.get_logger().info(f'Strafe Left: {vy:.2f} m/s')
                elif key == 'd':
                    vy = -self.linear_speed
                    self.get_logger().info(f'Strafe Right: {vy:.2f} m/s')
                elif key == 'q':
                    omega = self.angular_speed
                    self.get_logger().info(f'Rotate CCW: {omega:.2f} rad/s')
                elif key == 'e':
                    omega = -self.angular_speed
                    self.get_logger().info(f'Rotate CW: {omega:.2f} rad/s')

                # Stop keys
                elif key == 'x' or key == ' ':
                    self.stop()
                    continue

                # Speed control
                elif key == '+' or key == '=':
                    self.linear_speed += self.speed_increment
                    self.angular_speed += self.speed_increment
                    self.get_logger().info(f'Speed increased: Linear={self.linear_speed:.2f}, Angular={self.angular_speed:.2f}')
                    continue
                elif key == '-' or key == '_':
                    self.linear_speed = max(0.05, self.linear_speed - self.speed_increment)
                    self.angular_speed = max(0.05, self.angular_speed - self.speed_increment)
                    self.get_logger().info(f'Speed decreased: Linear={self.linear_speed:.2f}, Angular={self.angular_speed:.2f}')
                    continue

                # Help
                elif key == 'h':
                    self.print_instructions()
                    continue

                # Quit
                elif key == '\x03':  # CTRL+C
                    self.get_logger().info('Shutting down...')
                    self.stop()
                    break

                else:
                    continue

                # Publish the velocity
                self.publish_velocity(vx, vy, omega)

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
        finally:
            self.stop()

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleKeyboardController()

    try:
        controller.run()
    except Exception as e:
        controller.get_logger().error(f'Error: {e}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
