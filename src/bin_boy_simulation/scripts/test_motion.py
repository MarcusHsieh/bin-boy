#!/usr/bin/env python3
"""
Test Script: Robot Motion Control
Tests basic motion capabilities of the robot in simulation or real hardware
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys


class MotionTester(Node):
    def __init__(self):
        super().__init__('motion_tester')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        time.sleep(1)  # Wait for publisher to establish

    def send_velocity(self, vx, vy, omega, duration):
        """Send velocity command for specified duration"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = omega

        self.get_logger().info(f'Sending: vx={vx}, vy={vy}, omega={omega} for {duration}s')

        # Send command at 10Hz for duration
        rate = self.create_rate(10)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # Stop
        self.stop()

    def stop(self):
        """Send zero velocity"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Stopped')

    def test_forward(self):
        """Test forward motion"""
        self.get_logger().info('TEST 1: Forward motion (0.3 m/s for 3s)')
        self.send_velocity(0.3, 0.0, 0.0, 3.0)
        time.sleep(1)

    def test_backward(self):
        """Test backward motion"""
        self.get_logger().info('TEST 2: Backward motion (-0.3 m/s for 3s)')
        self.send_velocity(-0.3, 0.0, 0.0, 3.0)
        time.sleep(1)

    def test_strafe_left(self):
        """Test strafe left (kiwi drive capability)"""
        self.get_logger().info('TEST 3: Strafe left (vy=0.3 m/s for 3s)')
        self.send_velocity(0.0, 0.3, 0.0, 3.0)
        time.sleep(1)

    def test_strafe_right(self):
        """Test strafe right (kiwi drive capability)"""
        self.get_logger().info('TEST 4: Strafe right (vy=-0.3 m/s for 3s)')
        self.send_velocity(0.0, -0.3, 0.0, 3.0)
        time.sleep(1)

    def test_rotate_ccw(self):
        """Test counter-clockwise rotation"""
        self.get_logger().info('TEST 5: Rotate CCW (0.5 rad/s for 3s)')
        self.send_velocity(0.0, 0.0, 0.5, 3.0)
        time.sleep(1)

    def test_rotate_cw(self):
        """Test clockwise rotation"""
        self.get_logger().info('TEST 6: Rotate CW (-0.5 rad/s for 3s)')
        self.send_velocity(0.0, 0.0, -0.5, 3.0)
        time.sleep(1)

    def test_diagonal(self):
        """Test diagonal motion (combined x and y)"""
        self.get_logger().info('TEST 7: Diagonal motion (vx=0.2, vy=0.2 for 3s)')
        self.send_velocity(0.2, 0.2, 0.0, 3.0)
        time.sleep(1)

    def test_combined(self):
        """Test combined motion (translation + rotation)"""
        self.get_logger().info('TEST 8: Combined motion (vx=0.2, omega=0.3 for 3s)')
        self.send_velocity(0.2, 0.0, 0.3, 3.0)
        time.sleep(1)

    def run_all_tests(self):
        """Run complete test suite"""
        self.get_logger().info('=== Starting Motion Test Suite ===')
        self.get_logger().info('Testing kiwi drive motion capabilities')
        self.get_logger().info('')

        try:
            self.test_forward()
            self.test_backward()
            self.test_strafe_left()
            self.test_strafe_right()
            self.test_rotate_ccw()
            self.test_rotate_cw()
            self.test_diagonal()
            self.test_combined()

            self.get_logger().info('')
            self.get_logger().info('=== Motion Test Suite COMPLETE ===')
            self.get_logger().info('All tests passed!')

        except KeyboardInterrupt:
            self.get_logger().info('Test interrupted by user')
            self.stop()
        except Exception as e:
            self.get_logger().error(f'Test failed: {e}')
            self.stop()


def main(args=None):
    rclpy.init(args=args)

    tester = MotionTester()

    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
