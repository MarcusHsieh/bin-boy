#!/usr/bin/env python3
"""
ROS2 Node for Kiwi Drive Control
Subscribes to cmd_vel and publishes odometry
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time
from typing import List, Optional

from bin_boy_control.sts3215_driver import KiwiDriveController


class KiwiDriveNode(Node):
    def __init__(self):
        super().__init__('kiwi_drive_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('wheel_ids', [1, 2, 3])  # [front, left_rear, right_rear]
        self.declare_parameter('wheel_radius', 0.05)  # meters
        self.declare_parameter('robot_radius', 0.15)  # meters (center to wheel)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('encoder_resolution', 4096)  # STS3215: 12-bit encoder

        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        wheel_ids = self.get_parameter('wheel_ids').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value

        self.get_logger().info(f'Connecting to servos on {serial_port} at {baudrate} baud')

        # Initialize hardware driver
        try:
            self.controller = KiwiDriveController(wheel_ids, baudrate, serial_port)
            self.get_logger().info('Successfully connected to servos')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to servos: {e}')
            raise

        # Initialize odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_encoder_positions = [None, None, None]

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer for odometry updates
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info('Kiwi Drive Node initialized')

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        self.get_logger().info(f'[DEBUG cmd_vel_callback] Received cmd_vel: vx={vx}, vy={vy}, omega={omega}')

        # Send to hardware
        try:
            self.get_logger().info(f'[DEBUG cmd_vel_callback] Calling set_velocity with 3 params: vx={vx}, vy={vy}, omega={omega}')
            self.controller.set_velocity(vx, vy, omega)
        except Exception as e:
            self.get_logger().error(f'Failed to set velocity: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def update_odometry(self):
        """Read encoders and update odometry"""
        # try:
            # Read current encoder positions
        positions = []
        speeds = []
        for i in range(1, 4):
            position, speed = self.controller.read_wheel_status(i)
            positions.append(position)
            speeds.append(speed)


        if None in positions:
            self.get_logger().warn('Failed to read some encoder positions')
            return

        current_time = self.get_clock().now()

        # Initialize on first read
        if None in self.last_encoder_positions:
            self.last_encoder_positions = positions
            self.last_time = current_time
            return

        # Calculate time delta
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Calculate encoder deltas (handle wraparound)
        deltas = []
        for i in range(3):
            delta = positions[i] - self.last_encoder_positions[i]

            # Handle wraparound
            if delta > self.encoder_resolution / 2:
                delta -= self.encoder_resolution
            elif delta < -self.encoder_resolution / 2:
                delta += self.encoder_resolution

            deltas.append(delta)

        # Convert encoder ticks to radians
        wheel_angular_displacements = [
            (delta / self.encoder_resolution) * 2 * math.pi
            for delta in deltas
        ]

        # Convert to linear wheel displacements
        wheel_linear_displacements = [
            displacement * self.wheel_radius
            for displacement in wheel_angular_displacements
        ]

        # Kiwi drive forward kinematics
        # Convert wheel displacements to robot displacement
        # Inverse of: wheel = (-vx*sin(theta) + vy*cos(theta) + omega*R) / r

        d_front = wheel_linear_displacements[0]
        d_left = wheel_linear_displacements[1]
        d_right = wheel_linear_displacements[2]

        # Solve for vx, vy, omega
        # Using least squares solution for overdetermined system
        # This is a simplified version - you may want to tune this

        d_x = -d_front / 2 + d_left / 2 + d_right / 2
        d_y = d_front - d_left / 2 - d_right / 2
        d_theta = (d_front + d_left + d_right) / (3 * self.robot_radius)

        # Update pose (dead reckoning)
        delta_x = d_x * math.cos(self.theta) - d_y * math.sin(self.theta)
        delta_y = d_x * math.sin(self.theta) + d_y * math.cos(self.theta)

        self.x += delta_x
        self.y += delta_y
        self.theta += d_theta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate velocities
        vx = d_x / dt
        vy = d_y / dt
        omega = d_theta / dt

        # Publish odometry
        self.publish_odometry(current_time, vx, vy, omega)

        # Publish joint states
        self.publish_joint_states(current_time, positions, wheel_angular_displacements)

        # Update state
        self.last_encoder_positions = positions
        self.last_time = current_time

        # except Exception as e:
            # self.get_logger().error('Odometry update failed %s', e)

    def publish_odometry(self, current_time, vx: float, vy: float, omega: float):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        # Publish
        self.odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self, current_time, positions: List[int], velocities: List[float]):
        """Publish joint states for visualization"""
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['front_wheel_joint', 'left_rear_wheel_joint', 'right_rear_wheel_joint']

        # Convert positions to radians
        joint_state.position = [
            (pos / self.encoder_resolution) * 2 * math.pi
            for pos in positions
        ]

        # Velocities already in rad/s
        joint_state.velocity = velocities

        self.joint_state_pub.publish(joint_state)

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down kiwi drive node')
        try:
            self.controller.stop()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KiwiDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
