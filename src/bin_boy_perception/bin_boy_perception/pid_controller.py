#!/usr/bin/env python3
"""
PID Controller for Person Following

Provides smooth, stable tracking with minimal oscillation.
"""

import time


class PIDController:
    """
    Generic PID controller implementation

    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        output_limits: Tuple of (min, max) for output clamping
        integral_limits: Tuple of (min, max) for integral term anti-windup
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=None, integral_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Output clamping
        self.output_limits = output_limits

        # Integral anti-windup
        self.integral_limits = integral_limits

        # State variables
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def update(self, error, current_time=None):
        """
        Compute PID output

        Args:
            error: Current error value (setpoint - measurement)
            current_time: Current time (seconds). If None, uses time.time()

        Returns:
            control_output: PID controller output
        """
        # Get current time
        if current_time is None:
            current_time = time.time()

        # Calculate dt
        if self.previous_time is None:
            dt = 0.0  # First iteration
        else:
            dt = current_time - self.previous_time

        # Prevent division by zero
        if dt <= 0:
            dt = 0.001

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt

        # Anti-windup: clamp integral term
        if self.integral_limits is not None:
            self.integral = max(self.integral_limits[0],
                              min(self.integral, self.integral_limits[1]))

        i_term = self.ki * self.integral

        # Derivative term
        if self.previous_error is None:
            d_term = 0.0
        else:
            error_derivative = (error - self.previous_error) / dt
            d_term = self.kd * error_derivative

        # Compute output
        output = p_term + i_term + d_term

        # Clamp output
        if self.output_limits is not None:
            output = max(self.output_limits[0],
                        min(output, self.output_limits[1]))

        # Update state
        self.previous_error = error
        self.previous_time = current_time

        return output

    def set_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains without resetting state"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd


class PersonFollowingPID:
    """
    Dual PID controllers for person following

    Separate PIDs for:
    - Angular control (centering person in view)
    - Linear control (maintaining target distance)
    """

    def __init__(self):
        # Angular PID (centering in camera view)
        # Tuned for smooth rotation without oscillation
        self.angular_pid = PIDController(
            kp=1.2,           # Proportional: responsive turning
            ki=0.1,           # Integral: eliminate steady-state offset
            kd=0.3,           # Derivative: dampen oscillation
            output_limits=(-1.0, 1.0),      # Max angular velocity ±1.0 rad/s
            integral_limits=(-0.5, 0.5)     # Prevent integral windup
        )

        # Linear PID (distance control)
        # Tuned for smooth approach without overshoot
        self.linear_pid = PIDController(
            kp=0.4,           # Proportional: gentle approach
            ki=0.05,          # Integral: eliminate distance offset
            kd=0.2,           # Derivative: prevent overshoot
            output_limits=(-0.5, 0.5),      # Max linear velocity ±0.5 m/s
            integral_limits=(-0.3, 0.3)     # Prevent integral windup
        )

        # Dead zones (stop control when close enough)
        self.angular_deadzone = 0.05  # ~3 degrees
        self.linear_deadzone = 0.15   # 15cm

    def reset(self):
        """Reset both controllers"""
        self.angular_pid.reset()
        self.linear_pid.reset()

    def compute_velocities(self, angle_error, distance_error, current_time=None):
        """
        Compute linear and angular velocities using PID

        Args:
            angle_error: Angular error in radians (+ = person right, - = person left)
                        Note: Will be negated to match ROS convention
            distance_error: Distance error in meters (+ = too far, - = too close)
            current_time: Current timestamp (seconds)

        Returns:
            (linear_vel, angular_vel): Velocity commands
        """
        # Angular control (negate for ROS convention: +Z = turn left)
        if abs(angle_error) < self.angular_deadzone:
            angular_vel = 0.0
            # Reset integral when in deadzone to prevent windup
            self.angular_pid.integral = 0.0
        else:
            # Negate angle_error because ROS +angular.z = turn LEFT
            # If person is to RIGHT (angle > 0), we want to turn RIGHT (negative angular)
            angular_vel = -self.angular_pid.update(angle_error, current_time)

        # Linear control
        if abs(distance_error) < self.linear_deadzone:
            linear_vel = 0.0
            # Reset integral when in deadzone
            self.linear_pid.integral = 0.0
        else:
            linear_vel = self.linear_pid.update(distance_error, current_time)

        return linear_vel, angular_vel

    def set_angular_gains(self, kp=None, ki=None, kd=None):
        """Update angular PID gains"""
        self.angular_pid.set_gains(kp, ki, kd)

    def set_linear_gains(self, kp=None, ki=None, kd=None):
        """Update linear PID gains"""
        self.linear_pid.set_gains(kp, ki, kd)

    def set_deadzones(self, angular=None, linear=None):
        """Update deadzone thresholds"""
        if angular is not None:
            self.angular_deadzone = angular
        if linear is not None:
            self.linear_deadzone = linear
