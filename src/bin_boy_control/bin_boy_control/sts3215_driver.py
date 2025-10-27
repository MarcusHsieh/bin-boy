#!/usr/bin/env python3
"""
STS3215 Serial Bus Servo Driver
Based on Feetech STS3215 protocol documentation
"""

import serial
import time
import struct
from typing import Optional, List, Tuple

class STS3215:
    """Driver for Feetech STS3215 30kg serial bus servo with magnetic encoder"""

    # Memory table addresses (from STS3215 datasheet)
    ADDR_MODEL_L = 3
    ADDR_MODEL_H = 4
    ADDR_ID = 5
    ADDR_BAUD_RATE = 6
    ADDR_RETURN_DELAY = 7
    ADDR_RESPONSE_STATUS = 8
    ADDR_MIN_ANGLE_L = 9
    ADDR_MIN_ANGLE_H = 10
    ADDR_MAX_ANGLE_L = 11
    ADDR_MAX_ANGLE_H = 12
    ADDR_MAX_TEMP = 13
    ADDR_MAX_VOLTAGE = 14
    ADDR_MIN_VOLTAGE = 15
    ADDR_MAX_TORQUE_L = 16
    ADDR_MAX_TORQUE_H = 17
    ADDR_UNLOAD_COND = 18
    ADDR_LED_ALARM = 19
    ADDR_P_COEF = 21
    ADDR_D_COEF = 22
    ADDR_I_COEF = 23
    ADDR_MIN_PWM_L = 24
    ADDR_MIN_PWM_H = 25
    ADDR_CLOCKWISE_INSENS = 26
    ADDR_ANTICLOCKWISE_INSENS = 27
    ADDR_PROTECTION_CURRENT_L = 28
    ADDR_PROTECTION_CURRENT_H = 29
    ADDR_PROTECTION_TIME = 30
    ADDR_OVERLOAD_TORQUE = 31
    ADDR_MODE = 33
    ADDR_PROTECT_VOLT = 34
    ADDR_PROTECT_TIME = 35
    ADDR_OVERLOAD_TIME = 36
    ADDR_MODE_EXEC = 37
    ADDR_TORQUE_ENABLE = 40
    ADDR_ACC = 41
    ADDR_GOAL_POSITION_L = 42
    ADDR_GOAL_POSITION_H = 43
    ADDR_GOAL_TIME_L = 44
    ADDR_GOAL_TIME_H = 45
    ADDR_GOAL_SPEED_L = 46
    ADDR_GOAL_SPEED_H = 47
    ADDR_TORQUE_LIMIT_L = 48
    ADDR_TORQUE_LIMIT_H = 49
    ADDR_LOCK = 55
    ADDR_PRESENT_POSITION_L = 56
    ADDR_PRESENT_POSITION_H = 57
    ADDR_PRESENT_SPEED_L = 58
    ADDR_PRESENT_SPEED_H = 59
    ADDR_PRESENT_LOAD_L = 60
    ADDR_PRESENT_LOAD_H = 61
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMP = 63
    ADDR_ASYNC_WRITE_FLAG = 64
    ADDR_SERVO_STATUS = 65
    ADDR_MOVING = 66
    ADDR_PRESENT_CURRENT_L = 69
    ADDR_PRESENT_CURRENT_H = 70

    # Instructions
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_SYNC_WRITE = 0x83
    INST_SYNC_READ = 0x82

    # Modes
    MODE_SERVO = 0  # Position control mode
    MODE_MOTOR = 1  # Speed control mode (continuous rotation)
    MODE_STEP = 3   # Step mode

    BROADCAST_ID = 0xFE

    def __init__(self, port: str, baudrate: int = 1000000, timeout: float = 0.01):
        """
        Initialize STS3215 driver

        Args:
            port: Serial port (e.g., '/dev/ttyUSB0')
            baudrate: Baud rate (default 1000000 for STS3215)
            timeout: Serial timeout in seconds
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        self.ser.flush()

    def __del__(self):
        """Close serial port on destruction"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

    def _checksum(self, packet: bytes) -> int:
        """Calculate checksum for packet"""
        return (~sum(packet[2:-1])) & 0xFF

    def _send_packet(self, servo_id: int, instruction: int, params: bytes = b'') -> None:
        """Send a packet to the servo"""
        length = len(params) + 2  # length includes instruction and checksum
        packet = bytes([0xFF, 0xFF, servo_id, length, instruction]) + params
        checksum = self._checksum(packet)
        packet += bytes([checksum])

        self.ser.write(packet)
        self.ser.flush()

    def _read_response(self) -> Optional[Tuple[int, bytes]]:
        """Read response packet from servo"""
        # Wait for header
        header = self.ser.read(2)
        if header != b'\xFF\xFF':
            return None

        # Read ID and length
        id_byte = self.ser.read(1)
        if not id_byte:
            return None
        servo_id = id_byte[0]

        length_byte = self.ser.read(1)
        if not length_byte:
            return None
        length = length_byte[0]

        # Read error + params + checksum
        remaining = self.ser.read(length)
        if len(remaining) != length:
            return None

        error = remaining[0]
        params = remaining[1:-1]
        checksum = remaining[-1]

        # Verify checksum
        packet = bytes([0xFF, 0xFF, servo_id, length]) + remaining[:-1]
        if checksum != self._checksum(packet):
            return None

        return (servo_id, params)

    def ping(self, servo_id: int) -> bool:
        """Ping a servo to check if it's connected"""
        self._send_packet(servo_id, self.INST_PING)
        response = self._read_response()
        return response is not None

    def read_position(self, servo_id: int) -> Optional[int]:
        """
        Read current position from servo

        Returns:
            Position in raw encoder units (0-4095), or None on error
        """
        self._send_packet(servo_id, self.INST_READ, bytes([self.ADDR_PRESENT_POSITION_L, 2]))
        response = self._read_response()

        if response is None:
            return None

        _, data = response
        if len(data) < 2:
            return None

        position = struct.unpack('<H', data[:2])[0]
        return position

    def read_speed(self, servo_id: int) -> Optional[int]:
        """
        Read current speed from servo

        Returns:
            Speed in raw units (-1023 to 1023), or None on error
            Positive = CCW, Negative = CW
        """
        self._send_packet(servo_id, self.INST_READ, bytes([self.ADDR_PRESENT_SPEED_L, 2]))
        response = self._read_response()

        if response is None:
            return None

        _, data = response
        if len(data) < 2:
            return None

        speed = struct.unpack('<h', data[:2])[0]  # signed short
        return speed

    def set_mode(self, servo_id: int, mode: int) -> bool:
        """
        Set servo operating mode

        Args:
            servo_id: Servo ID
            mode: MODE_SERVO (position), MODE_MOTOR (speed), or MODE_STEP

        Returns:
            True on success
        """
        self._send_packet(servo_id, self.INST_WRITE, bytes([self.ADDR_MODE, mode]))
        time.sleep(0.01)
        return True

    def set_speed_mode(self, servo_id: int, speed: int) -> bool:
        """
        Set speed in motor mode (continuous rotation)

        Args:
            servo_id: Servo ID
            speed: Speed (-1023 to 1023). Positive = CCW, Negative = CW

        Returns:
            True on success
        """
        # Ensure in motor mode
        self.set_mode(servo_id, self.MODE_MOTOR)

        # Clamp speed
        speed = max(-1023, min(1023, speed))

        # Pack as signed short
        speed_bytes = struct.pack('<h', speed)

        self._send_packet(
            servo_id,
            self.INST_WRITE,
            bytes([self.ADDR_GOAL_SPEED_L]) + speed_bytes
        )

        return True

    def torque_enable(self, servo_id: int, enable: bool = True) -> bool:
        """Enable or disable servo torque"""
        value = 1 if enable else 0
        self._send_packet(servo_id, self.INST_WRITE, bytes([self.ADDR_TORQUE_ENABLE, value]))
        return True

    def sync_write_speed(self, servo_data: List[Tuple[int, int]]) -> bool:
        """
        Synchronously write speed to multiple servos

        Args:
            servo_data: List of (servo_id, speed) tuples

        Returns:
            True on success
        """
        if not servo_data:
            return False

        # Build sync write packet
        # Format: [ADDR, LENGTH, [ID1, DATA1, ID2, DATA2, ...]]
        params = bytes([self.ADDR_GOAL_SPEED_L, 2])  # 2 bytes per servo

        for servo_id, speed in servo_data:
            speed = max(-1023, min(1023, speed))
            speed_bytes = struct.pack('<h', speed)
            params += bytes([servo_id]) + speed_bytes

        self._send_packet(self.BROADCAST_ID, self.INST_SYNC_WRITE, params)
        return True


class KiwiDriveController:
    """
    Kiwi drive (3-wheel omnidirectional) controller using STS3215 servos

    Wheel layout:
        Wheel 0: Front (0°)
        Wheel 1: Rear Left (120°)
        Wheel 2: Rear Right (240°)
    """

    def __init__(self, driver: STS3215, wheel_ids: List[int] = [1, 2, 3]):
        """
        Initialize kiwi drive controller

        Args:
            driver: STS3215 driver instance
            wheel_ids: List of servo IDs [front, left_rear, right_rear]
        """
        self.driver = driver
        self.wheel_ids = wheel_ids

        # Initialize all servos to motor mode
        for servo_id in self.wheel_ids:
            self.driver.set_mode(servo_id, STS3215.MODE_MOTOR)
            self.driver.torque_enable(servo_id, True)

    def set_velocity(self, vx: float, vy: float, omega: float, max_speed: int = 1000) -> None:
        """
        Set robot velocity using kiwi drive inverse kinematics

        Args:
            vx: Forward velocity (m/s)
            vy: Strafe velocity (m/s) - positive is left
            omega: Angular velocity (rad/s) - positive is CCW
            max_speed: Maximum servo speed units
        """
        import math

        # Wheel radius and robot radius - ACTUAL ROBOT MEASUREMENTS
        WHEEL_RADIUS = 0.0725  # meters (145mm diameter wheels)
        ROBOT_RADIUS = 0.1815  # meters (wheels outside body: 157.5mm + 19mm + 5mm)

        # Kiwi drive inverse kinematics
        # Wheel 0 (front, 0°): at angle 0°
        # Wheel 1 (left rear, 120°): at angle 2π/3
        # Wheel 2 (right rear, 240°): at angle 4π/3

        # Convert to wheel velocities (rad/s)
        # Formula: wheel_speed = (1/r) * (-vx*sin(theta) + vy*cos(theta) + omega*R)

        wheel_0 = (-vx * 0 + vy * 1 + omega * ROBOT_RADIUS) / WHEEL_RADIUS
        wheel_1 = (-vx * math.sin(2*math.pi/3) + vy * math.cos(2*math.pi/3) + omega * ROBOT_RADIUS) / WHEEL_RADIUS
        wheel_2 = (-vx * math.sin(4*math.pi/3) + vy * math.cos(4*math.pi/3) + omega * ROBOT_RADIUS) / WHEEL_RADIUS

        # Convert from rad/s to servo speed units
        # STS3215: 1 speed unit ≈ 0.732 RPM (approximate)
        # TODO: Calibrate this conversion factor
        SPEED_FACTOR = 50  # Tuning parameter

        speeds = [
            int(wheel_0 * SPEED_FACTOR),
            int(wheel_1 * SPEED_FACTOR),
            int(wheel_2 * SPEED_FACTOR)
        ]

        # Clamp to max speed
        speeds = [max(-max_speed, min(max_speed, s)) for s in speeds]

        # Send to servos
        servo_data = list(zip(self.wheel_ids, speeds))
        self.driver.sync_write_speed(servo_data)

    def stop(self) -> None:
        """Stop all wheels"""
        servo_data = [(servo_id, 0) for servo_id in self.wheel_ids]
        self.driver.sync_write_speed(servo_data)

    def read_wheel_positions(self) -> List[Optional[int]]:
        """Read current position of all wheels for odometry"""
        positions = []
        for servo_id in self.wheel_ids:
            pos = self.driver.read_position(servo_id)
            positions.append(pos)
        return positions

    def read_wheel_speeds(self) -> List[Optional[int]]:
        """Read current speed of all wheels"""
        speeds = []
        for servo_id in self.wheel_ids:
            speed = self.driver.read_speed(servo_id)
            speeds.append(speed)
        return speeds


if __name__ == '__main__':
    # Test code
    print("STS3215 Driver Test")
    print("Attempting to connect to servos...")

    try:
        driver = STS3215('/dev/ttyUSB0', baudrate=1000000)

        # Ping servos 1, 2, 3
        for i in [1, 2, 3]:
            if driver.ping(i):
                print(f"Servo {i}: Connected")
            else:
                print(f"Servo {i}: Not found")

        # Test kiwi drive
        print("\nInitializing kiwi drive controller...")
        kiwi = KiwiDriveController(driver, wheel_ids=[1, 2, 3])

        print("Testing forward motion for 2 seconds...")
        kiwi.set_velocity(0.2, 0, 0)
        time.sleep(2)

        print("Stopping...")
        kiwi.stop()

    except Exception as e:
        print(f"Error: {e}")
