#!/usr/bin/env python3
"""
Hardware verification script for MPU6050 IMU
Tests I2C communication and sensor data reading
"""

import sys
import time

try:
    import smbus2
except ImportError:
    print("ERROR: smbus2 not installed. Run: pip install smbus2")
    sys.exit(1)

# MPU6050 Constants
MPU6050_ADDR_0x68 = 0x68
MPU6050_ADDR_0x69 = 0x69
WHO_AM_I_REG = 0x75
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41

def scan_i2c_bus(bus_num):
    """Scan I2C bus for devices"""
    print(f"\n{'='*60}")
    print(f"Scanning I2C bus {bus_num}...")
    print(f"{'='*60}")

    try:
        bus = smbus2.SMBus(bus_num)
        devices = []

        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                devices.append(addr)
                print(f"  Found device at address: 0x{addr:02X}")
            except:
                pass

        bus.close()
        return devices
    except Exception as e:
        print(f"  ERROR: Cannot access I2C bus {bus_num}: {e}")
        return []

def verify_mpu6050(bus_num, address):
    """Verify MPU6050 connection and read WHO_AM_I register"""
    print(f"\n{'='*60}")
    print(f"Verifying MPU6050 at address 0x{address:02X} on bus {bus_num}...")
    print(f"{'='*60}")

    try:
        bus = smbus2.SMBus(bus_num)

        # Read WHO_AM_I register
        who_am_i = bus.read_byte_data(address, WHO_AM_I_REG)
        print(f"  WHO_AM_I register: 0x{who_am_i:02X}")

        if who_am_i == 0x68:
            print("  ✓ MPU6050 identified successfully!")
        else:
            print(f"  ✗ Unexpected WHO_AM_I value (expected 0x68, got 0x{who_am_i:02X})")
            return False

        # Wake up the sensor
        bus.write_byte_data(address, PWR_MGMT_1, 0)
        time.sleep(0.1)

        # Read power management register to confirm
        pwr_mgmt = bus.read_byte_data(address, PWR_MGMT_1)
        print(f"  Power Management: 0x{pwr_mgmt:02X} (sensor awake: {pwr_mgmt == 0})")

        bus.close()
        return True

    except Exception as e:
        print(f"  ✗ Error communicating with MPU6050: {e}")
        return False

def read_sensor_data(bus_num, address):
    """Read and display raw sensor data"""
    print(f"\n{'='*60}")
    print(f"Reading sensor data from MPU6050...")
    print(f"{'='*60}")

    try:
        bus = smbus2.SMBus(bus_num)

        # Wake up sensor
        bus.write_byte_data(address, PWR_MGMT_1, 0)
        time.sleep(0.1)

        print("\nReading 5 samples at 1 Hz...\n")

        for i in range(5):
            # Read accelerometer (6 bytes starting at ACCEL_XOUT_H)
            accel_data = bus.read_i2c_block_data(address, ACCEL_XOUT_H, 6)
            ax = (accel_data[0] << 8) | accel_data[1]
            ay = (accel_data[2] << 8) | accel_data[3]
            az = (accel_data[4] << 8) | accel_data[5]

            # Convert to signed
            if ax >= 0x8000: ax -= 0x10000
            if ay >= 0x8000: ay -= 0x10000
            if az >= 0x8000: az -= 0x10000

            # Read gyroscope (6 bytes starting at GYRO_XOUT_H)
            gyro_data = bus.read_i2c_block_data(address, GYRO_XOUT_H, 6)
            gx = (gyro_data[0] << 8) | gyro_data[1]
            gy = (gyro_data[2] << 8) | gyro_data[3]
            gz = (gyro_data[4] << 8) | gyro_data[5]

            # Convert to signed
            if gx >= 0x8000: gx -= 0x10000
            if gy >= 0x8000: gy -= 0x10000
            if gz >= 0x8000: gz -= 0x10000

            # Read temperature
            temp_data = bus.read_i2c_block_data(address, TEMP_OUT_H, 2)
            temp_raw = (temp_data[0] << 8) | temp_data[1]
            if temp_raw >= 0x8000: temp_raw -= 0x10000
            temp_c = (temp_raw / 340.0) + 36.53

            print(f"Sample {i+1}:")
            print(f"  Accel (raw):  X={ax:6d}  Y={ay:6d}  Z={az:6d}")
            print(f"  Gyro (raw):   X={gx:6d}  Y={gy:6d}  Z={gz:6d}")
            print(f"  Temperature:  {temp_c:.2f}°C")
            print()

            time.sleep(1.0)

        bus.close()
        print("✓ Sensor data read successfully!")
        return True

    except Exception as e:
        print(f"✗ Error reading sensor data: {e}")
        return False

def main():
    print("\n" + "="*60)
    print(" MPU6050 Hardware Verification Tool")
    print("="*60)

    # Scan I2C bus 1
    devices = scan_i2c_bus(1)

    if not devices:
        print("\n✗ No I2C devices found on bus 1!")
        print("\nTroubleshooting:")
        print("  1. Check wiring:")
        print("     MPU6050 VCC  → Jetson Pin 1 (3.3V)")
        print("     MPU6050 GND  → Jetson Pin 6 (GND)")
        print("     MPU6050 SCL  → Jetson Pin 5 (GPIO3)")
        print("     MPU6050 SDA  → Jetson Pin 3 (GPIO2)")
        print("     MPU6050 AD0  → 3.3V (for 0x69) or GND (for 0x68)")
        print("  2. Verify I2C is enabled: ls -l /dev/i2c-*")
        print("  3. Check permissions: sudo usermod -a -G i2c $USER")
        sys.exit(1)

    # Check for MPU6050 at 0x69 (requested address)
    if MPU6050_ADDR_0x69 in devices:
        print(f"\n✓ Found MPU6050 at expected address 0x69!")
        address = MPU6050_ADDR_0x69
    elif MPU6050_ADDR_0x68 in devices:
        print(f"\n⚠ Found MPU6050 at 0x68 (not 0x69)")
        print("  This means AD0 pin is LOW. Connect AD0 to 3.3V for 0x69.")
        address = MPU6050_ADDR_0x68
    else:
        print(f"\n✗ No MPU6050 found at 0x68 or 0x69")
        print(f"  Found devices at: {[f'0x{addr:02X}' for addr in devices]}")
        sys.exit(1)

    # Verify MPU6050
    if not verify_mpu6050(1, address):
        sys.exit(1)

    # Read sensor data
    if not read_sensor_data(1, address):
        sys.exit(1)

    # Final summary
    print("\n" + "="*60)
    print(" ✓ VERIFICATION COMPLETE - MPU6050 IS WORKING!")
    print("="*60)
    print(f"\nYour MPU6050 is at address 0x{address:02X} on I2C bus 1")
    print("\nNext steps:")
    print("  1. Test with ROS2 node:")
    print(f"     ros2 run bin_boy_control mpu6050_node --ros-args -p i2c_address:={address}")
    print("  2. Check published data:")
    print("     ros2 topic echo /imu/data_raw")
    print("  3. Launch full control system:")
    print("     ros2 launch bin_boy_control full_control.launch.py")
    print()

if __name__ == '__main__':
    main()
