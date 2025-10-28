# Autonomous Trash Can - Complete Setup Guide

## Project Overview

An autonomous trash can that follows the user around using:
- **Kiwi Drive** (3-wheel omnidirectional platform)
- **2D LIDAR** for mapping and obstacle avoidance
- **Camera** with YOLOv5 for person detection/tracking
- **IMU** for orientation
- **Encoder motors** for precise odometry

---

## Hardware Components

### Motors & Control
- **3x Feetech STS3215** - 30kg-cm serial bus servos with 360° magnetic encoders
- **2x Serial Bus Servo Driver Boards** - Control up to 253 servos each
- **Protocol:** TTL UART half-duplex serial bus at 1Mbaud

### Sensors
- **LD14P LIDAR** - 360° 2D laser scanner (0.15-8m range, 6Hz scan rate)
- **CSI Camera** - Wide angle camera running YOLOv5 for person detection
- **OV9281 Camera** - 120fps global shutter mono camera (optional)
- **MPU-6050** - 6-axis IMU (gyro + accelerometer)

### Compute
- **NVIDIA Jetson Nano 4GB** - Main compute platform

### Power & Connectivity
- USB 3.2 hub, JST connectors, DC power cables

---

## 1. URDF Robot Description

### What We Created

**Package:** `bin_boy_description`

**Structure:**
```
bin_boy_description/
├── urdf/
│   └── bin_boy.urdf.xacro      # Robot model with kiwi drive
├── launch/
│   └── display.launch.py       # Launch robot visualization
└── rviz/
    └── view_robot.rviz         # RViz configuration
```

**Robot Specifications (in URDF):**
- Base: 36cm diameter cylinder, 60cm tall
- 3 wheels in kiwi configuration (0°, 120°, 240°)
- Wheel radius: 5cm
- LIDAR mounted at 50cm height
- Camera mounted at 55cm height, slight forward tilt
- IMU at base center

### Viewing Your Robot

```bash
cd ~/bin-boy
source install/setup.bash
ros2 launch bin_boy_description display.launch.py
```

This will open RViz2 showing your robot model with all sensors and transforms.

---

## 2. Motor Control System

### What We Created

**Package:** `bin_boy_control`

**Key Files:**
- `sts3215_driver.py` - Low-level STS3215 servo communication
- `kiwi_drive_node.py` - ROS2 node for kiwi drive control + odometry
- `kiwi_drive.launch.py` - Launch file
- `kiwi_params.yaml` - Configuration file

### STS3215 Servo Details

**Communication Protocol:**
- Half-duplex TTL UART serial bus
- Default baudrate: 1,000,000 baud
- Packet-based protocol with checksums
- Each servo has unique ID (1-253)

**Operating Modes:**
1. **Position Mode (MODE_SERVO)** - Standard servo (0-4095 positions)
2. **Speed Mode (MODE_MOTOR)** - Continuous rotation (what we use!)
3. **Step Mode (MODE_STEP)** - Stepper motor emulation

**Encoder:**
- 12-bit magnetic encoder (4096 positions per revolution)
- Built-in for odometry
- Absolute position feedback

### Kiwi Drive Kinematics

**Wheel Layout:**
```
        Front (0°)
           W0
          /  \
         /    \
        /  +z  \
   W1  -------- W2
   (120°)    (240°)
```

**Inverse Kinematics:**
```
For velocity command (vx, vy, omega):

wheel_0 = (vy + omega*R) / r
wheel_1 = (-vx*sin(120°) + vy*cos(120°) + omega*R) / r
wheel_2 = (-vx*sin(240°) + vy*cos(240°) + omega*R) / r

Where:
- R = robot radius (center to wheel)
- r = wheel radius
```

**Forward Kinematics (Odometry):**
The node reads encoder positions and uses the inverse to calculate robot displacement.

### Hardware Connections

**Serial Bus Wiring:**
```
Driver Board <-> Jetson Nano
  TX ---------> RX (via USB adapter or GPIO)
  GND --------> GND

Driver Board <-> Servos
  Serial bus daisy-chained to all 3 servos
  Each servo has unique ID (1, 2, 3)
```

**Power:**
- Servos: 12V power supply
- Jetson: 5V power supply
- Keep grounds common!

---

## 3. Setting Up Motors

### Step 1: Set Servo IDs

You need to set each servo to a unique ID using the manufacturer's software:
- Front wheel = ID 1
- Left rear wheel = ID 2
- Right rear wheel = ID 3

### Step 2: Connect to Jetson

1. Connect driver board to Jetson via USB or GPIO UART
2. Determine serial port: `ls /dev/ttyUSB* /dev/ttyTHS*`
3. Grant permissions: `sudo chmod 666 /dev/ttyUSB0` (or add user to dialout group)

### Step 3: Test Servo Communication

```bash
cd ~/bin-boy/src/bin_boy_control/bin_boy_control
python3 sts3215_driver.py
```

This will ping servos 1, 2, 3 and test basic motion.

### Step 4: Build and Run ROS2 Node

```bash
cd ~/bin-boy
colcon build --packages-select bin_boy_control
source install/setup.bash

# Test the node
ros2 launch bin_boy_control kiwi_drive.launch.py serial_port:=/dev/ttyUSB0
```

### Step 5: Control the Robot

```bash
# In another terminal
source install/setup.bash

# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"

# Strafe left
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0, y: 0.2, z: 0}, angular: {x: 0, y: 0, z: 0}}"

# Rotate CCW
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"
```

### Step 6: Verify Odometry

```bash
# Watch odometry output
ros2 topic echo /odom

# View in RViz
rviz2
# Add -> Odometry -> Topic: /odom
# Set Fixed Frame: odom
```

---

## 4. Calibration & Tuning

### Calibrate Physical Parameters

Measure your actual robot and update `config/kiwi_params.yaml`:

```yaml
wheel_radius: 0.05    # Measure actual wheel radius
robot_radius: 0.15    # Measure center to wheel distance
```

### Tune Speed Conversion

The `SPEED_FACTOR` in `sts3215_driver.py` (line ~265) converts m/s to servo units.

**To calibrate:**
1. Command a known velocity (e.g., 0.1 m/s)
2. Measure actual distance traveled in 10 seconds
3. Adjust SPEED_FACTOR until actual matches commanded

### Tune PID (if needed)

STS3215 servos have internal PID control. If needed, you can adjust P, D, I coefficients via the driver.

---

## 5. Current System Status

### Working
- [x] LIDAR running and visualizing
- [x] Robot URDF created
- [x] Motor driver code written
- [x] Kiwi drive kinematics implemented
- [x] Odometry calculation implemented
- [x] ROS2 nodes and launch files ready

### Pending
- [ ] Test servo communication
- [ ] Calibrate wheel odometry
- [ ] Tune motor speed conversions

### Next Steps (After Motors Work)
1. **IMU Integration** - Fuse IMU with wheel odometry using `robot_localization`
2. **SLAM** - Set up `slam_toolbox` for mapping
3. **Person Detection** - Integrate YOLOv5 camera for user tracking
4. **Following Behavior** - Create person-following logic
5. **Obstacle Avoidance** - Use LIDAR for collision avoidance
6. **Nav2** - Full navigation stack integration

---

## 5.5. IMU (MPU6050) Setup

### What We Have

**Package:** `bin_boy_control` (MPU6050 node included)

**Key Files:**
- `mpu6050_node.py` - ROS2 node for MPU6050 IMU
- `full_control.launch.py` - Integrated launch with kiwi drive + IMU + EKF
- `ekf.yaml` - Sensor fusion configuration
- `verify_mpu6050.py` - Hardware verification script
- `full_control.rviz` - RViz configuration with IMU visualization

### MPU6050 Overview

**Specifications:**
- 6-axis IMU (3-axis gyroscope + 3-axis accelerometer)
- I2C interface (address 0x68 or 0x69)
- Operating voltage: 3.3V or 5V (check your module)
- Sample rate: Up to 1kHz (configured for 100Hz)
- Gyroscope range: ±500°/s (configured)
- Accelerometer range: ±4g (configured)

**What It Provides:**
- Angular velocity (rad/s) for orientation estimation
- Linear acceleration (m/s²) for motion detection
- Temperature reading for sensor health monitoring

### Hardware Wiring

**Connecting MPU6050 to Jetson Nano:**

```
MPU6050 Pin    →    Jetson Nano (40-pin Header)
────────────────────────────────────────────────
VCC            →    Pin 1 (3.3V) or Pin 2 (5V)*
GND            →    Pin 6 (GND)
SCL            →    Pin 5 (I2C_2_SCL / GPIO3)
SDA            →    Pin 3 (I2C_2_SDA / GPIO2)
AD0            →    GND (for 0x68) or leave floating
INT            →    Not connected (optional)
```

**IMPORTANT:**
- We're using address **0x68** (AD0 pulled LOW to GND or left floating)
- This is I2C Bus 1 on the Jetson Nano
- Most MPU6050 modules have onboard 3.3V regulators and work with 5V input
- Verify your specific module's voltage requirements!

### Installation Steps

**Step 1: Install I2C Tools (if not installed)**

```bash
sudo apt-get update
sudo apt-get install i2c-tools python3-smbus

# Verify I2C is enabled
ls -l /dev/i2c-*

# Add user to i2c group for permissions
sudo usermod -a -G i2c $USER
# Log out and back in for group changes to take effect
```

**Step 2: Connect Hardware**

1. Power off Jetson Nano
2. Connect MPU6050 according to wiring diagram above
3. Ensure AD0 is connected to 3.3V for address 0x69
4. Double-check all connections
5. Power on Jetson Nano

**Step 3: Verify Hardware Connection**

```bash
# Scan I2C bus 1 for devices
sudo i2cdetect -y 1

# You should see "68" in the output (or "69" if AD0 is high)
# Example output:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
```

**Step 4: Run Verification Script**

```bash
cd ~/bin-boy/src/bin_boy_control/scripts
python3 verify_mpu6050.py
```

This script will:
- Scan I2C bus for devices
- Verify MPU6050 is present at 0x68
- Read WHO_AM_I register (should be 0x68)
- Wake up the sensor
- Read 5 samples of accelerometer, gyroscope, and temperature data
- Display all readings and confirm sensor is working

**Expected Output:**
```
============================================================
 MPU6050 Hardware Verification Tool
============================================================

============================================================
Scanning I2C bus 1...
============================================================
  Found device at address: 0x68

============================================================
Verifying MPU6050 at address 0x68 on bus 1...
============================================================
  WHO_AM_I register: 0x68
  ✓ MPU6050 identified successfully!
  Power Management: 0x00 (sensor awake: True)

============================================================
Reading sensor data from MPU6050...
============================================================

Sample 1:
  Accel (raw):  X=  -248  Y=   156  Z= 16234
  Gyro (raw):   X=   -45  Y=    32  Z=    12
  Temperature:  28.45°C

[... 4 more samples ...]

✓ Sensor data read successfully!

============================================================
 ✓ VERIFICATION COMPLETE - MPU6050 IS WORKING!
============================================================
```

### ROS2 Integration

**Step 5: Build the Package**

```bash
cd ~/bin-boy
colcon build --packages-select bin_boy_control
source install/setup.bash
```

**Step 6: Test IMU Node Standalone**

```bash
# Launch just the IMU node
ros2 run bin_boy_control mpu6050_node

# In another terminal, check the data
source install/setup.bash
ros2 topic echo /imu/data_raw

# Check publish rate
ros2 topic hz /imu/data_raw
# Should show ~100 Hz
```

**Step 7: Launch Full Control System**

```bash
# Launch everything: kiwi drive + IMU + EKF sensor fusion
ros2 launch bin_boy_control full_control.launch.py

# Without RViz (headless)
ros2 launch bin_boy_control full_control.launch.py use_rviz:=false

# With custom serial port
ros2 launch bin_boy_control full_control.launch.py serial_port:=/dev/ttyUSB0
```

**What This Launch File Does:**
1. Starts `kiwi_drive_node` - Motor control and wheel odometry
2. Starts `mpu6050_node` - IMU data publishing
3. Starts `ekf_filter_node` - Fuses wheel odometry + IMU data
4. (Optional) Starts RViz2 with IMU visualization

### Sensor Fusion with EKF

The `robot_localization` package's Extended Kalman Filter (EKF) fuses multiple sensor inputs:

**Inputs:**
- **Wheel Odometry** (`/odom` topic):
  - Position (x, y, yaw)
  - Velocities (vx, vy, vyaw)
  - High frequency (50Hz)
  - Subject to wheel slip

- **IMU Data** (`/imu/data_raw` topic):
  - Angular velocity (especially yaw rate)
  - Linear acceleration (x, y)
  - High frequency (100Hz)
  - Drift over time

**Output:**
- **Filtered Odometry** (`/odom_filtered` topic):
  - Best estimate combining both sensors
  - More accurate than either alone
  - Corrects for wheel slip using IMU
  - Corrects for IMU drift using wheels

**EKF Configuration** (`config/ekf.yaml`):
```yaml
odom0: /odom
odom0_config: [true,  true,  false,    # x, y, z position
               false, false, true,     # roll, pitch, yaw
               true,  true,  false,    # x_dot, y_dot, z_dot
               false, false, true,     # roll_dot, pitch_dot, yaw_dot
               false, false, false]    # x_ddot, y_ddot, z_ddot

imu0: /imu/data_raw
imu0_config: [false, false, false,     # x, y, z position
              false, false, true,      # roll, pitch, yaw (use yaw from IMU)
              false, false, false,     # x_dot, y_dot, z_dot
              false, false, true,      # roll_dot, pitch_dot, yaw_dot (use yaw rate)
              true,  true,  false]     # x_ddot, y_ddot, z_ddot (use accel x, y)
```

### Visualization with RViz

**View IMU and Sensor Fusion:**

```bash
# Launch with RViz
ros2 launch bin_boy_control full_control.launch.py

# Or launch RViz separately with the config
rviz2 -d install/bin_boy_control/share/bin_boy_control/rviz/full_control.rviz
```

**What You'll See:**
- **Robot Model** - Your bin-boy URDF
- **IMU Visualization** - Live IMU data arrows:
  - Red arrows: Linear acceleration
  - Green arrows: Angular velocity
- **Wheel Odometry** - Raw odometry from encoders (red/orange)
- **Filtered Odometry** - EKF-fused odometry (green)
- **Robot Path** - Trail showing robot movement
- **TF Frames** - All coordinate transforms

### Testing and Calibration

**Test 1: Static IMU Test**
```bash
# Robot stationary
ros2 topic echo /imu/data_raw

# Check:
# - Linear acceleration Z should be ~9.81 m/s² (gravity)
# - Angular velocities should be near zero (small drift is normal)
# - Values should be relatively stable
```

**Test 2: Rotation Test**
```bash
# Manually rotate the robot slowly
ros2 topic echo /imu/data_raw

# Check:
# - Angular velocity Z (yaw rate) changes when rotating
# - Sign is correct (positive = CCW, negative = CW)
```

**Test 3: Movement Test**
```bash
# Drive robot forward with motors
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0, z: 0}}"

# In another terminal, compare odometry sources
ros2 topic echo /odom           # Raw wheel odometry
ros2 topic echo /odom_filtered  # EKF-fused odometry

# Check:
# - Both should show forward movement
# - Filtered odometry should be smoother
# - IMU should show linear acceleration during acceleration/deceleration
```

**Test 4: Sensor Fusion Quality**
```bash
# Drive in a square pattern and return to start
# Compare final positions:
# - Wheel odometry will likely drift more
# - Filtered odometry should be more accurate
```

### Troubleshooting

**Problem: No I2C devices detected**
```bash
# Check I2C is enabled
ls -l /dev/i2c-*

# If missing, enable I2C in Jetson
# Edit /boot/config.txt or use jetson-io tool

# Check permissions
groups $USER  # Should include "i2c"
```

**Problem: Device at 0x69 instead of 0x68**
- AD0 pin is HIGH (connected to 3.3V)
- Either connect AD0 to GND, or update configuration to use 0x69:
```bash
ros2 run bin_boy_control mpu6050_node --ros-args -p i2c_address:=0x69
```

**Problem: IMU data looks wrong**
```bash
# Check WHO_AM_I register
sudo i2cget -y 1 0x68 0x75  # Should return 0x68

# Check sensor orientation
# Z-axis should point up when robot is level
# If inverted, you may need to flip the board or adjust signs in code
```

**Problem: EKF not publishing**
```bash
# Check if robot_localization is installed
ros2 pkg list | grep robot_localization

# If missing:
sudo apt install ros-humble-robot-localization  # or ros-foxy-robot-localization

# Check EKF status
ros2 node info /ekf_filter_node
```

**Problem: RViz doesn't show IMU**
```bash
# Check if rviz_imu_plugin is installed
sudo apt install ros-humble-rviz-imu-plugin  # or ros-foxy-rviz-imu-plugin

# If still not visible, manually add in RViz:
# Displays -> Add -> By topic -> /imu/data_raw -> Imu
```

### IMU Coordinate Frame

The IMU is mounted at the robot's base center with the following orientation:

```
      +Y (forward)
       ↑
       |
       |
  +Z   •────→ +X (right)
  (up)
```

**Axis Definitions:**
- **+X**: Right side of robot
- **+Y**: Forward direction
- **+Z**: Up (opposite of gravity)

**Angular Velocity:**
- **+Roll (X)**: Right side down
- **+Pitch (Y)**: Nose down
- **+Yaw (Z)**: Counter-clockwise rotation (viewed from above)

**Alignment with Robot:**
- IMU frame_id: `imu_link`
- Parent frame: `base_link`
- Transform defined in robot URDF

### Advanced Topics

**Calibrating IMU Biases:**

The MPU6050 has small constant biases. For best results:

1. Collect data while stationary
2. Calculate average gyro readings
3. Subtract these offsets in the node

(Future enhancement: auto-calibration on startup)

**Adding Magnetometer (Optional):**

MPU6050 doesn't have a magnetometer. For absolute heading:
- Add MPU9250/MPU9255 (includes magnetometer)
- Or add separate HMC5883L/QMC5883L magnetometer
- Update EKF config to use absolute orientation

**Improving Sensor Fusion:**

Tune these EKF parameters in `ekf.yaml` if needed:
- `process_noise_covariance`: How much to trust motion model
- `odom0_rejection_threshold`: Outlier rejection for wheel odom
- `imu0_rejection_threshold`: Outlier rejection for IMU

### Quick Reference Commands

```bash
# Hardware verification
python3 ~/bin-boy/src/bin_boy_control/scripts/verify_mpu6050.py

# Test IMU node
ros2 run bin_boy_control mpu6050_node

# Launch full system
ros2 launch bin_boy_control full_control.launch.py

# Check IMU data
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw

# Check fused odometry
ros2 topic echo /odom_filtered

# View TF tree
ros2 run tf2_tools view_frames

# Scan I2C bus
sudo i2cdetect -y 1
```

---

## 6. System Architecture (Full Vision)

```
┌─────────────────────────────────────────────────────────┐
│                    Behavior Layer                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │Person Follow │  │   Obstacle   │  │State Machine │   │
│  │    Logic     │  │  Avoidance   │  │ (FSM)        │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                  Planning & Navigation                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │ slam_toolbox │  │    Nav2      │  │  Costmaps    │   │
│  │   (SLAM)     │  │   Planner    │  │              │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                  Perception Layer                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │   YOLOv5     │  │    LIDAR     │  │robot_local.  │   │
│  │Person Detect │  │  Scan Data   │  │  (EKF Odom)  │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                    Control Layer                        │
│  ┌──────────────┐  ┌──────────────┐                     │
│  │ Kiwi Drive   │  │   cmd_vel    │                     │
│  │  Controller  │←─│  Subscriber  │                     │
│  └──────────────┘  └──────────────┘                     │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                   Hardware Layer                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │  STS3215     │  │   LD14P      │  │  MPU-6050    │   │
│  │   Servos     │  │   LIDAR      │  │    IMU       │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────┘
```

---

## 7. Package Dependencies

Install these when needed:

```bash
# SLAM
sudo apt install ros-foxy-slam-toolbox

# Navigation
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup

# Sensor fusion
sudo apt install ros-foxy-robot-localization

# Python dependencies
pip3 install pyserial opencv-python
```

---

## 8. Troubleshooting

### Servos Not Responding
- Check power supply (12V, sufficient current)
- Verify serial port permissions
- Confirm baud rate (1000000)
- Test with manufacturer software first

### Odometry Drifting
- Calibrate wheel radius
- Calibrate robot radius
- Add IMU fusion with robot_localization
- Check for wheel slippage

### LIDAR Not Publishing
```bash
# Check if LIDAR is running
ros2 topic hz /scan

# Restart LIDAR node
ros2 launch ldlidar_sl_ros2 ld14p.launch.py
```

### TF Errors
```bash
# View TF tree
ros2 run tf2_tools view_frames.py

# Check for missing transforms
ros2 run tf2_ros tf2_echo base_footprint base_laser
```

---

## 9. Recommended Development Order

**Phase 1: Foundation**
1. Get LIDAR working
2. Create robot URDF
3. Get motors communicating
4. Tune odometry

**Phase 2: Perception**
5. Add IMU data publishing
6. Fuse odometry with robot_localization
7. Test person detection with YOLOv5
8. Create person tracking node

**Phase 3: Mapping**
9. Set up slam_toolbox
10. Create maps of environment
11. Tune SLAM parameters

**Phase 4: Behavior**
12. Implement person-following logic
13. Add obstacle avoidance
14. Create state machine (follow/wait/avoid/dock)

**Phase 5: Integration**
15. Nav2 integration for path planning
16. Fine-tune all parameters
17. Real-world testing

---

## 10. Additional Resources

**STS3215 Documentation:**
- Similar to Dynamixel protocol
- Feetech website has SDK examples
- Check Waveshare documentation

**ROS2 Learning:**
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

**Kiwi Drive:**
- [Kiwi Drive Kinematics](https://robotics.stackexchange.com/questions/19613/kiwi-drive-inverse-kinematics)
- Three-wheel omnidirectional robot papers

---

## Summary: What to Do Next

**When motors arrive:**
1. Set servo IDs (1, 2, 3)
2. Connect to Jetson
3. Run test script: `python3 sts3215_driver.py`
4. Launch ROS2 node: `ros2 launch bin_boy_control kiwi_drive.launch.py`
5. Test with cmd_vel commands
6. Calibrate odometry
7. Move to Phase 2 (IMU + person detection)

**You now have:**
- Complete URDF robot model
- Motor driver library
- Kiwi drive kinematics
- Odometry calculation
- ROS2 integration
- Launch files and configs

Everything is ready to test as soon as your motors arrive, sir.
