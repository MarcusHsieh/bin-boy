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
│  └──────────────┘  └──────────────┘  └──────────────┘  │
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
