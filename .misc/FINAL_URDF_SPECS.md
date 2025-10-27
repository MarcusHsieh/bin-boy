# Final URDF Specifications - Bin Boy Robot

## Geometry Summary

### Base Body (White Cylinder)
- **Diameter:** 315mm (radius: 157.5mm)
- **Height:** 300mm (reduced from 800mm)
- **Position:** Bottom of cylinder at z=0 relative to base_link
- **Ground clearance:** Bottom of cylinder sits exactly at ground level (z=0 world frame)

### Wheels (3x Black Cylinders - Kiwi Drive)
- **Diameter:** 145mm (radius: 72.5mm)
- **Width:** 38mm
- **Position from center:** 181.5mm (157.5mm base + 19mm half-wheel + 5mm clearance)
- **Layout:** 0°, 120°, 240° around robot center
- **Orientation:** Each wheel tangent to circle (perpendicular to radius)
- **Ground contact:** Wheel bottoms exactly at ground level (z=0)
- **NO OVERLAP with body:** Wheels positioned completely outside cylinder

### Coordinate Frame Heights (Above Ground)
- **base_footprint:** 0mm (on ground)
- **base_link:** 72.5mm (at wheel center height)
- **Cylinder bottom:** 0mm (sits on ground)
- **Cylinder top:** 300mm
- **Camera:** 150mm from ground
- **LIDAR center:** 319mm (300mm + 19mm half-height)

### LIDAR Sensor (LD14P)
- **Diameter:** 96.3mm
- **Height:** 38mm
- **Position:** Centered on TOP of cylinder
- **Height above ground:** 319mm (cylinder top + half LIDAR height)
- **NO OVERLAP:** LIDAR sits completely on top, no penetration into body

### Camera (CSI Wide Angle)
- **Size:** 25mm × 25mm × 15mm box
- **Position:** On edge of cylinder, 150mm from ground
- **Radial position:** 157.5mm + 12.5mm = 170mm from center
- **No tilt:** Points straight forward (0° pitch)
- **NO OVERLAP:** Camera positioned outside cylinder edge

### IMU (MPU-6050)
- **Size:** 20mm × 16mm × 3mm
- **Position:** Inside base, centered, 50mm above base_link (122.5mm above ground)

---

## Frame Tree

```
base_footprint (ground, z=0)
    └── base_link (z=72.5mm, wheel center height)
        ├── cylinder_visual (bottom at z=0, top at z=300mm)
        │
        ├── front_wheel
        │   └── position: (181.5mm, 0, 0) at angle 0°
        │       wheel center at z=72.5mm, bottom touches ground
        │
        ├── left_rear_wheel
        │   └── position: 181.5mm at angle 120°
        │       wheel center at z=72.5mm, bottom touches ground
        │
        ├── right_rear_wheel
        │   └── position: 181.5mm at angle 240°
        │       wheel center at z=72.5mm, bottom touches ground
        │
        ├── base_laser (LIDAR)
        │   └── position: (0, 0, 319mm) - on top of cylinder
        │
        ├── camera_link
        │   └── position: (170mm, 0, 150mm) - on edge, halfway up
        │       └── camera_optical_frame
        │
        └── imu_link
            └── position: (0, 0, 122.5mm) - inside cylinder
```

---

## Dimensional Calculations

### Wheel Clearance from Body
```
Cylinder radius:        157.5mm
Wheel width/2:           19.0mm
Clearance:                5.0mm
                        -------
Wheel center distance:  181.5mm
```

Gap between cylinder edge and wheel inner edge: **5mm** 

### Ground Contact Verification
```
Wheel radius:            72.5mm
base_link height:        72.5mm  (= wheel radius)
Wheel z-position:         0.0mm  (relative to base_link)

Wheel center absolute:   72.5mm above ground
Wheel bottom absolute:    0.0mm (72.5 - 72.5) TOUCHES GROUND

Cylinder bottom:          0.0mm (at base_link z=0) ON GROUND
```

### LIDAR Clearance from Body
```
Cylinder height:        300.0mm
Cylinder top:           300.0mm above ground
LIDAR bottom:           300.0mm (sits on top)
LIDAR center:           319.0mm (300 + 19mm)
```

LIDAR completely above cylinder: **NO OVERLAP**

---

## Parameters for Control

**Updated in `kiwi_params.yaml` and `sts3215_driver.py`:**

```yaml
wheel_radius: 0.0725  # 72.5mm
robot_radius: 0.1815  # 181.5mm (wheel centers from origin)
```

These match the URDF geometry exactly.

---

## Visualization Test

```bash
cd ~/bin-boy
source install/setup.bash
ros2 launch bin_boy_description display.launch.py
```

**Expected View:**
- Short white cylinder (315mm wide × 300mm tall) on ground
- 3 black wheels outside cylinder, touching ground
- Small black LIDAR disk on top of cylinder
- Blue camera box on edge at mid-height
- Green IMU inside cylinder
- All components properly spaced with no overlaps

---

## Summary

✅ **All requirements met:**

1. Wheels positioned outside body (5mm clearance)
2. Cylinder lowered to 300mm
3. LIDAR on top of cylinder (no overlap)
4. Camera on edge at 150mm height
5. Wheels touch ground (not buried)
6. Entire model properly elevated
7. Parameters synchronized across all files

