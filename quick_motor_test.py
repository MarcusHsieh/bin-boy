#!/usr/bin/env python3
"""
Quick motor movement verification test
Moves motors briefly to confirm physical movement
"""

import sys
sys.path.append('/home/jetson/bin-boy/src/bin_boy_control')

from bin_boy_control.sts3215_driver import KiwiDriveController
import time

print("Quick Motor Movement Test")
print("="*60)

# Initialize
controller = KiwiDriveController([1, 2, 3], 1000000, '/dev/ttyACM0')
print("Controllers initialized")

# Test forward motion
print("\nMoving forward for 1 second...")
controller.set_velocity(vx=0, vy=0, omega=-200)
time.sleep(5)

# Stop
print("Stopping...")
controller.stop()

print("\n✅ Test complete! If motors moved, integration is working correctly.")
