#!/usr/bin/env python3
"""
Print one arm's joint angles (deg) and gripper opening (mm). Read-only, never moves the arm.

Use it to find out which CAN interface is which arm: run it, move an arm by hand,
and see whether the numbers change.

    python3 tools/check_joints.py can_leader_r
"""
import sys
import time

from piper_sdk import C_PiperInterface_V2

if len(sys.argv) != 2:
    sys.exit(__doc__)

arm = C_PiperInterface_V2(sys.argv[1])
arm.ConnectPort()
time.sleep(0.5)
print(f"Reading {sys.argv[1]} (Ctrl+C to stop)\n   j1     j2     j3     j4     j5     j6  | grip mm |  Hz")
try:
    while True:
        msg = arm.GetArmJointMsgs()
        js = msg.joint_state
        joints = [getattr(js, f"joint_{i}") / 1000 for i in range(1, 7)]
        grip = arm.GetArmGripperMsgs().gripper_state.grippers_angle / 1000
        print(" ".join(f"{j:6.1f}" for j in joints), f"| {grip:7.1f} | {msg.Hz:4.0f}")
        time.sleep(0.3)
except KeyboardInterrupt:
    pass
