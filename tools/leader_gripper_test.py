#!/usr/bin/env python3
"""
Test whether the LEADER's gripper can be opened by hand once its motor is switched off.

Sends ONE command, to the leader's gripper only (joints are untouched): gripper motor OFF.
Then prints the gripper opening for 15 s while you try to open/close it by hand.

    python3 tools/leader_gripper_test.py can_leader_r            # switch gripper motor off + watch
    python3 tools/leader_gripper_test.py can_leader_r --enable   # switch it back on
"""
import sys
import time

from piper_sdk import C_PiperInterface_V2

if len(sys.argv) < 2:
    sys.exit(__doc__)
can = sys.argv[1]
enable = "--enable" in sys.argv

arm = C_PiperInterface_V2(can)
arm.ConnectPort()
time.sleep(0.5)

code = 0x01 if enable else 0x00          # 0x01 = enable, 0x00 = disable (motor off)
for _ in range(5):                        # send a few times so it isn't missed
    arm.GripperCtrl(0, 0, code, 0)
    time.sleep(0.05)
time.sleep(0.3)

g = arm.GetArmGripperMsgs().gripper_state
print(f"gripper motor is now {'ON' if g.foc_status.driver_enable_status else 'OFF'}")
if enable:
    sys.exit(0)

print("Try to open and close the leader's gripper by hand now (15 s) ...\n")
t0 = time.time()
while time.time() - t0 < 15:
    g = arm.GetArmGripperMsgs().gripper_state
    print(f"  opening {g.grippers_angle / 1000:6.1f} mm   motor {'ON' if g.foc_status.driver_enable_status else 'off'}")
    time.sleep(0.5)
print("\nDone. If the number changed when you moved it, the leader gripper can drive the follower.")
