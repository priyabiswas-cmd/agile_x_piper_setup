#!/usr/bin/env python3
"""
Read-only check of what the laptop receives from the LEADER. Sends no commands.

    python3 tools/leader_diag.py can_leader_r

Part 1 (6 s): raw CAN frames per ID. Keep the leader STILL for 3 s, then MOVE it.
Part 2 (4 s): what piper_sdk reports. Keep moving the leader.
"""
import collections
import sys
import time

import can

ch = sys.argv[1] if len(sys.argv) > 1 else "can_leader_r"

bus = can.interface.Bus(channel=ch, interface="socketcan")
still, moving = collections.Counter(), collections.Counter()
print("Part 1: keep the leader STILL ...")
t0, announced = time.time(), False
while time.time() - t0 < 6:
    el = time.time() - t0
    if el > 3 and not announced:
        print("        now MOVE the leader ...")
        announced = True
    m = bus.recv(0.1)
    if m is not None:
        (still if el < 3 else moving)[f"0x{m.arbitration_id:03X}"] += 1
bus.shutdown()
print("  frames while STILL :", dict(sorted(still.items())) or "NONE")
print("  frames while MOVING:", dict(sorted(moving.items())) or "NONE")

print("\nPart 2: piper_sdk view (keep moving the leader) ...")
from piper_sdk import C_PiperInterface_V2  # noqa: E402

arm = C_PiperInterface_V2(ch)
arm.ConnectPort()
for _ in range(8):
    time.sleep(0.5)
    jc, gc, js = arm.GetArmJointCtrl(), arm.GetArmGripperCtrl(), arm.GetArmJointMsgs()
    age = time.time() - jc.time_stamp if jc.time_stamp else -1
    joints = [round(getattr(jc.joint_ctrl, f"joint_{i}") / 1000, 1) for i in range(1, 7)]
    print(f"  ctrl age {age:6.2f}s  joints {joints}  grip {gc.gripper_ctrl.grippers_angle / 1000:.1f}mm"
          f"  | feedback ts {js.time_stamp:.0f}  isOk {arm.isOk()}")
