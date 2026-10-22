#!/usr/bin/env python3
"""
Set an arm's built-in role (stored on the arm). Needed once per arm.

    python3 tools/set_role.py can_leader_r leader      # MasterSlaveConfig(0xFA): teaching input arm
    python3 tools/set_role.py can_follower_r follower  # MasterSlaveConfig(0xFC): normal / output arm

leader   : joints AND gripper are moved by hand; the arm publishes its pose as control
           frames (CAN 0x155-0x157, 0x159) that teleop.py / record.py read.
follower : normal mode; accepts joint/gripper commands and reports feedback.

After setting, POWER-CYCLE the arm (support it first, it goes limp), then run
`bash scripts/setup_can.sh` and check with `python3 tools/arm_status.py <can>`.
"""
import sys
import time

from piper_sdk import C_PiperInterface_V2

ROLES = {"leader": 0xFA, "follower": 0xFC}

if len(sys.argv) != 3 or sys.argv[2] not in ROLES:
    sys.exit(__doc__)
can, role = sys.argv[1], sys.argv[2]

print(f"About to set the arm on {can} as {role.upper()} (0x{ROLES[role]:02X}).")
if role == "leader" and "follower" in can:
    print("  !! the interface name says follower. Are the adapters swapped?")
if role == "follower" and "leader" in can:
    print("  !! the interface name says leader. Are the adapters swapped?")
if input("Type 'yes' to continue: ").strip().lower() != "yes":
    sys.exit("Cancelled.")

arm = C_PiperInterface_V2(can)
arm.ConnectPort()
time.sleep(0.3)
for _ in range(3):
    arm.MasterSlaveConfig(ROLES[role], 0, 0, 0)
    time.sleep(0.05)

print(f"\nSent. Now:\n"
      f"  1. support the arm and power-cycle it\n"
      f"  2. bash scripts/setup_can.sh\n"
      f"  3. python3 tools/arm_status.py {can}")
