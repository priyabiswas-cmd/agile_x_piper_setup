#!/usr/bin/env python3
"""
Show each arm's control mode and what that means physically. Read-only, sends no commands.

    python3 tools/arm_status.py can_leader_r can_follower_r

Ready for teleop:
  leader   -> TEACHING_MODE + START_RECORDING   (drag mode: free to move by hand)
  follower -> STANDBY or CAN_CTRL                (NOT teaching mode)

The teach button toggles START_RECORDING (drag, free) <-> STOP_RECORDING (holds pose, stiff).
"""
import sys
import time

from piper_sdk import C_PiperInterface_V2

TEACHING, CAN_CTRL, STANDBY = 0x02, 0x01, 0x00
START_REC, STOP_REC = 0x01, 0x02


def meaning(mode, teach, drivers_on):
    if mode == TEACHING and teach == START_REC:
        return "drag mode: FREE to move by hand        (correct for LEADER)"
    if mode == TEACHING:
        return "teaching mode, drag OFF: HOLDS pose (stiff). Short-press teach to enter drag mode"
    if mode == CAN_CTRL:
        return "laptop (CAN) control: follows commands (correct for FOLLOWER while teleop runs)"
    if mode == STANDBY:
        return ("standby, motors on: holds pose" if drivers_on else
                "standby, motors off: limp          (correct for FOLLOWER before teleop)")
    return "other mode"


if len(sys.argv) < 2:
    sys.exit(__doc__)

for can in sys.argv[1:]:
    arm = C_PiperInterface_V2(can)
    arm.ConnectPort()
    time.sleep(0.6)
    st = arm.GetArmStatus().arm_status
    drivers = arm.GetArmEnableStatus()
    grip = arm.GetArmGripperMsgs().gripper_state
    mode, teach = int(st.ctrl_mode), int(st.teach_status)
    print(f"\n=== {can} ===")
    print(f"  mode          : {st.ctrl_mode}")
    print(f"  teach status  : {st.teach_status}")
    print(f"  arm status    : {st.arm_status}")
    print(f"  motor drivers : " + " ".join(f"j{i + 1}={'on' if on else 'off'}" for i, on in enumerate(drivers))
          + f"  gripper={'on' if grip.foc_status.driver_enable_status else 'off'}")
    print(f"  gripper       : {grip.grippers_angle / 1000:.1f} mm open")
    print(f"  => {meaning(mode, teach, all(drivers))}")
    arm.DisconnectPort()
