#!/usr/bin/env python3
"""
Diagnose a gripper that does not respond to open/close commands. Run on the FOLLOWER.

    python3 tools/gripper_debug.py can_follower_r              # read-only report
    python3 tools/gripper_debug.py can_follower_r --test       # clear errors, enable, open 30 mm, close
    python3 tools/gripper_debug.py can_follower_r --set-range 70   # set max travel to 70 mm (small gripper)
    python3 tools/gripper_debug.py can_follower_r --zero       # set zero point: ONLY with gripper fully CLOSED
    python3 tools/gripper_debug.py can_follower_r --test --canmode --effort 3000
        # same test, but first switch the arm to CAN-command mode (holding its current pose)
        # and use a stronger gripper effort (0-5000, default 1000)

--test enables the arm (joints hold their current pose) the same way AgileX's
piper_ctrl_gripper.py demo does. Keep the e-stop in reach.
"""
import sys
import time

from piper_sdk import C_PiperInterface_V2

if len(sys.argv) < 2:
    sys.exit(__doc__)
can = sys.argv[1]
args = sys.argv[2:]
effort = int(args[args.index("--effort") + 1]) if "--effort" in args else 1000

arm = C_PiperInterface_V2(can)
arm.ConnectPort()
time.sleep(0.6)


def report(title):
    g = arm.GetArmGripperMsgs().gripper_state
    f = g.foc_status
    st = arm.GetArmStatus().arm_status
    print(f"\n--- {title} ---")
    print(f"  arm mode        : {st.ctrl_mode}   arm status: {st.arm_status}")
    print(f"  gripper opening : {g.grippers_angle / 1000:.1f} mm    effort {g.grippers_effort / 1000:.2f} N·m")
    print(f"  enabled={f.driver_enable_status}  homed/zeroed={f.homing_status}")
    errs = {k: getattr(f, k) for k in ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                                        "driver_overheating", "sensor_status", "driver_error_status")}
    bad = [k for k, v in errs.items() if v]
    print(f"  errors          : {', '.join(bad) if bad else 'none'}")


def read_params():
    arm.ArmParamEnquiryAndConfig(0x04)            # ask for gripper/teaching params (0x47E)
    for _ in range(20):
        time.sleep(0.05)
        fb = arm.GetGripperTeachingPendantParamFeedback()
        if fb.time_stamp > 0:
            p = fb.arm_gripper_teaching_param_feedback
            print(f"  max travel      : {p.max_range_config} mm   (0 = NOT SET, 70 = small gripper, 100 = large)")
            print(f"  teach coeff     : {p.teaching_range_per}   friction: {p.teaching_friction}")
            return p.max_range_config
    print("  max travel      : no answer (older firmware may not support this query)")
    return None


report("current state")
max_range = read_params()

if "--set-range" in args:
    val = int(args[args.index("--set-range") + 1])
    if val not in (0, 70, 100):
        sys.exit("--set-range must be 70 (small gripper) or 100 (large gripper)")
    arm.GripperTeachingPendantParamConfig(100, val, 1)
    time.sleep(0.3)
    print(f"\nSet max travel to {val} mm. Reading back:")
    read_params()
    print("Power-cycle the arm, then run --test.")

if "--zero" in args:
    if input("\nIs the gripper FULLY CLOSED? Type 'yes' to set this as zero: ").strip().lower() == "yes":
        arm.GripperCtrl(0, 1000, 0x00, 0)        # disable gripper
        time.sleep(1.5)
        arm.GripperCtrl(0, 1000, 0x00, 0xAE)     # set current position as zero
        time.sleep(0.5)
        report("after setting zero")

if "--test" in args:
    if int(arm.GetArmStatus().arm_status.ctrl_mode) == 0x02:
        sys.exit("\nSTOP: this arm is in TEACHING MODE, so it ignores all commands (gripper included).\n"
                 "Support the arm, power-cycle it, do NOT press its teach button, run\n"
                 "`bash scripts/setup_can.sh`, then run this --test again.")
    print("\nEnabling arm (joints will hold their pose) ...")
    t0 = time.time()
    while not arm.EnablePiper():
        if time.time() - t0 > 5:
            sys.exit("Arm did not enable. Check e-stop / power.")
        time.sleep(0.01)
    if "--canmode" in args:
        # target = current pose first, so switching to CAN joint mode does not move the arm
        js = arm.GetArmJointMsgs().joint_state
        cur = [getattr(js, f"joint_{i}") for i in range(1, 7)]
        for _ in range(20):
            arm.MotionCtrl_2(0x01, 0x01, 10, 0x00)
            arm.JointCtrl(*cur)
            time.sleep(0.01)
        print(f"  arm mode now: {arm.GetArmStatus().arm_status.ctrl_mode}")
    arm.GripperCtrl(0, effort, 0x02, 0)           # disable + clear errors
    time.sleep(0.3)
    arm.GripperCtrl(0, effort, 0x01, 0)           # enable
    time.sleep(0.3)
    report("after clear-errors + enable")
    for target_mm, secs in ((30, 3.0), (0, 2.0)):
        print(f"\nCommanding {target_mm} mm for {secs:.0f} s ...")
        t0 = time.time()
        last = 0
        while time.time() - t0 < secs:
            if "--canmode" in args:
                arm.MotionCtrl_2(0x01, 0x01, 10, 0x00)
                arm.JointCtrl(*cur)
            arm.GripperCtrl(target_mm * 1000, effort, 0x01, 0)
            if time.time() - last > 0.5:
                last = time.time()
                gs = arm.GetArmGripperMsgs().gripper_state
                print(f"  opening {gs.grippers_angle / 1000:5.1f} mm   effort {gs.grippers_effort / 1000:5.2f}")
            time.sleep(0.005)
    report("after test")
    print("\nIf the opening reached ~30 mm, the gripper works: tell Claude, the fix goes into teleop.")
