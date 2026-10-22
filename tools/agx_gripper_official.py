#!/usr/bin/env python3
"""
Test / calibrate the AgileX gripper ("gripper for PiPER/NERO") with AgileX's OFFICIAL
pyAgxArm SDK, to rule out our own code. Run on the FOLLOWER.

Install once:
    pip3 install "git+https://github.com/agilexrobotics/pyAgxArm.git"

    python3 tools/agx_gripper_official.py can_follower_r               # status only
    python3 tools/agx_gripper_official.py can_follower_r --calibrate   # official calibration
    python3 tools/agx_gripper_official.py can_follower_r --move        # open 30 mm, then close
Options:  --range 0.07 (default, small gripper) | --range 0.1 (large gripper)
          --force 2.0   gripper force in N for --move (0-3, default 1.0)

Calibration follows pyAgxArm/demos/piper/test1.py:
  disable gripper -> set max stroke -> squeeze it closed by hand -> calibrate (set zero)
--move enables the arm; joints hold their current pose. Keep the e-stop in reach.
"""
import sys
import time

from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile

if len(sys.argv) < 2:
    sys.exit(__doc__)
CAN = sys.argv[1]
ARGS = sys.argv[2:]
RANGE = float(ARGS[ARGS.index("--range") + 1]) if "--range" in ARGS else 0.07
FORCE = float(ARGS[ARGS.index("--force") + 1]) if "--force" in ARGS else 1.0


def connect():
    # detect firmware first, like pyAgxArm/demos/detect_piper_series.py
    cfg = create_agx_arm_config(robot="piper", interface="socketcan", channel=CAN)
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    t0 = time.time()
    while robot.get_firmware() is None:
        if time.time() - t0 > 10:
            sys.exit(f"No firmware reply on {CAN}. Arm powered? scripts/setup_can.sh run?")
        time.sleep(0.5)
    sv = robot.get_firmware()["software_version"]
    fw = resolve_firmware_profile("piper", sv)
    robot.disconnect()
    cfg = create_agx_arm_config(robot="piper", firmeware_version=fw, interface="socketcan", channel=CAN)
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    eff = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    print(f"Connected to {CAN}: firmware {sv} (profile {fw})")
    return robot, eff


def status(eff, title):
    t0 = time.time()
    gs = eff.get_gripper_status()
    while gs is None and time.time() - t0 < 3:
        time.sleep(0.1)
        gs = eff.get_gripper_status()
    print(f"\n--- {title} ---")
    if gs is None:
        print("  NO gripper feedback at all (CAN 0x2A8). Check the gripper cable at the wrist.")
        return None
    m, f = gs.msg, gs.msg.foc_status
    unit = "deg" if m.mode == "angle" else "mm"
    val = m.value if m.mode == "angle" else m.value * 1000
    print(f"  mode {m.mode}   position {val:.1f} {unit}   force {m.force:.2f} N   feedback {gs.hz:.0f} Hz")
    print(f"  enabled={f.driver_enable_status}  homed={f.homing_status}")
    bad = [k for k in ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                       "driver_overheating", "sensor_status", "driver_error_status") if getattr(f, k)]
    print(f"  errors: {', '.join(bad) if bad else 'none'}")
    p = eff.get_gripper_teaching_pendant_param()
    if p is not None:
        print(f"  max stroke setting: {p.msg.max_range_config} m")
    return m.mode


robot, eff = connect()
mode = status(eff, "current state")

if "--calibrate" in ARGS:
    print("\nCalibrating (official sequence) ...")
    eff.disable_gripper()
    time.sleep(0.5)
    ok = eff.set_gripper_teaching_pendant_param(max_range_config=RANGE)
    print(f"  max stroke set to {RANGE} m: {'OK' if ok else 'no ACK'}")
    input("  Now SQUEEZE THE GRIPPER FULLY CLOSED BY HAND, then press Enter ...")
    status(eff, "after squeezing")
    ok = eff.calibrate_gripper(timeout=2.0)
    print(f"  calibrate (set zero): {'OK, ACK received' if ok else 'NO ACK from gripper'}")
    print("  Power-cycle the arm, run scripts/setup_can.sh, then run --move.")

if "--move" in ARGS:
    print("\nEnabling arm (joints hold their pose) ...")
    t0 = time.time()
    while not robot.enable():
        if time.time() - t0 > 5:
            sys.exit("Arm did not enable. Check e-stop / power, and that it is NOT in teaching mode.")
        time.sleep(0.01)
    ja = robot.get_joint_angles()
    if ja is not None:                     # CAN joint mode, target = current pose (no motion)
        robot.set_speed_percent(10)
        robot.move_j(list(ja.msg))
    print(f"  force {FORCE} N")
    for target_m, secs in ((0.03, 3.0), (0.0, 2.0)):
        print(f"\nCommanding {target_m * 1000:.0f} mm for {secs:.0f} s ...")
        t0, last = time.time(), 0
        while time.time() - t0 < secs:
            if mode == "angle":
                eff.move_gripper_deg(value=target_m * 1000, force=FORCE)   # rough: treat mm as deg
            else:
                eff.move_gripper_m(value=target_m, force=FORCE)
            if time.time() - last > 0.5:
                last = time.time()
                gs = eff.get_gripper_status()
                if gs is not None:
                    v = gs.msg.value if gs.msg.mode == "angle" else gs.msg.value * 1000
                    print(f"  position {v:6.1f}   force {gs.msg.force:5.2f} N")
            time.sleep(0.02)
    status(eff, "after move test")

robot.disconnect()
