#!/usr/bin/env python3
"""
Leader -> follower teleoperation (no recording). Use this to test the setup.

    python3 teleop.py                                   # right pair, default names
    python3 teleop.py --speed 30                        # gentler first test
    python3 teleop.py --pair right:can_leader_r:can_follower_r \
                      --pair left:can_leader_l:can_follower_l   # bimanual

Joints follow the leader. The follower GRIPPER follows the leader's teaching pendant
(0-100 mm). With --gripper keyboard it is driven by keys instead (no Enter needed):
o = open   c = close   [ / ] = 10 mm less / more

Ctrl+C stops sending commands. The follower stays powered and HOLDS its last pose.
Add --disable-on-exit to make it go limp instead (support it by hand first!).
"""
import argparse
import time

from piper_teleop import Bridge, parse_pair
from piper_teleop.keyboard import KeyReader, gripper_help, handle_gripper_key


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pair", action="append", help="side:leader_can:follower_can (repeat for 2 pairs)")
    ap.add_argument("--speed", type=int, default=60, help="follower speed %% once synced (default 60)")
    ap.add_argument("--start-speed", type=int, default=15, help="follower speed %% while syncing (default 15)")
    ap.add_argument("--hz", type=int, default=200, help="command rate (default 200)")
    ap.add_argument("--gripper", choices=["leader", "keyboard"], default="leader",
                    help="where the follower gripper command comes from (default leader)")
    ap.add_argument("--gripper-scale", type=float, default=1.0, help="--gripper leader: follower = leader x scale")
    ap.add_argument("--gripper-effort", type=float, default=1.0,
                    help="follower gripper max force, N·m (0-5, default 1.0)")
    ap.add_argument("--disable-on-exit", action="store_true", help="follower goes limp on exit (it will FALL)")
    args = ap.parse_args()

    specs = args.pair or ["right:can_leader_r:can_follower_r"]
    pairs = [parse_pair(s, speed=args.speed, start_speed=args.start_speed,
                        gripper_scale=args.gripper_scale, gripper=args.gripper,
                        gripper_effort_nm=args.gripper_effort) for s in specs]

    bridge = Bridge(pairs, hz=args.hz)
    bridge.start()
    print("\nTeleop running. Move the leader. Ctrl+C to stop.")
    if args.gripper == "keyboard":
        print(gripper_help(pairs))
    print()

    try:
        with KeyReader() as kb:
            last_print = 0.0
            while True:
                for ch in kb.keys():
                    handle_gripper_key(pairs, ch)
                if time.time() - last_print > 1.0:
                    last_print = time.time()
                    for p in pairs:
                        (lj, _), (fj, fg) = p.leader.read_raw(), p.follower.read_raw()
                        err = max(abs(a - b) for a, b in zip(lj, fj)) / 1000
                        state = "paused" if p.paused else ("synced" if p.synced else "syncing")
                        lead = "moving" if p.leader.age() < 0.3 else "still "
                        print(f"[{p.side}] {state:8s} leader {lead}  max joint error {err:5.1f} deg   "
                              f"gripper {fg / 1000:4.0f} mm")
                time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopping ...")
    finally:
        bridge.stop()
        if args.disable_on_exit:
            input("Support the follower arm(s) by hand, then press Enter to disable ...")
            for p in pairs:
                p.follower.disable()
            print("Followers disabled.")
        else:
            print("Followers are holding position (still enabled).")


if __name__ == "__main__":
    main()
