#!/usr/bin/env python3
"""
Record teleop episodes as a LeRobot dataset.

Teleop runs in a background thread at --hz (smooth follower); frames are logged at --fps.

Per frame (7 values per pair = 6 joints [rad] + gripper [m]):
  observation.state         follower measured pose
  action                    leader pose (= what the follower was commanded)
  observation.images.<name> RealSense colour image

    python3 record.py --repo-id rpl/piper_pick_cube --task "pick up the red cube" \
        --cam wrist=123456789012 --cam top=234567890123

Keys (single press, no Enter):  Enter = start / stop+save episode   r = discard   q = quit
Follower gripper follows the leader's teaching pendant (0-100 mm).
With --gripper keyboard:  o = open   c = close   [ / ] = 10 mm less / more
Find camera serials with:  python3 -c "from piper_teleop.cameras import list_serials; print(list_serials())"
"""
import argparse
import inspect
import time

import numpy as np

from piper_teleop import JOINT_NAMES, Bridge, parse_pair
from piper_teleop.cameras import RealSense
from piper_teleop.keyboard import KeyReader, gripper_help, handle_gripper_key

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset          # lerobot >= 0.3
except ImportError:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset   # older lerobot


# ---- small compatibility layer across lerobot versions ----------------------
def ds_add_frame(ds, frame, task):
    if "task" in inspect.signature(ds.save_episode).parameters:   # old API: task given at save
        ds.add_frame(frame)
    else:
        ds.add_frame({**frame, "task": task})


def ds_save_episode(ds, task):
    if "task" in inspect.signature(ds.save_episode).parameters:
        ds.save_episode(task=task)
    else:
        ds.save_episode()


def ds_discard_episode(ds):
    if hasattr(ds, "clear_episode_buffer"):
        ds.clear_episode_buffer()
    else:
        ds.episode_buffer = ds.create_episode_buffer()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pair", action="append", help="side:leader_can:follower_can (repeat for 2 pairs)")
    ap.add_argument("--repo-id", required=True, help="dataset name, e.g. rpl/piper_pick_cube")
    ap.add_argument("--task", required=True, help="language instruction stored with every frame")
    ap.add_argument("--root", default=None, help="where to save (default: ~/.cache/huggingface/lerobot/<repo-id>)")
    ap.add_argument("--cam", action="append", default=[], help="name=serial or serial (repeatable)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--hz", type=int, default=200, help="teleop command rate")
    ap.add_argument("--speed", type=int, default=60)
    ap.add_argument("--start-speed", type=int, default=15)
    ap.add_argument("--gripper", choices=["leader", "keyboard"], default="leader")
    ap.add_argument("--gripper-scale", type=float, default=1.0)
    ap.add_argument("--gripper-effort", type=float, default=1.0,
                    help="follower gripper max force, N·m (0-5, default 1.0)")
    args = ap.parse_args()

    # ---- arms ----
    specs = args.pair or ["right:can_leader_r:can_follower_r"]
    pairs = [parse_pair(s, speed=args.speed, start_speed=args.start_speed,
                        gripper_scale=args.gripper_scale, gripper=args.gripper,
                        gripper_effort_nm=args.gripper_effort) for s in specs]

    # ---- cameras ----
    cams = {}
    for i, c in enumerate(args.cam):
        name, serial = c.split("=", 1) if "=" in c else (f"cam{i}", c)
        cams[name] = RealSense(serial, fps=args.fps)
        print(f"camera '{name}' -> {serial}")

    # ---- dataset ----
    names = [f"{p.side}_{n}" for p in pairs for n in JOINT_NAMES]
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(names),), "names": names},
        "action": {"dtype": "float32", "shape": (len(names),), "names": names},
    }
    for name, cam in cams.items():
        features[f"observation.images.{name}"] = {
            "dtype": "video", "shape": cam.shape, "names": ["height", "width", "channels"]}
    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=args.fps, features=features, root=args.root,
                               robot_type="piper_x" + ("_bimanual" if len(pairs) > 1 else ""),
                               use_videos=bool(cams))

    # ---- go ----
    bridge = Bridge(pairs, hz=args.hz)
    bridge.start()
    print("\nTeleop running. Wait for 'synced', then: Enter = start/stop episode, r = discard, q = quit")
    if args.gripper == "keyboard":
        print(gripper_help(pairs))
    print()
    kb = KeyReader().__enter__()

    recording, n, ep, period = False, 0, 0, 1.0 / args.fps
    try:
        while True:
            t0 = time.perf_counter()

            for k in kb.keys():
                if handle_gripper_key(pairs, k):
                    continue
                if k == "\r":
                    k = "\n"
                if k == "q":
                    raise KeyboardInterrupt
                if k == "r" and recording:
                    ds_discard_episode(ds)
                    recording, n = False, 0
                    print("  episode discarded")
                elif k == "\n" and not recording:
                    if not bridge.all_synced:
                        print("  not synced yet: move the leader near the follower's pose and wait")
                        continue
                    recording, n = True, 0
                    print(f"* recording episode {ep} ...")
                elif k == "\n" and recording:
                    recording = False
                    if n > 0:
                        ds_save_episode(ds, args.task)
                        print(f"  saved episode {ep}: {n} frames ({n / args.fps:.1f} s)")
                        ep += 1

            if recording:
                if not bridge.all_synced:     # leader data dropped mid-episode -> episode is bad
                    ds_discard_episode(ds)
                    recording, n = False, 0
                    print("  leader data lost: episode discarded")
                    continue
                frame = {
                    "observation.state": np.array(sum((p.state() for p in pairs), []), dtype=np.float32),
                    "action": np.array(sum((p.action() for p in pairs), []), dtype=np.float32),
                }
                for name, cam in cams.items():
                    frame[f"observation.images.{name}"] = cam.read()
                ds_add_frame(ds, frame, args.task)
                n += 1

            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
            elif recording:
                print(f"  warning: frame took {dt * 1000:.0f} ms (> {period * 1000:.0f} ms)")
    except KeyboardInterrupt:
        pass
    finally:
        kb.__exit__(None, None, None)
        bridge.stop()
        for cam in cams.values():
            cam.stop()
        if hasattr(ds, "finalize"):
            ds.finalize()
        print(f"\nDone. {ep} episode(s) in {ds.root}. Followers are holding position.")


if __name__ == "__main__":
    main()
