#!/usr/bin/env python3
"""
Find out which CAN interface is which arm: move ONE arm at a time by hand.
Listen-only (python-can, no piper_sdk): sends nothing to any arm. Ctrl+C for a summary.

    python3 tools/which_arm.py can_leader_l can_follower_l can_leader_r can_follower_r

Prints a short line whenever an arm moves, e.g.
    can_leader_r    MOVING  12.3 deg  (leader ctrl)
"leader ctrl" = leader set as Teaching Input Arm (0x155-0x157, only sent while it moves).
"feedback"    = arm in normal mode (0x2A5-0x2A7, sent all the time), e.g. a follower.
"""
import struct
import sys
import time

import can

chans = sys.argv[1:] or ["can0", "can1"]
buses = {ch: can.interface.Bus(channel=ch, interface="socketcan") for ch in chans}
IDS = {0x2A5: (0, "feedback"), 0x2A6: (2, "feedback"), 0x2A7: (4, "feedback"),
       0x155: (0, "leader ctrl"), 0x156: (2, "leader ctrl"), 0x157: (4, "leader ctrl")}
MOVE_DEG = 1.0          # report when any joint is this far from where it was last reported

joints = {ch: None for ch in chans}
kind = {ch: "-" for ch in chans}
seen = {ch: set() for ch in chans}      # every frame type seen (a bus can carry both)
ref = {ch: None for ch in chans}           # pose at last report
peak = {ch: 0.0 for ch in chans}           # largest movement seen (deg)
frames = {ch: 0 for ch in chans}
w = max(len(c) for c in chans)

print("Listening (nothing is sent). Move ONE arm at a time. Ctrl+C for the summary.\n")
t_start = time.time()
try:
    while True:
        for ch, bus in buses.items():
            while (m := bus.recv(0)) is not None:
                frames[ch] += 1
                if m.arbitration_id not in IDS or len(m.data) != 8:
                    continue
                i, k = IDS[m.arbitration_id]
                if joints[ch] is None:
                    joints[ch] = [0.0] * 6
                a, b = struct.unpack(">ii", bytes(m.data))
                joints[ch][i:i + 2] = [a / 1000, b / 1000]
                kind[ch] = k
                seen[ch].add(k)
        for ch in chans:
            if joints[ch] is None:
                continue
            if ref[ch] is None:
                ref[ch] = list(joints[ch])
                continue
            d = max(abs(x - y) for x, y in zip(joints[ch], ref[ch]))
            if d >= MOVE_DEG:
                peak[ch] = max(peak[ch], d)
                print(f"{ch:>{w}}  MOVING {d:6.1f} deg  ({kind[ch]})", flush=True)
                ref[ch] = list(joints[ch])
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    for bus in buses.values():
        bus.shutdown()

secs = max(time.time() - t_start, 1e-3)
print(f"\n{'interface':<{w}}  {'frames/s':>8}  {'sends':<12}  moved?")
for ch in chans:
    if seen[ch] == {"leader ctrl", "feedback"}:
        role = "BOTH leader ctrl + feedback -> two arms on one bus? check cabling!"
    else:
        role = {"leader ctrl": "LEADER (Teaching Input Arm)", "feedback": "normal mode (follower?)"}.get(kind[ch], "nothing received")
    sends = "+".join(sorted(seen[ch])) or "-"
    print(f"{ch:<{w}}  {frames[ch] / secs:8.0f}  {sends:<12}  {'yes' if peak[ch] else 'no '}   -> {role}")
