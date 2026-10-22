"""
Leader / follower wrappers around the AgileX piper_sdk.

Setup this code assumes (RPL lab):
  * every arm has its OWN USB-CAN adapter -> its own CAN interface
  * the laptop reads the leader and sends its pose to the follower as joint commands
  * the FOLLOWER is in normal mode (not teaching mode), so it accepts CAN commands

The LEADER can be in either of two modes; Leader() detects which automatically:
  "drag"    : teach button single-clicked (TEACHING_MODE + START_RECORDING). Joints are
              free to move by hand; we read joint feedback. WHAT THE LAB USES.
  "master"  : optional, set with tools/set_role.py <can> leader (MasterSlaveConfig 0xFA).
              The arm publishes its pose as control frames (CAN 0x155-0x157, 0x159).

By default the FOLLOWER gripper follows the leader's teaching pendant (gripper target
in CAN 0x159, master mode). The keyboard can drive it instead (TeleopPair gripper=...).
Gripper travel is 0-100 mm (large gripper, GRIPPER_MAX_UM).

SDK units: joints in 0.001 deg, gripper in 0.001 mm.
"""
import math
import time

from piper_sdk import C_PiperInterface_V2

MDEG_TO_RAD = math.pi / 180.0 / 1000.0
UM_TO_M = 1e-6
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]
STALE_AFTER_S = 0.5   # leader data older than this -> stop commanding follower
TEACHING_MODE = 0x02  # ArmMsgFeedbackStatusEnum.CtrlMode.TEACHING_MODE
START_RECORDING = 0x01
GRIPPER_MAX_UM = 100_000    # large Piper gripper: 0-100 mm
GRIPPER_STEP_UM = 10_000    # one '[' / ']' key press = 10 mm


def _fresh(ts):
    return ts > 1e9 and (time.time() - ts) < STALE_AFTER_S


def _connect(can_name, wait_for_ctrl=False, **kwargs):
    arm = C_PiperInterface_V2(can_name, **kwargs)
    arm.ConnectPort()
    t0 = time.time()
    while time.time() - t0 < 2.0:              # wait for the first frame
        if arm.GetArmJointMsgs().time_stamp > 0:
            return arm
        if wait_for_ctrl and arm.GetArmJointCtrl().time_stamp > 0:
            return arm
        time.sleep(0.05)
    raise RuntimeError(
        f"No data on {can_name}. Is the arm powered, and was scripts/setup_can.sh run?")


def _joints_mdeg(arm):
    js = arm.GetArmJointMsgs().joint_state
    return [getattr(js, f"joint_{i}") for i in range(1, 7)]


def _gripper_um(arm):
    return arm.GetArmGripperMsgs().gripper_state.grippers_angle


def _in_teaching_mode(arm):
    time.sleep(0.3)        # let a status frame arrive
    return int(arm.GetArmStatus().arm_status.ctrl_mode) == TEACHING_MODE


def to_si(joints_mdeg, gripper_um):
    """SDK units -> [6 joints in rad, gripper in m]."""
    return [j * MDEG_TO_RAD for j in joints_mdeg] + [gripper_um * UM_TO_M]


class Leader:
    """Hand-moved arm. Read-only: we never command it."""

    def __init__(self, can_name, wait_s=60):
        self.can = can_name
        self.arm = C_PiperInterface_V2(can_name)
        self.arm.ConnectPort()
        # A leader set as Teaching Input Arm (master) only transmits while it is MOVED,
        # so wait for the first frame and ask the user to move it.
        t0, last_hint = time.time(), 0.0
        while (self.arm.GetArmJointCtrl().time_stamp == 0
               and self.arm.GetArmJointMsgs().time_stamp == 0):
            if time.time() - t0 > wait_s:
                raise RuntimeError(
                    f"No data from the leader on {can_name} after {wait_s}s.\n"
                    f"  Move the leader by hand (as Teaching Input Arm it only sends while moving).\n"
                    f"  Check: arm powered, adapter in the right USB port, scripts/setup_can.sh run.")
            if time.time() - last_hint > 3:
                print(f"[leader {can_name}] waiting for data: MOVE THE LEADER A LITTLE ...")
                last_hint = time.time()
            time.sleep(0.05)
        time.sleep(0.3)
        if self.arm.GetArmJointCtrl().time_stamp > 0:
            self.mode = "master"
            print(f"[leader {can_name}] Teaching Input Arm (master) mode: reading its control frames")
        else:
            self.mode = "drag"
            st = self.arm.GetArmStatus().arm_status
            if int(st.ctrl_mode) != TEACHING_MODE or int(st.teach_status) != START_RECORDING:
                print(f"WARNING: leader on {can_name} is not in drag mode "
                      f"(mode {st.ctrl_mode}, teach {st.teach_status}), so it may be stiff.\n"
                      f"         SINGLE-click its teach button (never double-click: that replays a trajectory).")
            else:
                print(f"[leader {can_name}] drag mode: joints by hand")

    def age(self):
        """Seconds since the leader last sent data."""
        ts = (self.arm.GetArmJointCtrl() if self.mode == "master" else self.arm.GetArmJointMsgs()).time_stamp
        return time.time() - ts if ts > 1e9 else 0.0

    def is_fresh(self):
        if self.mode == "master":
            # No new frames just means the leader is held still: the follower keeps
            # its last commanded pose, which is also the safe outcome if the cable is pulled.
            return True
        if not self.arm.isOk():
            return False
        ts = self.arm.GetArmJointMsgs().time_stamp
        return True if ts < 1e9 else _fresh(ts)

    def read_raw(self):
        if self.mode == "master":
            jc = self.arm.GetArmJointCtrl().joint_ctrl
            g = self.arm.GetArmGripperCtrl().gripper_ctrl.grippers_angle
            return [getattr(jc, f"joint_{i}") for i in range(1, 7)], g
        return _joints_mdeg(self.arm), _gripper_um(self.arm)

    def read(self):
        return to_si(*self.read_raw())

    def has_gripper(self):
        """True once a gripper reading has arrived (0x159 in master mode, 0x2A8 in drag mode)."""
        msg = self.arm.GetArmGripperCtrl() if self.mode == "master" else self.arm.GetArmGripperMsgs()
        return msg.time_stamp > 0


class Follower:
    """Motor-driven arm that copies the leader."""

    def __init__(self, can_name, gripper_effort=1000):     # gripper_effort: 0.001 N·m, 0-5000
        self.can = can_name
        # SDK-side joint/gripper limits clip commands to the arm's safe range
        self.arm = _connect(can_name, wait_for_ctrl=True,
                            start_sdk_joint_limit=True, start_sdk_gripper_limit=True)
        # piper_sdk clips gripper commands AND feedback to 0-70 mm by default; we use 0-100 mm
        self.arm.SetSDKGripperRangeParam(0.0, GRIPPER_MAX_UM * UM_TO_M)
        self.gripper_effort = gripper_effort
        time.sleep(0.3)
        if _fresh(self.arm.GetArmJointCtrl().time_stamp):
            raise RuntimeError(
                f"Arm on {can_name} (follower) is sending leader/master control frames.\n"
                f"  * The USB adapters are probably swapped (this is the leader): check the port map.\n"
                f"  * If it really is the follower: python3 tools/set_role.py {can_name} follower")
        if _in_teaching_mode(self.arm):
            # Refuse to enable: either the follower is still in teaching mode, or the
            # adapters are swapped and this is actually the LEADER (enabling it would lock it).
            raise RuntimeError(
                f"Arm on {can_name} (follower) is in TEACHING MODE.\n"
                f"  * If this really is the follower: short-press its teach button to exit, then retry.\n"
                f"  * If this is the leader: the USB adapters are swapped. Check the port map,\n"
                f"    re-plug, and run scripts/setup_can.sh again.\n"
                f"Run: python3 tools/arm_status.py {can_name}")

    def enable(self, timeout=5.0):
        t0 = time.time()
        while not self.arm.EnablePiper():
            if time.time() - t0 > timeout:
                raise RuntimeError(
                    f"Follower on {self.can} did not enable. Check e-stop and power.")
            time.sleep(0.01)
        # gripper: clear any error, then enable (same sequence as AgileX's piper_ctrl_gripper.py)
        self.arm.GripperCtrl(0, self.gripper_effort, 0x02, 0)
        time.sleep(0.2)
        _, g = self.read_raw()
        self.arm.GripperCtrl(max(0, int(g)), self.gripper_effort, 0x01, 0)

    def disable(self):
        """WARNING: the arm goes limp and falls under gravity. Support it first."""
        while self.arm.DisablePiper():
            time.sleep(0.01)

    def command(self, joints_mdeg, gripper_um, speed_pct):
        # ctrl_mode 0x01 = CAN command, move_mode 0x01 = MOVE J (joint space)
        self.arm.MotionCtrl_2(0x01, 0x01, int(speed_pct), 0x00)
        self.arm.JointCtrl(*[int(j) for j in joints_mdeg])
        self.arm.GripperCtrl(abs(int(gripper_um)), self.gripper_effort, 0x01, 0)

    def read_raw(self):
        return _joints_mdeg(self.arm), _gripper_um(self.arm)

    def read(self):
        return to_si(*self.read_raw())


class TeleopPair:
    """
    One leader -> one follower.

    Joints always come from the leader. The gripper comes from:
      gripper="leader"   : follower copies the leader's teaching pendant (default), x gripper_scale
      gripper="keyboard" : keys open/close the follower gripper

    Starts in a slow 'sync' phase: the follower moves at start_speed until it is
    within sync_tol_deg of the leader on every joint, then switches to full speed.
    This stops the follower from jumping if the arms start in different poses.
    """

    def __init__(self, side, leader_can, follower_can, speed=60, start_speed=15,
                 sync_tol_deg=5.0, gripper_scale=1.0, gripper="leader", gripper_effort_nm=1.0):
        if gripper not in ("keyboard", "leader"):
            raise ValueError("gripper must be 'keyboard' or 'leader'")
        self.side = side
        self.leader = Leader(leader_can)
        if not 0 <= gripper_effort_nm <= 5:
            raise ValueError("gripper effort must be 0-5 N·m")
        self.follower = Follower(follower_can, gripper_effort=int(gripper_effort_nm * 1000))
        self.speed, self.start_speed = speed, start_speed
        self.sync_tol_mdeg = sync_tol_deg * 1000
        self.gripper_scale = gripper_scale
        self.gripper_source = gripper
        self.gripper_cmd_um = 0
        self.synced = False
        self.paused = False      # True while leader data is stale

    def start(self):
        _, g = self.follower.read_raw()
        self.gripper_cmd_um = max(0, g)          # keyboard mode: hold current opening
        self.follower.enable()
        print(f"[{self.side}] leader {self.leader.can} -> follower {self.follower.can}, "
              f"gripper from {self.gripper_source}. Syncing slowly ...")

    # gripper (keyboard mode) ----------------------------------------
    def set_gripper(self, um):
        self.gripper_cmd_um = int(min(GRIPPER_MAX_UM, max(0, um)))

    def gripper_open(self):
        self.set_gripper(GRIPPER_MAX_UM)

    def gripper_close(self):
        self.set_gripper(0)

    def gripper_step(self, direction):
        self.set_gripper(self.gripper_cmd_um + direction * GRIPPER_STEP_UM)

    def _gripper_target(self, leader_gripper_um):
        if self.gripper_source == "leader":
            if not self.leader.has_gripper():
                return self.gripper_cmd_um       # no pendant reading yet: hold current opening
            return min(GRIPPER_MAX_UM, max(0, abs(leader_gripper_um) * self.gripper_scale))
        return self.gripper_cmd_um

    # control loop ----------------------------------------------------
    def step(self):
        """Call at a fixed rate (e.g. 100-200 Hz). Returns True if a command was sent."""
        if not self.leader.is_fresh():
            if not self.paused:
                print(f"[{self.side}] leader data stale -> follower holding position")
            self.paused = True
            return False
        if self.paused:
            print(f"[{self.side}] leader data back -> re-syncing slowly")
            self.paused, self.synced = False, False

        lj, lg = self.leader.read_raw()
        if not self.synced:
            fj, _ = self.follower.read_raw()
            err = max(abs(a - b) for a, b in zip(lj, fj))
            if err < self.sync_tol_mdeg:
                self.synced = True
                print(f"[{self.side}] synced -> full speed ({self.speed}%)")
        speed = self.speed if self.synced else self.start_speed
        self.follower.command(lj, self._gripper_target(lg), speed)
        return True

    # for recording -------------------------------------------------
    def action(self):
        """What the follower was told to do: leader joints + gripper command (SI units)."""
        j, g = self.leader.read_raw()
        return to_si(j, self._gripper_target(g))

    def state(self):
        """What the follower actually did (SI units)."""
        return self.follower.read()


class Bridge:
    """
    Runs leader -> follower forwarding for all pairs in a background thread at a
    fixed rate, independent of camera / recording speed (which is much slower).
    """

    def __init__(self, pairs, hz=200):
        import threading
        self.pairs, self.period = pairs, 1.0 / hz
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            t0 = time.perf_counter()
            for p in self.pairs:
                p.step()
            dt = time.perf_counter() - t0
            if dt < self.period:
                time.sleep(self.period - dt)

    def start(self):
        for p in self.pairs:
            p.start()
        self._thread.start()

    def stop(self):
        """Stops sending commands. Followers HOLD their last position (stay enabled)."""
        self._stop.set()
        self._thread.join(timeout=1.0)

    @property
    def all_synced(self):
        return all(p.synced and not p.paused for p in self.pairs)


def parse_pair(spec, **kw):
    """'right:can_leader_r:can_follower_r' -> TeleopPair"""
    parts = spec.split(":")
    if len(parts) != 3:
        raise ValueError(f"--pair must be side:leader_can:follower_can, got '{spec}'")
    return TeleopPair(parts[0], parts[1], parts[2], **kw)
