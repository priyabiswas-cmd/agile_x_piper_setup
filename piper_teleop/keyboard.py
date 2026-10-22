"""Single-key terminal input (no Enter needed). Ctrl+C still works."""
import os
import queue
import select
import sys
import termios
import threading
import tty


class KeyReader:
    def __init__(self):
        self.q = queue.Queue()
        self._fd = sys.stdin.fileno()
        self._old = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def __enter__(self):
        if os.isatty(self._fd):
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)          # keys arrive immediately, Ctrl+C still raises
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def _run(self):
        while not self._stop.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                ch = sys.stdin.read(1)
                if ch:
                    self.q.put(ch)

    def keys(self):
        """All keys pressed since the last call."""
        out = []
        while not self.q.empty():
            out.append(self.q.get())
        return out


# gripper keys per pair, in --pair order:  (open, close, close a bit, open a bit)
GRIPPER_KEYS = [("o", "c", "[", "]"),     # 1st pair (right)
                ("u", "n", "{", "}")]     # 2nd pair (left)


def gripper_help(pairs):
    return "   ".join(f"[{p.side} gripper] {k[0]}=open {k[1]}=close {k[2]}/{k[3]}=-/+10mm"
                      for p, k in zip(pairs, GRIPPER_KEYS) if p.gripper_source == "keyboard")


def handle_gripper_key(pairs, ch):
    """Apply a gripper key to the matching pair. Returns True if the key was a gripper key."""
    for p, (k_open, k_close, k_less, k_more) in zip(pairs, GRIPPER_KEYS):
        if p.gripper_source != "keyboard":
            continue
        if ch == k_open:
            p.gripper_open()
        elif ch == k_close:
            p.gripper_close()
        elif ch == k_less:
            p.gripper_step(-1)
        elif ch == k_more:
            p.gripper_step(+1)
        else:
            continue
        print(f"  [{p.side}] gripper -> {p.gripper_cmd_um / 1000:.0f} mm")
        return True
    return False
