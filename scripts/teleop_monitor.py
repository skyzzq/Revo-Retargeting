#!/usr/bin/env python3
"""Low-overhead teleop watcher. Best-effort, depth-1, no controller_manager calls."""
from __future__ import annotations

import argparse
import math
import signal
import time

import rclpy
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from revo3_mit_controller_msgs.msg import Revo3MITCommand
from sensor_msgs.msg import JointState

GLOVE_TOPIC = {"left": "/manus_glove_0", "right": "/manus_glove_1"}
WATCH = ("index_MCP_joint", "thumb_MCP_joint", "little_MCP_joint")
LABEL = ("iMCP", "tMCP", "lMCP")

# Compatible with RELIABLE publishers; does not request history from them.
SAMPLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _suffix_index(names) -> tuple[int, int, int]:
    lookup = {}
    for i, name in enumerate(names):
        text = str(name)
        cut = text.find("_")
        lookup[text[cut + 1 :] if cut >= 0 else text] = i
    return tuple(lookup.get(key, -1) for key in WATCH)  # type: ignore[return-value]


def _pick(values, index: tuple[int, int, int] | None) -> tuple[float | None, float | None, float | None]:
    if index is None:
        return (None, None, None)
    n = len(values)
    out = []
    for src in index:
        out.append(float(values[src]) if 0 <= src < n else None)
    return out[0], out[1], out[2]


def _deg(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "--"
    return f"{value * 180.0 / math.pi:.0f}"


class SideBuf:
    __slots__ = (
        "n_glove",
        "n_cmd",
        "n_state",
        "last_glove",
        "last_cmd",
        "last_state",
        "cmd_idx",
        "state_idx",
        "cmd",
        "state",
        "warn_state",
        "warn_glove",
        "warn_cmd",
    )

    def __init__(self) -> None:
        self.n_glove = 0
        self.n_cmd = 0
        self.n_state = 0
        self.last_glove = 0.0
        self.last_cmd = 0.0
        self.last_state = 0.0
        self.cmd_idx: tuple[int, int, int] | None = None
        self.state_idx: tuple[int, int, int] | None = None
        self.cmd = (None, None, None)
        self.state = (None, None, None)
        self.warn_state = False
        self.warn_glove = False
        self.warn_cmd = False


class TeleopMonitor(Node):
    def __init__(self, hand_mode: str, period_s: float) -> None:
        super().__init__("teleop_monitor")
        self.sides = ["left", "right"] if hand_mode == "both" else [hand_mode]
        self.buf = {side: SideBuf() for side in self.sides}
        self._prev = time.monotonic()
        for side in self.sides:
            self.create_subscription(ManusGlove, GLOVE_TOPIC[side], self._on_glove(side), SAMPLE_QOS)
            ns = f"/revo3_{side}"
            self.create_subscription(
                Revo3MITCommand,
                f"{ns}/joint_forward_mit_controller/commands",
                self._on_cmd(side),
                SAMPLE_QOS,
            )
            self.create_subscription(
                JointState,
                f"{ns}/revo3_joint_state/joint_states",
                self._on_state(side),
                SAMPLE_QOS,
            )
        self.create_timer(period_s, self._tick)
        print(f"[teleop_mon] {','.join(self.sides)} every {period_s:.0f}s (best-effort, no CM)", flush=True)

    def _on_glove(self, side: str):
        buf = self.buf[side]

        def _cb(_msg: ManusGlove) -> None:
            buf.n_glove += 1
            buf.last_glove = time.monotonic()

        return _cb

    def _on_cmd(self, side: str):
        buf = self.buf[side]

        def _cb(msg: Revo3MITCommand) -> None:
            buf.n_cmd += 1
            buf.last_cmd = time.monotonic()
            if buf.cmd_idx is None:
                buf.cmd_idx = _suffix_index(msg.joint_names)
            buf.cmd = _pick(msg.position, buf.cmd_idx)

        return _cb

    def _on_state(self, side: str):
        buf = self.buf[side]

        def _cb(msg: JointState) -> None:
            buf.n_state += 1
            buf.last_state = time.monotonic()
            if buf.state_idx is None:
                buf.state_idx = _suffix_index(msg.name)
            buf.state = _pick(msg.position, buf.state_idx)

        return _cb

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._prev)
        self._prev = now
        for side in self.sides:
            buf = self.buf[side]
            glove_hz = buf.n_glove / dt
            cmd_hz = buf.n_cmd / dt
            state_hz = buf.n_state / dt
            buf.n_glove = buf.n_cmd = buf.n_state = 0
            joints = " ".join(
                f"{lab} {_deg(c)}/{_deg(s)}" for lab, c, s in zip(LABEL, buf.cmd, buf.state)
            )
            print(
                f"[teleop_mon] {side[0].upper()} "
                f"glove {glove_hz:4.0f}Hz  cmd {cmd_hz:4.0f}Hz  state {state_hz:4.0f}Hz  {joints}",
                flush=True,
            )
            self._edge_warn(side, buf, now)

    def _edge_warn(self, side: str, buf: SideBuf, now: float) -> None:
        tag = side[0].upper()
        state_silent = buf.last_state == 0.0 or now - buf.last_state > 0.4
        glove_silent = buf.last_glove == 0.0 or now - buf.last_glove > 0.5
        cmd_silent = buf.last_cmd == 0.0 or now - buf.last_cmd > 0.2
        if state_silent != buf.warn_state:
            buf.warn_state = state_silent
            if state_silent:
                print(f"[teleop_mon] WARN {tag} joint_state silent (bus?)", flush=True)
        if glove_silent != buf.warn_glove:
            buf.warn_glove = glove_silent
            if glove_silent:
                print(f"[teleop_mon] WARN {tag} glove silent", flush=True)
        if cmd_silent != buf.warn_cmd:
            buf.warn_cmd = cmd_silent
            if cmd_silent:
                print(f"[teleop_mon] WARN {tag} command silent", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand-mode", default="both", choices=("left", "right", "both"))
    parser.add_argument("--period", type=float, default=2.0)
    args = parser.parse_args()

    def _stop(_signum=None, _frame=None):
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    rclpy.init()
    node = TeleopMonitor(args.hand_mode, max(1.0, args.period))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
