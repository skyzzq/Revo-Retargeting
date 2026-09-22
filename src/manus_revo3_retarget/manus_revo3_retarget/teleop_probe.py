#!/usr/bin/env python3
"""Record teleop stage timing and physical state with low probe overhead.

Callbacks only append packed numbers. Analysis and disk I/O happen after stop,
so the script is less likely to create the 60-400 ms gaps it is measuring.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import rclpy
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from revo3_mit_controller_msgs.msg import Revo3MITCommand
from sensor_msgs.msg import JointState


VALID_HAND_MODES = {"left", "right", "both"}
JOINT_SUFFIXES = (
    "little_MPR_joint", "little_MCP_joint", "little_PIP_joint", "little_DIP_joint",
    "ring_MPR_joint", "ring_MCP_joint", "ring_PIP_joint", "ring_DIP_joint",
    "middle_MPR_joint", "middle_MCP_joint", "middle_PIP_joint", "middle_DIP_joint",
    "index_MPR_joint", "index_MCP_joint", "index_PIP_joint", "index_DIP_joint",
    "thumb_MCP_joint", "thumb_PIP_joint", "thumb_DIP_joint", "thumb_CMP_joint",
    "thumb_CMR_joint",
)
KEY_JOINTS = ("index_MCP_joint", "middle_MCP_joint", "index_MPR_joint", "thumb_MCP_joint")
ERGO_KEYS = ("IndexMCPStretch", "MiddleMCPStretch", "PinkyMCPStretch")
GAP_MS = 20.0


def joint_names(side: str) -> list[str]:
    return [f"{side}_{suffix}" for suffix in JOINT_SUFFIXES]


def _qos(depth: int) -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth)


def _header_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _pack_named(names, values, order: list[str], cache: dict) -> list[float]:
    key = id(names)
    index = cache.get(key)
    if index is None or len(index) != len(order):
        lookup = {str(name): i for i, name in enumerate(names)}
        index = [lookup.get(name, -1) for name in order]
        cache[key] = index
    out = [0.0] * len(order)
    n = len(values)
    for dst, src in enumerate(index):
        if 0 <= src < n:
            out[dst] = float(values[src])
    return out


def _stats(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(values)
    n = len(ordered)
    return {
        "n": n,
        "mean": sum(ordered) / n,
        "p50": ordered[n // 2],
        "p95": ordered[min(n - 1, int(n * 0.95))],
        "min": ordered[0],
        "max": ordered[-1],
    }


def _hz(recv: list[float]) -> dict[str, float]:
    if len(recv) < 2:
        return {"n": len(recv), "hz": 0.0, "dt_mean_ms": 0.0, "dt_max_ms": 0.0, "gaps_ge_20ms": 0}
    dts = [1000.0 * (recv[i] - recv[i - 1]) for i in range(1, len(recv))]
    mean = sum(dts) / len(dts)
    return {
        "n": len(recv),
        "hz": 0.0 if mean <= 0.0 else 1000.0 / mean,
        "dt_mean_ms": mean,
        "dt_min_ms": min(dts),
        "dt_max_ms": max(dts),
        "gaps_ge_20ms": sum(1 for dt in dts if dt >= GAP_MS),
        "dt_ms": _stats(dts),
    }


def _nearest_before(needles: list[float], haystack: list[float], hay_index: list[int]) -> list[int]:
    """For each needle time, index of last haystack time <= needle, else -1."""
    out = [-1] * len(needles)
    j = 0
    last = -1
    n_h = len(hay_index)
    for i, t in enumerate(needles):
        while j < n_h and haystack[hay_index[j]] <= t:
            last = hay_index[j]
            j += 1
        out[i] = last
    return out


class TeleopProbe(Node):
    def __init__(self, hand_mode: str, lite: bool):
        super().__init__("teleop_probe")
        self.hand_mode = hand_mode
        self.lite = lite
        self.sides = ["left", "right"] if hand_mode == "both" else [hand_mode]
        self.order = {side: joint_names(side) for side in self.sides}
        self.name_cache: dict[str, dict] = defaultdict(dict)
        self.t0 = time.monotonic()
        self.cpu0 = time.process_time()
        self.streams: dict[str, dict[str, list]] = {}
        self.spin_ns: list[int] = []

        glove_qos = _qos(64)
        cmd_qos = _qos(256)
        state_qos = _qos(256)
        self.create_subscription(ManusGlove, "/manus_glove_0", lambda m: self._on_glove(m), glove_qos)
        self.create_subscription(ManusGlove, "/manus_glove_1", lambda m: self._on_glove(m), glove_qos)
        for side in self.sides:
            self._init_stream(f"glove_{side}", ("recv", "cb_ns", "ergo"))
            self._init_stream(f"target_{side}", ("recv", "stamp", "cb_ns", "pos"))
            self._init_stream(f"cmd_{side}", ("recv", "stamp", "cb_ns", "pos", "vel"))
            self._init_stream(f"state_{side}", ("recv", "stamp", "cb_ns", "pos", "vel", "effort"))
            ns = f"/revo3_{side}"
            self.create_subscription(
                Revo3MITCommand,
                f"{ns}/joint_forward_mit_controller/retarget_targets",
                lambda m, s=side: self._on_mit(f"target_{s}", s, m),
                cmd_qos,
            )
            self.create_subscription(
                Revo3MITCommand,
                f"{ns}/joint_forward_mit_controller/commands",
                lambda m, s=side: self._on_mit(f"cmd_{s}", s, m),
                cmd_qos,
            )
            self.create_subscription(
                JointState,
                f"{ns}/revo3_joint_state/joint_states",
                lambda m, s=side: self._on_state(s, m),
                state_qos,
            )

    def _init_stream(self, name: str, fields: tuple[str, ...]) -> None:
        self.streams[name] = {field: [] for field in fields}

    def _now(self) -> float:
        return time.monotonic() - self.t0

    def _on_glove(self, msg: ManusGlove) -> None:
        t_cb = time.perf_counter_ns()
        side = str(msg.side).strip().lower()
        if side in ("l",):
            side = "left"
        elif side in ("r",):
            side = "right"
        if side not in self.sides:
            return
        ergo = [0.0] * len(ERGO_KEYS)
        wanted = {name: i for i, name in enumerate(ERGO_KEYS)}
        for item in msg.ergonomics:
            idx = wanted.get(item.type)
            if idx is not None:
                ergo[idx] = float(item.value)
        stream = self.streams[f"glove_{side}"]
        stream["recv"].append(self._now())
        stream["ergo"].append(ergo)
        stream["cb_ns"].append(time.perf_counter_ns() - t_cb)

    def _on_mit(self, stream_name: str, side: str, msg: Revo3MITCommand) -> None:
        t_cb = time.perf_counter_ns()
        stream = self.streams[stream_name]
        order = self.order[side]
        cache = self.name_cache[stream_name]
        stream["recv"].append(self._now())
        stream["stamp"].append(_header_s(msg.header.stamp))
        stream["pos"].append(_pack_named(msg.joint_names, msg.position, order, cache))
        if "vel" in stream:
            if self.lite or not msg.velocity:
                stream["vel"].append([])
            else:
                stream["vel"].append(_pack_named(msg.joint_names, msg.velocity, order, cache))
        stream["cb_ns"].append(time.perf_counter_ns() - t_cb)

    def _on_state(self, side: str, msg: JointState) -> None:
        t_cb = time.perf_counter_ns()
        stream = self.streams[f"state_{side}"]
        order = self.order[side]
        cache = self.name_cache[f"state_{side}"]
        stream["recv"].append(self._now())
        stream["stamp"].append(_header_s(msg.header.stamp))
        stream["pos"].append(_pack_named(msg.name, msg.position, order, cache))
        if self.lite:
            stream["vel"].append([])
            stream["effort"].append([])
        else:
            stream["vel"].append(_pack_named(msg.name, msg.velocity, order, cache) if msg.velocity else [])
            stream["effort"].append(_pack_named(msg.name, msg.effort, order, cache) if msg.effort else [])
        stream["cb_ns"].append(time.perf_counter_ns() - t_cb)


def _pair_latency_ms(later_recv: list[float], earlier_recv: list[float]) -> list[float]:
    if not later_recv or not earlier_recv:
        return []
    order = list(range(len(earlier_recv)))
    idxs = _nearest_before(later_recv, earlier_recv, order)
    out = []
    for i, src in enumerate(idxs):
        if src >= 0:
            out.append(1000.0 * (later_recv[i] - earlier_recv[src]))
    return out


def _tracking_deg(cmd_recv, cmd_pos, state_recv, state_pos, joint_index: int) -> list[float]:
    if not cmd_recv or not state_recv:
        return []
    order = list(range(len(cmd_recv)))
    idxs = _nearest_before(state_recv, cmd_recv, order)
    out = []
    for i, src in enumerate(idxs):
        if src < 0 or src >= len(cmd_pos) or i >= len(state_pos):
            continue
        if joint_index >= len(cmd_pos[src]) or joint_index >= len(state_pos[i]):
            continue
        out.append(abs(math.degrees(state_pos[i][joint_index] - cmd_pos[src][joint_index])))
    return out


def _step_deg(positions: list[list[float]], joint_index: int) -> list[float]:
    vals = [row[joint_index] for row in positions if joint_index < len(row)]
    if len(vals) < 2:
        return []
    return [abs(math.degrees(vals[i] - vals[i - 1])) for i in range(1, len(vals))]


def analyze(node: TeleopProbe, duration_s: float) -> dict:
    cpu_s = time.process_time() - node.cpu0
    wall_s = time.monotonic() - node.t0
    cb_all = []
    summary = {
        "duration_s": wall_s,
        "hand_mode": node.hand_mode,
        "lite": node.lite,
        "stages": {},
        "pipeline_ms": {},
        "tracking_deg": {},
        "command_step_deg": {},
        "probe_overhead": {},
    }
    for name, stream in node.streams.items():
        recv = stream["recv"]
        stage = _hz(recv)
        if stream.get("cb_ns"):
            cb_ms = [ns / 1e6 for ns in stream["cb_ns"]]
            cb_all.extend(cb_ms)
            stage["callback_ms"] = _stats(cb_ms)
        summary["stages"][name] = stage

    for side in node.sides:
        glove = node.streams[f"glove_{side}"]["recv"]
        target = node.streams[f"target_{side}"]["recv"]
        cmd = node.streams[f"cmd_{side}"]["recv"]
        state = node.streams[f"state_{side}"]["recv"]
        summary["pipeline_ms"][side] = {
            "glove_to_target": _stats(_pair_latency_ms(target, glove)),
            "target_to_cmd": _stats(_pair_latency_ms(cmd, target)),
            "cmd_to_state": _stats(_pair_latency_ms(state, cmd)),
            "glove_to_cmd": _stats(_pair_latency_ms(cmd, glove)),
        }
        cmd_pos = node.streams[f"cmd_{side}"]["pos"]
        state_pos = node.streams[f"state_{side}"]["pos"]
        track = {}
        steps = {}
        for suffix in KEY_JOINTS:
            idx = JOINT_SUFFIXES.index(suffix)
            track[suffix] = _stats(_tracking_deg(cmd, cmd_pos, state, state_pos, idx))
            steps[suffix] = _stats(_step_deg(cmd_pos, idx))
        summary["tracking_deg"][side] = track
        summary["command_step_deg"][side] = steps

    spin_ms = [ns / 1e6 for ns in node.spin_ns]
    cb_stats = _stats(cb_all)
    spin_stats = _stats(spin_ms)
    summary["probe_overhead"] = {
        "wall_s": wall_s,
        "process_cpu_s": cpu_s,
        "cpu_fraction": (cpu_s / wall_s) if wall_s > 0 else 0.0,
        "callback_ms": cb_stats,
        "spin_once_ms": spin_stats,
        "self_gap_risk": bool(spin_stats and spin_stats["p95"] >= 2.0),
        "note": (
            "Callbacks only append packed numbers. If spin_once p95 >= 2 ms or "
            "cpu_fraction is high, probe-induced gaps are possible."
        ),
    }
    summary["expected"] = {
        "glove_hz": 120.0,
        "target_hz": 120.0,
        "cmd_hz": 200.0,
        "state_hz": 200.0,
        "cmd_dt_max_ms": 20.0,
    }
    return summary


def _write_traces(out_dir: Path, node: TeleopProbe) -> None:
    for name, stream in node.streams.items():
        path = out_dir / f"{name}.jsonl"
        recv = stream["recv"]
        with path.open("w", encoding="utf-8") as handle:
            for i, t in enumerate(recv):
                row = {"t": t}
                if "stamp" in stream and i < len(stream["stamp"]):
                    row["stamp"] = stream["stamp"][i]
                if "cb_ns" in stream and i < len(stream["cb_ns"]):
                    row["cb_us"] = stream["cb_ns"][i] / 1000.0
                if "ergo" in stream and i < len(stream["ergo"]):
                    row["ergo"] = stream["ergo"][i]
                if "pos" in stream and i < len(stream["pos"]):
                    row["pos"] = stream["pos"][i]
                if "vel" in stream and i < len(stream["vel"]) and stream["vel"][i]:
                    row["vel"] = stream["vel"][i]
                if "effort" in stream and i < len(stream["effort"]) and stream["effort"][i]:
                    row["effort"] = stream["effort"][i]
                handle.write(json.dumps(row, separators=(",", ":")) + "\n")


def _print_summary(summary: dict) -> None:
    print("=== teleop probe ===")
    print(f"duration={summary['duration_s']:.2f}s  cpu={summary['probe_overhead']['cpu_fraction']:.2%}")
    print(f"{'stage':<16} {'n':>6} {'hz':>7} {'dt_mean':>8} {'dt_max':>8} {'gaps>=20':>8} {'cb_p95_us':>10}")
    for name, stage in summary["stages"].items():
        cb = stage.get("callback_ms") or {}
        cb_p95 = f"{cb['p95'] * 1000:.0f}" if cb else "-"
        print(
            f"{name:<16} {stage.get('n', 0):6d} {stage.get('hz', 0.0):7.1f} "
            f"{stage.get('dt_mean_ms', 0.0):8.2f} {stage.get('dt_max_ms', 0.0):8.2f} "
            f"{stage.get('gaps_ge_20ms', 0):8d} {cb_p95:>10}"
        )
    print("\n=== pipeline delay (recv pairing, ms) ===")
    for side, item in summary["pipeline_ms"].items():
        parts = []
        for key in ("glove_to_target", "target_to_cmd", "cmd_to_state", "glove_to_cmd"):
            stats = item.get(key)
            if stats:
                parts.append(f"{key} p50={stats['p50']:.2f} p95={stats['p95']:.2f} max={stats['max']:.2f}")
        print(f"  {side}: " + " | ".join(parts))
    print("\n=== |state-cmd| deg / cmd step deg ===")
    for side in summary["tracking_deg"]:
        print(f"  -- {side} --")
        for suffix in KEY_JOINTS:
            err = summary["tracking_deg"][side].get(suffix)
            step = summary["command_step_deg"][side].get(suffix)
            err_s = f"err p50={err['p50']:.2f} p95={err['p95']:.2f} max={err['max']:.2f}" if err else "err=none"
            step_s = f"step p95={step['p95']:.3f} max={step['max']:.3f}" if step else "step=none"
            print(f"    {suffix}: {err_s}  {step_s}")
    overhead = summary["probe_overhead"]
    spin = overhead.get("spin_once_ms") or {}
    print(
        "\n=== probe overhead ===\n"
        f"  cpu_fraction={overhead['cpu_fraction']:.2%}  "
        f"spin_p95={spin.get('p95', 0):.3f}ms  "
        f"self_gap_risk={overhead['self_gap_risk']}"
    )
    if overhead["self_gap_risk"]:
        print("  WARN: probe spin p95 >= 2 ms; long gaps may be partly self-inflicted.")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record teleop stage timing and Revo3 physical state.")
    parser.add_argument("--hand-mode", choices=sorted(VALID_HAND_MODES), default="both")
    parser.add_argument("--duration", type=float, default=10.0, help="Seconds to record.")
    parser.add_argument("--out", type=Path, default=None, help="Output directory.")
    parser.add_argument("--lite", action="store_true", help="Skip velocity/effort packing to cut probe CPU.")
    parser.add_argument("--no-traces", action="store_true", help="Write summary.json only, no jsonl traces.")
    args = parser.parse_args(remove_ros_args(argv)[1:])
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and > 0")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv if argv is None else argv)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out or Path("recordings") / "teleop_probe" / f"{args.hand_mode}_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=None)
    node = TeleopProbe(args.hand_mode, args.lite)
    print(
        f"[teleop_probe] recording {args.duration:.1f}s hand_mode={args.hand_mode} -> {out_dir}",
        flush=True,
    )
    end = time.monotonic() + float(args.duration)
    try:
        while rclpy.ok() and time.monotonic() < end:
            t0 = time.perf_counter_ns()
            rclpy.spin_once(node, timeout_sec=0.001)
            node.spin_ns.append(time.perf_counter_ns() - t0)
    except KeyboardInterrupt:
        pass

    summary = analyze(node, args.duration)
    summary["output_dir"] = str(out_dir)
    meta = {
        "created_local": datetime.now().isoformat(timespec="seconds"),
        "pid": os.getpid(),
        "hand_mode": args.hand_mode,
        "duration_requested_s": args.duration,
        "joint_suffixes": list(JOINT_SUFFIXES),
        "key_joints": list(KEY_JOINTS),
        "ergo_keys": list(ERGO_KEYS),
    }
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    if not args.no_traces:
        _write_traces(out_dir, node)
    _print_summary(summary)
    print(f"\n[teleop_probe] wrote {out_dir}", flush=True)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
