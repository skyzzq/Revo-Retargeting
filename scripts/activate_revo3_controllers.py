#!/usr/bin/env python3
"""Load + configure + activate Revo3 joint_state and MIT controllers.

Spawners often time out under CM load. This helper can load missing controllers
via controller_manager services and finish configure/activate.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


NEEDED = ("revo3_joint_state", "joint_forward_mit_controller")


def call(node: Node, cli, req, timeout: float, wait_service: float = 3.0):
    if not cli.wait_for_service(timeout_sec=min(timeout, wait_service)):
        raise RuntimeError(f"service unavailable: {cli.srv_name}")
    fut = cli.call_async(req)
    deadline = time.time() + timeout
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    raise TimeoutError(cli.srv_name)


def list_names(node: Node, list_cli, timeout: float) -> dict[str, str]:
    resp = call(node, list_cli, ListControllers.Request(), min(20.0, timeout), wait_service=5.0)
    return {c.name: c.state for c in resp.controller}


def ensure_side(node: Node, side: str, timeout: float, wait_loaded: float, poll_s: float) -> bool:
    ns = f"/revo3_{side}/controller_manager"
    cg = ReentrantCallbackGroup()
    list_cli = node.create_client(ListControllers, f"{ns}/list_controllers", callback_group=cg)
    load_cli = node.create_client(LoadController, f"{ns}/load_controller", callback_group=cg)
    conf_cli = node.create_client(ConfigureController, f"{ns}/configure_controller", callback_group=cg)
    sw_cli = node.create_client(SwitchController, f"{ns}/switch_controller", callback_group=cg)

    deadline = time.time() + wait_loaded
    names: dict[str, str] = {}
    load_attempted: set[str] = set()

    while time.time() < deadline:
        try:
            names = list_names(node, list_cli, timeout)
            needed_states = {k: names[k] for k in NEEDED if k in names}
            if needed_states:
                print(f"[{side}] loaded={{{', '.join(f'{k}={v}' for k, v in needed_states.items())}}}")

            missing = [n for n in NEEDED if n not in names]
            if missing:
                for name in missing:
                    if name in load_attempted:
                        continue
                    # Spawner may have died; load directly once CM responds.
                    req = LoadController.Request()
                    req.name = name
                    try:
                        loaded = call(node, load_cli, req, timeout, wait_service=5.0)
                        print(f"[{side}] load {name} -> ok={loaded.ok}")
                        load_attempted.add(name)
                    except Exception as exc:  # noqa: BLE001
                        print(f"[{side}] load {name} pending: {exc}")
                time.sleep(poll_s)
                continue

            if all(n in names for n in NEEDED):
                break
        except Exception as exc:  # noqa: BLE001 - retry until wait_loaded
            print(f"[{side}] waiting for controller_manager: {exc}")
        time.sleep(poll_s)
    else:
        print(
            f"[ERROR] {side}: required controllers never appeared: {NEEDED}; have={list(names)}",
            file=sys.stderr,
        )
        return False

    if all(names.get(n) == "active" for n in NEEDED):
        print(f"[{side}] FINAL:")
        for n in NEEDED:
            print(f"  {n}: active")
        return True

    to_configure = [n for n in NEEDED if names.get(n) == "unconfigured"]
    for name in to_configure:
        req = ConfigureController.Request()
        req.name = name
        conf = call(node, conf_cli, req, timeout, wait_service=5.0)
        print(f"[{side}] configure {name} -> ok={conf.ok}")
        if not conf.ok:
            return False

    names = list_names(node, list_cli, timeout)
    to_activate = [n for n in NEEDED if names.get(n) != "active"]
    if to_activate:
        # Activate one-by-one: joint_state first, then MIT (avoids STRICT conflicts).
        for name in [n for n in NEEDED if n in to_activate]:
            req = SwitchController.Request()
            req.activate_controllers = [name]
            req.deactivate_controllers = []
            req.strictness = SwitchController.Request.BEST_EFFORT
            req.activate_asap = True
            req.timeout.sec = int(max(1.0, min(15.0, timeout / 2.0)))
            sw = call(node, sw_cli, req, timeout, wait_service=5.0)
            print(f"[{side}] activate [{name}] -> ok={sw.ok}")
            if not sw.ok:
                return False

    names = list_names(node, list_cli, timeout)
    print(f"[{side}] FINAL:")
    ok = True
    for name in NEEDED:
        state = names.get(name, "missing")
        print(f"  {name}: {state}")
        if state != "active":
            ok = False
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("left", "right", "both"), help="which hand(s)")
    parser.add_argument("--timeout", type=float, default=45.0, help="per-service timeout seconds")
    parser.add_argument(
        "--wait-loaded",
        type=float,
        default=120.0,
        help="seconds to wait for controllers to become loadable",
    )
    parser.add_argument(
        "--poll",
        type=float,
        default=0.5,
        help="seconds between list_controllers polls while waiting",
    )
    args = parser.parse_args()
    sides = ("left", "right") if args.mode == "both" else (args.mode,)

    rclpy.init()
    node = Node(f"activate_revo3_controllers_{int(time.time())}")
    try:
        all_ok = True
        for side in sides:
            if not ensure_side(node, side, args.timeout, args.wait_loaded, args.poll):
                all_ok = False
        return 0 if all_ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
