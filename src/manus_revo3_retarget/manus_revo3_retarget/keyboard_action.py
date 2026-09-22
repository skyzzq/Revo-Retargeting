#!/usr/bin/env python3
"""Publish named Revo3 pose commands from a teleop terminal keyboard."""
from __future__ import annotations

import argparse
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


VALID_HAND_MODES = {"left", "right", "both"}
DEFAULT_TOPIC = "/manus_revo3_retarget/action_command"
RESUME_ACTIONS = {"glove", "none", "clear", "resume"}

KEY_ACTIONS = {
    "1": "open",
    "o": "open",
    "2": "fist",
    "f": "fist",
    "3": "pinch",
    "p": "pinch",
    "4": "point",
    "i": "point",
    "5": "ok",
    "k": "ok",
    "0": "glove",
    "g": "glove",
    " ": "glove",
    "\x1b": "glove",
}

HELP_TEXT = """
按键动作（覆盖手套映射，平滑过渡到预设姿态）
  1 / o    张开  open
  2 / f    握拳  fist
  3 / p    捏合  pinch
  4 / i    指向  point
  5 / k    OK    ok
  0 / g / 空格 / Esc    回到手套遥操
  h / ?    再打印本说明
  q / Ctrl-C            退出按键节点
再按一次当前动作键，也会回到手套遥操。
也可用: ros2 topic pub --once /manus_revo3_retarget/action_command std_msgs/String "{{data: fist}}"
侧向指令: left:fist  /  right:open  /  both:glove
""".strip()


class KeyboardActionNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("revo3_keyboard_action")
        self.hand_mode = args.hand_mode
        self.topic = args.action_command_topic.strip() or DEFAULT_TOPIC
        self.publisher = self.create_publisher(String, self.topic, 10)
        self.active_action = "glove"
        self.get_logger().info(
            f"Keyboard actions -> {self.topic} (hand_mode={self.hand_mode})"
        )

    def publish_action(self, action: str) -> None:
        action = action.strip().lower()
        if not action:
            return
        if ":" not in action and self.hand_mode in ("left", "right"):
            payload = f"{self.hand_mode}:{action}"
        else:
            payload = action
        msg = String()
        msg.data = payload
        self.publisher.publish(msg)
        self.active_action = action.split(":")[-1]
        self.get_logger().info(f"Sent action command: {payload}")
        print(f"[keyboard_action] {payload}", flush=True)

    def handle_key(self, key: str) -> bool:
        if key in ("q", "Q", "\x03"):
            return False
        if key in ("h", "H", "?"):
            print(HELP_TEXT, flush=True)
            return True
        action = KEY_ACTIONS.get(key)
        if action is None:
            return True
        if action not in RESUME_ACTIONS and action == self.active_action:
            action = "glove"
        self.publish_action(action)
        return True


def _read_key(timeout_s: float) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not ready:
        return None
    return sys.stdin.read(1)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send named Revo3 pose commands from the keyboard.")
    parser.add_argument("--hand-mode", choices=sorted(VALID_HAND_MODES), default="both")
    parser.add_argument("--action-command-topic", default=DEFAULT_TOPIC)
    parser.add_argument("--print-help-only", action="store_true")
    args = parser.parse_args(remove_ros_args(argv)[1:])
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv if argv is None else argv)
    if args.print_help_only:
        print(HELP_TEXT)
        return 0
    if not sys.stdin.isatty():
        print("[keyboard_action] stdin is not a TTY; run this node in a real terminal.", file=sys.stderr)
        return 1

    rclpy.init(args=None)
    node = KeyboardActionNode(args)
    settings = termios.tcgetattr(sys.stdin)
    print(HELP_TEXT, flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key(0.05)
            if key is None:
                continue
            if not node.handle_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
