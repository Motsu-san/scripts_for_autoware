#!/usr/bin/env python3
"""
Print per-stream dt between consecutive /diagnostics messages (by header.stamp).

EKF localizer (diagnostic_updater) uses names such as:
  - ekf_localizer: localization: ekf_localizer          -> main
  - ekf_localizer: localization: ekf_localizer: callback_pose
  - ekf_localizer: localization: ekf_localizer: callback_twist

Stamp is simulation time when use_sim_time is true; rosbag --rate does not stretch
these intervals.

Usage:
  source /opt/ros/$ROS_DISTRO/setup.bash
  # (optional) source your workspace install/setup.bash
  python3 check_ekf_diagnostics_stamp_delta.py
  python3 check_ekf_diagnostics_stamp_delta.py --topic /diagnostics
  python3 check_ekf_diagnostics_stamp_delta.py --print-main-level --diag-array-hz
  # tee 用: EKF の debug inject を数秒おきに交互に param set(別スレッド)
  python3 check_ekf_diagnostics_stamp_delta.py --print-main-level --diag-array-hz \\
    --inject-auto-toggle-node /localization/pose_twist_fusion_filter/ekf_localizer \\
    --inject-toggle-interval-sec 5 --inject-toggle-sets 8 2>&1 | tee ekf_diag_toggle.log
  # 上に加え、各 ros2 param set の経過時間(elapsed)と /parameter_events の反映時刻:
  python3 check_ekf_diagnostics_stamp_delta.py --print-main-level --diag-array-hz \\
    --inject-auto-toggle-node /localization/pose_twist_fusion_filter/ekf_localizer \\
    --inject-log-parameter-events
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import threading
import time
from typing import Callable, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rclpy.node import Node
from rclpy.qos import QoSProfile


LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def diagnostic_level_to_int(level: object) -> int:
    """Normalize DiagnosticStatus.level (int or single-byte bytes) to int."""
    if isinstance(level, (bytes, bytearray)):
        return int(level[0]) if len(level) > 0 else 0
    return int(level)


def main_diag_status_name(ekf_short_name: str = "ekf_localizer") -> str:
    return f"localization: {ekf_short_name}"


def normalize_node_name(n: str) -> str:
    s = (n or "").strip()
    if not s:
        return s
    return s if s.startswith("/") else f"/{s}"


def format_parameter_value(value_msg: object) -> str:
    """Short string for rcl_interfaces/ParameterValue (Parameter.msg value field)."""
    t = int(value_msg.type)
    if t == ParameterType.PARAMETER_BOOL:
        return f"bool={value_msg.bool_value}"
    if t == ParameterType.PARAMETER_INTEGER:
        return f"int={value_msg.integer_value}"
    if t == ParameterType.PARAMETER_DOUBLE:
        return f"double={value_msg.double_value}"
    if t == ParameterType.PARAMETER_STRING:
        return f"string={value_msg.string_value!r}"
    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        return f"bytes(len={len(value_msg.byte_array_value)})"
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return f"bool[]={list(value_msg.bool_array_value)}"
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return f"int[]={list(value_msg.integer_array_value)}"
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return f"double[]={list(value_msg.double_array_value)}"
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return f"string[]={list(value_msg.string_array_value)}"
    if t == ParameterType.PARAMETER_NOT_SET:
        return "not_set"
    return f"type={t}"


def ekf_diag_keys(name: str) -> list[str]:
    """Map diagnostic status name to logical stream key(s)."""
    n = name or ""
    keys: list[str] = []
    if "localization: ekf_localizer" not in n:
        return keys
    if "callback_pose" in n:
        keys.append("callback_pose")
    elif "callback_twist" in n:
        keys.append("callback_twist")
    elif "callback_" in n:
        keys.append("callback_other")
    else:
        keys.append("main")
    return keys


def inject_param_toggle_loop(
    *,
    node: str,
    param_name: str,
    interval_sec: float,
    n_sets: int,
    start_with_true: bool,
    log: Callable[[str], None],
) -> None:
    """Background: repeatedly `ros2 param set` to alternate inject flag (for tee evidence)."""
    if not shutil.which("ros2"):
        log("[param-toggle] ERROR: ros2 not in PATH")
        return
    for i in range(n_sets):
        odd = bool(i % 2)
        val = odd if not start_with_true else not odd
        cmd = ["ros2", "param", "set", node, param_name, "true" if val else "false"]
        log(f"[param-toggle] set {i + 1}/{n_sets} -> {val}  ({' '.join(cmd)})")
        t0 = time.perf_counter()
        r = subprocess.run(cmd, capture_output=True, text=True, check=False)
        elapsed = time.perf_counter() - t0
        out = (r.stdout or "").strip()
        err = (r.stderr or "").strip()
        log(
            f"[param-toggle] set {i + 1}/{n_sets} subprocess.returned "
            f"elapsed={elapsed:.4f}s rc={r.returncode} stdout={out!r} stderr={err!r}"
        )
        if r.returncode != 0:
            log("[param-toggle] WARN non-zero rc (see subprocess.returned line above)")
        if i < n_sets - 1:
            time.sleep(interval_sec)


class EkfDiagnosticsStampDelta(Node):
    def __init__(
        self,
        *,
        topic: str,
        qos_depth: int,
        print_main_level: bool,
        diag_array_hz: bool,
        diag_array_hz_interleaved: bool,
        ekf_short_name: str,
        parameter_events_watch_node: Optional[str] = None,
        parameter_events_watch_param: Optional[str] = None,
    ) -> None:
        super().__init__("ekf_diagnostics_stamp_delta")
        self._prev: dict[str, float] = {}
        self._print_main_level = print_main_level
        self._diag_array_hz = diag_array_hz
        self._diag_array_hz_interleaved = diag_array_hz_interleaved
        self._main_name = main_diag_status_name(ekf_short_name)
        self._prev_array_t: Optional[float] = None
        self._prev_array_t_interleaved: Optional[float] = None
        wn = (parameter_events_watch_node or "").strip()
        wp = (parameter_events_watch_param or "").strip()
        self._param_event_node = normalize_node_name(wn) if wn else None
        self._param_event_param = wp if wp else None
        qos = QoSProfile(depth=qos_depth)
        self.create_subscription(DiagnosticArray, topic, self._cb, qos)
        self.get_logger().info(f'Subscribed to "{topic}" (stamp-based dt for main / callback_*)')
        if self._param_event_node and self._param_event_param:
            self.create_subscription(
                ParameterEvent,
                "/parameter_events",
                self._on_parameter_event,
                qos,
            )
            self.get_logger().info(
                "[param-events] Subscribed to /parameter_events for "
                f"node={self._param_event_node!r} param={self._param_event_param!r} "
                "(DDS broadcast of parameter changes; same process as EKF would see soon after)"
            )

    def _cb(self, msg: DiagnosticArray) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        has_ekf_main = any((s.name or "") == self._main_name for s in msg.status)
        # /diagnostics には複数ノードが同一トピックへ出版する。無差別に header.stamp の差分を取ると
        # 他ノード分のメッセージが挟まり Hz が意味不明になる。EKF main 行が載っている配列だけを使う。
        if self._diag_array_hz and has_ekf_main:
            p0 = self._prev_array_t
            if p0 is not None:
                dt = t - p0
                if dt > 1e-9:
                    self.get_logger().info(
                        f"[DiagnosticArray EKF-only] hz ~= {1.0 / dt:.3f} (dt={dt:.4f} s)"
                    )
            self._prev_array_t = t
        if self._diag_array_hz_interleaved:
            p1 = self._prev_array_t_interleaved
            if p1 is not None:
                dt = t - p1
                if dt > 1e-9:
                    self.get_logger().info(
                        f"[DiagnosticArray all-publishers] hz ~= {1.0 / dt:.3f} (dt={dt:.4f} s) "
                        f"(interleaved stamps; not EKF rate)"
                    )
            self._prev_array_t_interleaved = t
        if self._print_main_level:
            for s in msg.status:
                if (s.name or "") == self._main_name:
                    lvl = diagnostic_level_to_int(s.level)
                    self.get_logger().info(
                        f'[main] level={lvl} ({LEVEL_NAMES.get(lvl, "?")}) msg="{s.message}"'
                    )
                    break
        updated_keys: set[str] = set()
        for s in msg.status:
            for key in ekf_diag_keys(s.name or ""):
                updated_keys.add(key)
        for key in sorted(updated_keys):
            p = self._prev.get(key)
            if p is not None:
                self.get_logger().info(f"[{key}] dt = {t - p:.4f} s")
            self._prev[key] = t

    def _on_parameter_event(self, msg: ParameterEvent) -> None:
        assert self._param_event_node and self._param_event_param
        if normalize_node_name(msg.node) != self._param_event_node:
            return
        stamp_t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        want = self._param_event_param
        for label, params in (
            ("changed", msg.changed_parameters),
            ("new", msg.new_parameters),
            ("deleted", msg.deleted_parameters),
        ):
            for p in params:
                if (p.name or "") != want:
                    continue
                self.get_logger().info(
                    f"[param-events] event_stamp={stamp_t:.9f}s kind={label} "
                    f"node={msg.node!r} name={p.name!r} value={format_parameter_value(p.value)}"
                )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Log dt of EKF-related diagnostics using DiagnosticArray.header.stamp.",
    )
    parser.add_argument(
        "--topic",
        default="/diagnostics",
        help="Diagnostics topic (default: /diagnostics)",
    )
    parser.add_argument(
        "--qos-depth",
        type=int,
        default=50,
        help="Subscription history depth (default: 50)",
    )
    parser.add_argument(
        "--print-main-level",
        action="store_true",
        help='Log merged main row (status name == "localization: <ekf_short_name>").',
    )
    parser.add_argument(
        "--diag-array-hz",
        action="store_true",
        help=(
            "Log rolling Hz using header.stamp only for messages that include the EKF main "
            'status row (name == "localization: <ekf_short_name>"). '
            "Use this on shared /diagnostics; raw interleaved Hz is misleading."
        ),
    )
    parser.add_argument(
        "--diag-array-hz-interleaved",
        action="store_true",
        help=(
            "Also log Hz from consecutive ANY /diagnostics message stamps (many publishers; "
            "values are not EKF publish rate)."
        ),
    )
    parser.add_argument(
        "--ekf-short-name",
        default="ekf_localizer",
        help='Short node name for main diag row (default: "ekf_localizer").',
    )
    parser.add_argument(
        "--inject-auto-toggle-node",
        default="",
        metavar="FULL_NODE_NAME",
        help=(
            "If set, a background thread runs `ros2 param set` on this node every "
            "`--inject-toggle-interval-sec`, alternating false/true/..."
        ),
    )
    parser.add_argument(
        "--inject-param-name",
        default="diagnostics.debug_inject_merge_error",
        help="Parameter name for inject toggle (default: diagnostics.debug_inject_merge_error).",
    )
    parser.add_argument(
        "--inject-toggle-interval-sec",
        type=float,
        default=5.0,
        help="Sleep between consecutive param sets (default: 5).",
    )
    parser.add_argument(
        "--inject-toggle-sets",
        type=int,
        default=8,
        metavar="N",
        help="Total number of ros2 param set calls (default: 8). i=0 -> false, i=1 -> true, ...",
    )
    parser.add_argument(
        "--inject-toggle-start-with-true",
        action="store_true",
        help="If set, first param value is true (then alternates). Default: first is false.",
    )
    parser.add_argument(
        "--inject-no-cleanup-on-exit",
        action="store_true",
        help="By default on exit we set inject param to false; pass this to skip.",
    )
    parser.add_argument(
        "--inject-log-parameter-events",
        action="store_true",
        help=(
            "With --inject-auto-toggle-node: also subscribe to /parameter_events and log when "
            "that node's --inject-param-name appears in new/changed/deleted lists (event stamp + value)."
        ),
    )
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    toggle_node = (args.inject_auto_toggle_node or "").strip()
    param_events_node = toggle_node if (toggle_node and args.inject_log_parameter_events) else ""
    param_events_param = (
        (args.inject_param_name or "").strip() if (toggle_node and args.inject_log_parameter_events) else ""
    )

    node = EkfDiagnosticsStampDelta(
        topic=args.topic,
        qos_depth=args.qos_depth,
        print_main_level=args.print_main_level,
        diag_array_hz=args.diag_array_hz,
        diag_array_hz_interleaved=args.diag_array_hz_interleaved,
        ekf_short_name=args.ekf_short_name,
        parameter_events_watch_node=param_events_node or None,
        parameter_events_watch_param=param_events_param or None,
    )
    if toggle_node:
        thr = threading.Thread(
            target=inject_param_toggle_loop,
            kwargs={
                "node": toggle_node,
                "param_name": args.inject_param_name,
                "interval_sec": args.inject_toggle_interval_sec,
                "n_sets": max(1, args.inject_toggle_sets),
                "start_with_true": args.inject_toggle_start_with_true,
                "log": node.get_logger().info,
            },
            daemon=True,
        )
        node.get_logger().info(
            f"[param-toggle] starting thread: node={toggle_node!r} param={args.inject_param_name!r} "
            f"interval={args.inject_toggle_interval_sec}s sets={args.inject_toggle_sets}"
        )
        thr.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if toggle_node and not args.inject_no_cleanup_on_exit and shutil.which("ros2"):
            t0 = time.perf_counter()
            r = subprocess.run(
                ["ros2", "param", "set", toggle_node, args.inject_param_name, "false"],
                capture_output=True,
                text=True,
                check=False,
            )
            dt = time.perf_counter() - t0
            node.get_logger().info(
                f"[param-toggle] cleanup: set inject param to false "
                f"(subprocess elapsed={dt:.4f}s rc={r.returncode})"
            )
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
