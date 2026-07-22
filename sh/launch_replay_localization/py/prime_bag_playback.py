#!/usr/bin/env python3
"""
rosbag 再生を --start-paused + --start-offset で開始し、sim time を合わせてから
initial_pose を設定してから /rosbag2_player/resume で本再生する。

launch_autoware.sh の -t 指定時（initial_pose.yaml あり）に使用する。
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rosbag2_interfaces.srv import IsPaused, PlayNext, Resume

_SCRIPT_DIR = Path(__file__).resolve().parent
SET_INITIAL_POSE_PY = _SCRIPT_DIR / "set_initial_pose.py"

DEFAULT_PLAYER_PREFIX = "/rosbag2_player"
DEFAULT_PLAY_CLOCK_HZ = 200.0
DEFAULT_SETTLE_SEC = 5.0
DEFAULT_SERVICE_TIMEOUT_SEC = 60.0
DEFAULT_CLOCK_PRIME_TIMEOUT_SEC = 30.0
CLOCK_TOLERANCE_SEC = 0.5
MAX_PLAY_NEXT_ATTEMPTS = 200


def unix_sec_to_time_msg(unix_sec: float) -> Time:
    sec = int(unix_sec)
    nanosec = int(round((unix_sec - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    t = Time()
    t.sec = sec
    t.nanosec = nanosec
    return t


def stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class BagPlaybackPrimer(Node):
    def __init__(self, player_prefix: str, target_unix_sec: float):
        super().__init__("bag_playback_primer")
        self._player_prefix = player_prefix.rstrip("/")
        self._target_unix_sec = float(target_unix_sec)
        self._resume_cli = self.create_client(
            Resume, f"{self._player_prefix}/resume"
        )
        self._play_next_cli = self.create_client(
            PlayNext, f"{self._player_prefix}/play_next"
        )
        self._is_paused_cli = self.create_client(
            IsPaused, f"{self._player_prefix}/is_paused"
        )
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._clock_pub = self.create_publisher(Clock, "/clock", clock_qos)
        self._last_clock_sec: Optional[float] = None
        self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)

    def _on_clock(self, msg: Clock) -> None:
        self._last_clock_sec = stamp_to_sec(msg.clock)

    def wait_for_player(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._resume_cli.wait_for_service(timeout_sec=0.2):
                return
            if not rclpy.ok():
                raise RuntimeError("ROS shutdown while waiting for rosbag2 player")
        raise TimeoutError(
            f"rosbag2 player service not available: {self._player_prefix}/resume "
            f"(timeout {timeout_sec}s)"
        )

    def _call_sync(self, client, request, label: str, timeout_sec: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"service unavailable: {label}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"service call timeout: {label}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service call failed: {label}")
        return result

    def prime_sim_clock(self) -> float:
        """一時停止中に /clock を target 付近まで進める。戻り値は観測した clock [s]。"""
        target = self._target_unix_sec
        self.get_logger().info(
            f"Priming sim clock toward target={target:.3f} (paused play_next/burst)"
        )

        for attempt in range(1, MAX_PLAY_NEXT_ATTEMPTS + 1):
            if self._last_clock_sec is not None and self._last_clock_sec + CLOCK_TOLERANCE_SEC >= target:
                self.get_logger().info(
                    f"Clock primed via playback: {self._last_clock_sec:.3f} "
                    f"(attempt {attempt})"
                )
                return self._last_clock_sec

            try:
                result = self._call_sync(
                    self._play_next_cli,
                    PlayNext.Request(),
                    "play_next",
                    timeout_sec=5.0,
                )
                if not result.success:
                    self.get_logger().warn(
                        f"play_next returned success=false (attempt {attempt})"
                    )
            except Exception as exc:
                self.get_logger().warn(f"play_next failed (attempt {attempt}): {exc}")

            rclpy.spin_once(self, timeout_sec=0.05)

        if self._last_clock_sec is not None:
            self.get_logger().warn(
                f"play_next did not reach target; last clock={self._last_clock_sec:.3f}. "
                "Publishing synthetic /clock at target."
            )
        else:
            self.get_logger().warn(
                "No /clock observed from bag; publishing synthetic /clock at target."
            )

        msg = Clock()
        msg.clock = unix_sec_to_time_msg(target)
        for _ in range(5):
            self._clock_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._last_clock_sec = target
        return target

    def publish_clock_at_target(self) -> None:
        msg = Clock()
        msg.clock = unix_sec_to_time_msg(self._target_unix_sec)
        for _ in range(5):
            self._clock_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def resume_playback(self) -> None:
        self.get_logger().info("Calling rosbag2 player resume")
        self._call_sync(
            self._resume_cli,
            Resume.Request(),
            "resume",
            timeout_sec=10.0,
        )

    def is_player_paused(self) -> Optional[bool]:
        if not self._is_paused_cli.wait_for_service(timeout_sec=0.5):
            return None
        try:
            result = self._call_sync(
                self._is_paused_cli,
                IsPaused.Request(),
                "is_paused",
                timeout_sec=5.0,
            )
            return bool(result.paused)
        except Exception:
            return None


def build_bag_play_cmd(
    bag_path: Path,
    *,
    rate: float,
    start_offset: float,
    clock_hz: Optional[float],
    use_bag_clock: bool,
) -> List[str]:
    cmd = [
        "ros2",
        "bag",
        "play",
        str(bag_path),
        "-r",
        str(rate),
        "--start-paused",
    ]
    if start_offset > 0.0:
        cmd.extend(["--start-offset", f"{start_offset:.3f}"])
    if not use_bag_clock:
        hz = clock_hz if clock_hz is not None else DEFAULT_PLAY_CLOCK_HZ
        cmd.extend(["--clock", str(hz)])
    return cmd


def open_play_stdin():
    """ros2 bag play のキーボード操作向けに TTY を開く（無ければ None）。"""
    try:
        if Path("/dev/tty").is_file():
            return open("/dev/tty", "r")  # noqa: SIM115
    except OSError:
        pass
    return None


def run_set_initial_pose(
    initial_pose_yaml: Path,
    stamp_unix_sec: float,
    skip_initial_localization: bool,
    settle_sec: float,
) -> None:
    if not SET_INITIAL_POSE_PY.is_file():
        raise FileNotFoundError(f"missing: {SET_INITIAL_POSE_PY}")

    cmd = [
        sys.executable,
        str(SET_INITIAL_POSE_PY),
        "--stamp-unix-sec",
        str(stamp_unix_sec),
        str(initial_pose_yaml),
    ]
    if skip_initial_localization:
        cmd.insert(-1, "--skip-initial-localization")

    print(f"Running: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"set_initial_pose failed (exit {proc.returncode})")

    if settle_sec > 0.0:
        print(f"Settle {settle_sec}s after initial pose...", flush=True)
        time.sleep(settle_sec)


def load_skip_initial_localization(yaml_path: Path) -> bool:
    import yaml

    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return bool(data.get("skip_initial_localization"))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Pause bag at offset, set initial pose at target time, then resume."
    )
    parser.add_argument("--bag", type=Path, required=True, help="rosbag path")
    parser.add_argument(
        "--start-offset",
        type=float,
        default=0.0,
        help="ros2 bag play --start-offset [s]",
    )
    parser.add_argument("-r", "--rate", type=float, default=1.0, help="playback rate")
    parser.add_argument(
        "--clock-hz",
        type=float,
        default=None,
        help="ros2 bag play --clock Hz (bag に /clock が無いとき)",
    )
    parser.add_argument(
        "--use-bag-clock",
        action="store_true",
        help="bag 内 /clock を使う（--clock を付けない）",
    )
    parser.add_argument(
        "--initial-pose-yaml",
        type=Path,
        required=True,
        help="initial_pose.yaml",
    )
    parser.add_argument(
        "--stamp-unix-sec",
        type=float,
        required=True,
        help="initial pose / sim time の目標 UNIX 時刻 (-t)",
    )
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=DEFAULT_SETTLE_SEC,
        help="initial pose 設定後の待ち [s]",
    )
    parser.add_argument(
        "--player-prefix",
        default=DEFAULT_PLAYER_PREFIX,
        help="rosbag2 player サービス prefix",
    )
    parser.add_argument(
        "--service-timeout-sec",
        type=float,
        default=DEFAULT_SERVICE_TIMEOUT_SEC,
        help="player サービス待ちタイムアウト",
    )
    args = parser.parse_args()

    if not args.bag.exists():
        print(f"Error: bag not found: {args.bag}", file=sys.stderr)
        return 2
    if not args.initial_pose_yaml.is_file():
        print(f"Error: initial_pose.yaml not found: {args.initial_pose_yaml}", file=sys.stderr)
        return 2

    skip_init = load_skip_initial_localization(args.initial_pose_yaml)
    play_cmd = build_bag_play_cmd(
        args.bag,
        rate=args.rate,
        start_offset=args.start_offset,
        clock_hz=args.clock_hz,
        use_bag_clock=args.use_bag_clock,
    )
    print(f"Starting paused bag play: {' '.join(play_cmd)}", flush=True)

    play_proc: Optional[subprocess.Popen] = None

    def _terminate_play(signum=None, frame=None):
        if play_proc is not None and play_proc.poll() is None:
            print("Terminating bag play process...", flush=True)
            play_proc.send_signal(signal.SIGINT)
            try:
                play_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                play_proc.kill()

    signal.signal(signal.SIGTERM, _terminate_play)
    signal.signal(signal.SIGINT, _terminate_play)

    play_proc = subprocess.Popen(play_cmd, stdin=open_play_stdin())
    exit_code = 1

    try:
        rclpy.init()
        primer = BagPlaybackPrimer(args.player_prefix, args.stamp_unix_sec)
        try:
            primer.wait_for_player(args.service_timeout_sec)
            paused = primer.is_player_paused()
            if paused is False:
                primer.get_logger().warn("Player reports not paused; continuing anyway")

            observed = primer.prime_sim_clock()
            primer.publish_clock_at_target()
            primer.get_logger().info(
                f"Clock ready (observed≈{observed:.3f}, target={args.stamp_unix_sec:.3f})"
            )

            run_set_initial_pose(
                args.initial_pose_yaml,
                args.stamp_unix_sec,
                skip_initial_localization=skip_init,
                settle_sec=args.settle_sec,
            )

            primer.resume_playback()
            primer.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

        print("Waiting for bag playback to complete...", flush=True)
        exit_code = play_proc.wait()
        if exit_code != 0:
            print(f"Warning: bag play exited with code {exit_code}", file=sys.stderr)
        else:
            print("Bag playback completed.", flush=True)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        _terminate_play()
        if play_proc.poll() is None:
            play_proc.wait(timeout=5)
        return 1
    finally:
        if play_proc is not None and play_proc.poll() is None:
            _terminate_play()

    return exit_code if exit_code is not None else 0


if __name__ == "__main__":
    sys.exit(main())
