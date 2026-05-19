#!/usr/bin/env python3
"""
固定 ndt_start_pose.yaml と bag から抽出した 1 フレーム点群で NDT scan matcher を N 回評価し、
/localization/pose_estimator/pose_with_covariance の平均を求める。

前提: localization_standalone が起動済み、/clock が流れていること。
試行ごとに EKF/NDT を deactivate → set_initial_pose (--skip-initial-localization) → 点群 1 回 pub → NDT pose 取得。
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool

# 同リポジトリの集計ユーティリティ
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from aggregate_pose_mean_from_bags import (  # noqa: E402
    average_quaternions,
    build_mean_pose_yaml,
    deviation_about_mean_pose,
    extract_covariance,
    extract_nearest_pointcloud_header,
    find_rosbag2_directory,
    msg_pose_to_arrays,
    stamp_to_sec,
    yaw_from_quat_xyzw,
)

SET_INITIAL_POSE_PY = _SCRIPT_DIR / "set_initial_pose.py"
DEFAULT_POSE_TOPIC = "/localization/pose_estimator/pose_with_covariance"
DEFAULT_POINTCLOUD_TOPIC = "/sensing/lidar/concatenated/pointcloud"
EKF_TRIGGER = "/localization/pose_twist_fusion_filter/trigger_node"
NDT_TRIGGER = "/localization/pose_estimator/trigger_node"


def extract_nearest_pointcloud_from_bag(
    bag_path: str, topic: str, target_sec: float
) -> Tuple[PointCloud2, Dict[str, Any]]:
    """指定時刻に最も近い PointCloud2 を bag から返す。"""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    bag_dir = find_rosbag2_directory(bag_path)
    meta = extract_nearest_pointcloud_header(bag_dir, topic, target_sec)
    if meta is None:
        raise RuntimeError(f"bag に点群がありません: topic={topic} bag={bag_dir}")

    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    if topic not in type_map:
        raise RuntimeError(f"topic 不在: {topic}")
    msg_type = get_message(type_map[topic])
    target_hdr = float(meta["pointcloud_header_stamp_sec"])
    best_dt = float(meta["dt_pointcloud_header_from_target_sec"])
    best_msg: Optional[PointCloud2] = None
    while reader.has_next():
        tname, data, _ts_ns = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, PointCloud2):
            continue
        hs = stamp_to_sec(msg.header.stamp)
        dt = abs(hs - target_hdr)
        if best_msg is None or dt < best_dt:
            best_dt = dt
            best_msg = msg
    if best_msg is None:
        raise RuntimeError("点群メッセージの読み込みに失敗しました")
    return best_msg, meta


class SimClockPublisher(Node):
    def __init__(self, unix_sec: float, rate_hz: float = 10.0):
        super().__init__("measure_ndt_sim_clock")
        sec = int(math.floor(unix_sec))
        nsec = int(round((unix_sec - sec) * 1e9))
        if nsec >= 1_000_000_000:
            sec += 1
            nsec -= 1_000_000_000
        self._stamp = Time(sec=sec, nanosec=nsec)
        self._pub = self.create_publisher(Clock, "/clock", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self) -> None:
        msg = Clock()
        msg.clock = self._stamp
        self._pub.publish(msg)


class NdtPoseMeanRunner(Node):
    def __init__(
        self,
        *,
        pointcloud: PointCloud2,
        initial_pose_yaml: Path,
        n_runs: int,
        pose_topic: str,
        publish_topic: str,
        trial_timeout_sec: float,
        settle_sec: float,
        cloud_publish_count: int,
        cloud_publish_hz: float,
    ):
        super().__init__("measure_ndt_pose_mean_runner")
        self._cloud = pointcloud
        self._initial_pose_yaml = initial_pose_yaml
        self._n_runs = n_runs
        self._pose_topic = pose_topic
        self._publish_topic = publish_topic
        self._trial_timeout = trial_timeout_sec
        self._settle_sec = settle_sec
        self._cloud_publish_count = max(1, cloud_publish_count)
        self._cloud_publish_period = 1.0 / max(cloud_publish_hz, 1.0)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2, self._publish_topic, sensor_qos
        )
        self._ekf_cli = self.create_client(SetBool, EKF_TRIGGER)
        self._ndt_cli = self.create_client(SetBool, NDT_TRIGGER)
        self._last_pose: Optional[PoseWithCovarianceStamped] = None
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self._pose_topic,
            self._on_pose,
            10,
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_pose = msg

    def _wait_service(self, client, name: str, timeout_sec: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.5):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(f"service not available: {name}")
        return False

    def _call_setbool(self, client, service_name: str, data: bool) -> bool:
        if not self._wait_service(client, service_name):
            return False
        req = SetBool.Request()
        req.data = data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done():
            self.get_logger().warn(f"service call timeout: {service_name}")
            return False
        try:
            res = future.result()
            return bool(res.success)
        except Exception as e:
            self.get_logger().warn(f"service call failed {service_name}: {e}")
            return False

    def _deactivate_localization(self) -> None:
        self._call_setbool(self._ekf_cli, EKF_TRIGGER, False)
        self._call_setbool(self._ndt_cli, NDT_TRIGGER, False)
        time.sleep(0.3)

    def _apply_initial_pose(self) -> bool:
        if not SET_INITIAL_POSE_PY.is_file():
            self.get_logger().error(f"missing: {SET_INITIAL_POSE_PY}")
            return False
        cmd = [
            sys.executable,
            str(SET_INITIAL_POSE_PY),
            "--skip-initial-localization",
            str(self._initial_pose_yaml),
        ]
        self.get_logger().info(f"Running: {' '.join(cmd)}")
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        if proc.returncode != 0:
            self.get_logger().error(
                f"set_initial_pose failed ({proc.returncode}): {proc.stderr or proc.stdout}"
            )
            return False
        time.sleep(self._settle_sec)
        return True

    def _publish_pointcloud_burst(self) -> None:
        for _ in range(self._cloud_publish_count):
            self._cloud_pub.publish(self._cloud)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(self._cloud_publish_period)

    def _wait_ndt_pose(self) -> Optional[PoseWithCovarianceStamped]:
        self._last_pose = None
        deadline = time.monotonic() + self._trial_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._last_pose is not None:
                return self._last_pose
        return None

    def run_trials(self) -> List[Dict[str, Any]]:
        rows: List[Dict[str, Any]] = []
        for i in range(1, self._n_runs + 1):
            self.get_logger().info(f"Trial {i}/{self._n_runs}")
            self._deactivate_localization()
            if not self._apply_initial_pose():
                raise RuntimeError(f"trial {i}: initial pose failed")
            self._publish_pointcloud_burst()
            pose = self._wait_ndt_pose()
            if pose is None:
                raise RuntimeError(
                    f"trial {i}: timeout waiting for {self._pose_topic} "
                    f"({self._trial_timeout}s)"
                )
            pos, quat = msg_pose_to_arrays(pose)
            cov = extract_covariance(pose)
            hdr = stamp_to_sec(pose.header.stamp)
            row: Dict[str, Any] = {
                "trial_index": i,
                "header_stamp_sec": hdr,
                "header_frame_id": pose.header.frame_id or "",
                "position": {
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                },
                "orientation": {
                    "x": float(quat[0]),
                    "y": float(quat[1]),
                    "z": float(quat[2]),
                    "w": float(quat[3]),
                },
            }
            if cov is not None:
                row["covariance"] = cov
            rows.append(row)
            self.get_logger().info(
                f"  pose x={pos[0]:.3f} y={pos[1]:.3f} yaw={math.degrees(yaw_from_quat_xyzw(quat)):.3f}deg"
            )
        return rows


def aggregate_trial_rows(
    rows: List[Dict[str, Any]],
    *,
    target_unix_sec: float,
    pose_topic: str,
    pointcloud_meta: Dict[str, Any],
    source_bag: str,
) -> Dict[str, Any]:
    positions = [
        np.array([r["position"]["x"], r["position"]["y"], r["position"]["z"]], dtype=float)
        for r in rows
    ]
    quats = [
        np.array(
            [
                r["orientation"]["x"],
                r["orientation"]["y"],
                r["orientation"]["z"],
                r["orientation"]["w"],
            ],
            dtype=float,
        )
        for r in rows
    ]
    P = np.stack(positions, axis=0)
    mean_pos = np.mean(P, axis=0)
    std_pos = np.std(P, axis=0)
    Q = np.stack(quats, axis=0)
    mean_quat = average_quaternions(Q)
    yaws = np.array([yaw_from_quat_xyzw(q) for q in Q])
    mean_yaw = math.atan2(float(np.mean(np.sin(yaws))), float(np.mean(np.cos(yaws))))

    covs = [np.array(r["covariance"], dtype=float) for r in rows if "covariance" in r]
    mean_cov: Optional[np.ndarray] = None
    if len(covs) == len(rows) and covs:
        mean_cov = np.mean(np.stack(covs, axis=0), axis=0)

    dev_stats, dev_per_run = deviation_about_mean_pose(mean_pos, mean_quat, P, Q)
    for i, row in enumerate(rows):
        row["deviation_about_mean"] = dev_per_run[i]

    per_run = []
    for r in rows:
        pr = dict(r)
        pr["dt_from_target_sec"] = abs(float(r["header_stamp_sec"]) - target_unix_sec)
        pr["pointcloud_header_stamp_sec"] = pointcloud_meta["pointcloud_header_stamp_sec"]
        pr["pointcloud_header_frame_id"] = pointcloud_meta.get(
            "pointcloud_header_frame_id", ""
        )
        pr["dt_pointcloud_header_from_target_sec"] = pointcloud_meta.get(
            "dt_pointcloud_header_from_target_sec", 0.0
        )
        pr["dt_pose_from_pointcloud_header_sec"] = abs(
            float(r["header_stamp_sec"])
            - float(pointcloud_meta["pointcloud_header_stamp_sec"])
        )
        per_run.append(pr)

    out: Dict[str, Any] = {
        "status": "ok",
        "method": "ndt_fixed_initial_pose_single_scan",
        "n_runs": len(rows),
        "target_unix_sec": target_unix_sec,
        "pose_topic": pose_topic,
        "source_rosbag": source_bag,
        "pointcloud_source_topic": pointcloud_meta.get("pointcloud_topic", ""),
        "pointcloud_header": {
            k: pointcloud_meta.get(k)
            for k in (
                "pointcloud_header_stamp_sec",
                "pointcloud_header_frame_id",
                "dt_pointcloud_header_from_target_sec",
            )
        },
        "mean_pose": {
            "position": {
                "x": float(mean_pos[0]),
                "y": float(mean_pos[1]),
                "z": float(mean_pos[2]),
            },
            "orientation": {
                "x": float(mean_quat[0]),
                "y": float(mean_quat[1]),
                "z": float(mean_quat[2]),
                "w": float(mean_quat[3]),
            },
            "yaw_rad_circular_mean_from_each_run": mean_yaw,
            "yaw_deg_circular_mean_from_each_run": math.degrees(mean_yaw),
        },
        "position_std_xyz_m": {
            "x": float(std_pos[0]),
            "y": float(std_pos[1]),
            "z": float(std_pos[2]),
        },
        "deviation_about_mean": {**dev_stats, "per_run": dev_per_run},
        "per_run": per_run,
    }
    if mean_cov is not None:
        out["mean_covariance"] = [float(x) for x in mean_cov.tolist()]
    return out, mean_pos, mean_quat, mean_cov, per_run, dev_stats, dev_per_run


def run_clock_only(unix_sec: float, rate_hz: float = 10.0) -> int:
    rclpy.init()
    node = SimClockPublisher(unix_sec, rate_hz=rate_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="固定初期位置+1点群で NDT pose を N 回平均")
    p.add_argument("--source-bag", required=True, help="点群抽出元 rosbag (.db3 またはディレクトリ)")
    p.add_argument("--target-unix-sec", type=float, required=True)
    p.add_argument("--initial-pose-yaml", required=True)
    p.add_argument("--n-runs", type=int, default=100)
    p.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC)
    p.add_argument("--pointcloud-topic", default=DEFAULT_POINTCLOUD_TOPIC)
    p.add_argument("--publish-pointcloud-topic", default="", help="空なら --pointcloud-topic と同じ")
    p.add_argument("--trial-timeout-sec", type=float, default=15.0)
    p.add_argument("--settle-sec", type=float, default=1.0, help="initial pose 設定後の待ち")
    p.add_argument("--cloud-publish-count", type=int, default=3)
    p.add_argument("--cloud-publish-hz", type=float, default=10.0)
    p.add_argument("--output-json", default="")
    p.add_argument("--output-mean-pose-yaml", default="")
    p.add_argument("--mean-pose-frame-id", default="map")
    p.add_argument(
        "--publish-clock",
        action="store_true",
        help="指定時刻で /clock を publish（launch 側で出していない場合）",
    )
    p.add_argument("--clock-hz", type=float, default=10.0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    initial_yaml = Path(args.initial_pose_yaml)
    if not initial_yaml.is_file():
        print(f"Error: ndt_start_pose.yaml not found: {initial_yaml}", file=sys.stderr)
        return 1

    publish_topic = args.publish_pointcloud_topic or args.pointcloud_topic
    try:
        cloud, pc_meta = extract_nearest_pointcloud_from_bag(
            args.source_bag, args.pointcloud_topic, float(args.target_unix_sec)
        )
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2
    pc_meta = dict(pc_meta)
    pc_meta["pointcloud_topic"] = args.pointcloud_topic

    rclpy.init()
    clock_node: Optional[SimClockPublisher] = None
    clock_stop = threading.Event()
    clock_thread: Optional[threading.Thread] = None
    runner: Optional[NdtPoseMeanRunner] = None
    try:
        if args.publish_clock:
            clock_node = SimClockPublisher(float(args.target_unix_sec), rate_hz=args.clock_hz)

            def _spin_clock() -> None:
                assert clock_node is not None
                while not clock_stop.is_set() and rclpy.ok():
                    rclpy.spin_once(clock_node, timeout_sec=0.05)

            clock_thread = threading.Thread(target=_spin_clock, daemon=True)
            clock_thread.start()
            time.sleep(0.3)
        runner = NdtPoseMeanRunner(
            pointcloud=cloud,
            initial_pose_yaml=initial_yaml,
            n_runs=int(args.n_runs),
            pose_topic=args.pose_topic,
            publish_topic=publish_topic,
            trial_timeout_sec=float(args.trial_timeout_sec),
            settle_sec=float(args.settle_sec),
            cloud_publish_count=int(args.cloud_publish_count),
            cloud_publish_hz=float(args.cloud_publish_hz),
        )
        # サービス待ち
        if not runner._wait_service(runner._ekf_cli, EKF_TRIGGER, timeout_sec=90.0):
            print("Error: EKF trigger service not ready", file=sys.stderr)
            return 3
        if not runner._wait_service(runner._ndt_cli, NDT_TRIGGER, timeout_sec=90.0):
            print("Error: NDT trigger service not ready", file=sys.stderr)
            return 3

        rows = runner.run_trials()
        result, mean_pos, mean_quat, mean_cov, per_run, dev_stats, dev_per_run = (
            aggregate_trial_rows(
                rows,
                target_unix_sec=float(args.target_unix_sec),
                pose_topic=args.pose_topic,
                pointcloud_meta=pc_meta,
                source_bag=args.source_bag,
            )
        )

        text = json.dumps(result, indent=2)
        print(text)
        if args.output_json:
            Path(args.output_json).write_text(text, encoding="utf-8")
            print(f"Wrote: {args.output_json}", file=sys.stderr)

        if args.output_mean_pose_yaml:
            yaml_deviation: Dict[str, Any] = {**dev_stats}
            yaml_deviation["per_run_longitudinal_m"] = [
                x["longitudinal_m"] for x in dev_per_run
            ]
            yaml_deviation["per_run_lateral_m"] = [x["lateral_m"] for x in dev_per_run]
            yaml_deviation["per_run_yaw_error_rad"] = [
                x["yaw_error_rad"] for x in dev_per_run
            ]
            yaml_deviation["per_run_yaw_error_deg"] = [
                x["yaw_error_deg"] for x in dev_per_run
            ]
            yaml_doc = build_mean_pose_yaml(
                initial_pose_yaml=initial_yaml,
                target_unix_sec=float(args.target_unix_sec),
                pose_topic=args.pose_topic,
                mean_pos=mean_pos,
                mean_quat=mean_quat,
                mean_cov=mean_cov,
                frame_id=args.mean_pose_frame_id,
                per_run=per_run,
                align_pointcloud_topic=args.pointcloud_topic,
                deviation_about_mean=yaml_deviation,
            )
            yaml_doc["aggregated"]["method"] = "ndt_fixed_initial_pose_single_scan"
            yaml_doc["aggregated"]["n_trials_requested"] = int(args.n_runs)
            yaml_text = yaml.dump(
                yaml_doc, default_flow_style=False, allow_unicode=True, sort_keys=False
            )
            Path(args.output_mean_pose_yaml).write_text(yaml_text, encoding="utf-8")
            print(f"Wrote: {args.output_mean_pose_yaml}", file=sys.stderr)
        return 0
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 4
    finally:
        clock_stop.set()
        if clock_thread is not None:
            clock_thread.join(timeout=2.0)
        if runner is not None:
            runner.destroy_node()
        if clock_node is not None:
            clock_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "--clock-only":
        if len(sys.argv) < 3:
            print("Usage: measure_ndt_pose_mean.py --clock-only <UNIX_SEC> [HZ]", file=sys.stderr)
            sys.exit(1)
        hz = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
        sys.exit(run_clock_only(float(sys.argv[2]), rate_hz=hz))
    sys.exit(main())
