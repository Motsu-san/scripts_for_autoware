#!/usr/bin/env python3
"""Publish a fixed pose from YAML for visualization."""

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def _pose_dict_to_values(pose: Dict[str, Any]) -> Tuple[float, float, float, float, float, float, float]:
    pos = pose["position"]
    ori = pose["orientation"]
    return (
        float(pos["x"]),
        float(pos["y"]),
        float(pos["z"]),
        float(ori["x"]),
        float(ori["y"]),
        float(ori["z"]),
        float(ori["w"]),
    )


def load_pose_from_yaml(
    yaml_path: Path,
    offset_from_nearest: Optional[int] = None,
) -> Tuple[float, float, float, float, float, float, float, str]:
    """YAML から位置姿勢と frame_id を読み込む。

    - initial_pose.yaml: ルートの pose.pose
    - mean_ndt_pose.yaml: --offset-from-nearest 指定時は
      aggregated.per_scan_summary の該当 offset の mean_pose
    """
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        raise ValueError(f"{yaml_path}: 空の YAML です")

    frame_id = "map"
    header = data.get("header")
    if isinstance(header, dict) and header.get("frame_id"):
        frame_id = str(header["frame_id"])
    stamp_block = data.get("mean_pose_header_stamp")
    if isinstance(stamp_block, dict) and stamp_block.get("frame_id"):
        frame_id = str(stamp_block["frame_id"])

    if offset_from_nearest is not None:
        agg = data.get("aggregated") or {}
        per_scan = agg.get("per_scan_summary") or []
        for scan in per_scan:
            if not isinstance(scan, dict):
                continue
            if scan.get("offset_from_nearest") == offset_from_nearest:
                mean_pose = scan.get("mean_pose")
                if not isinstance(mean_pose, dict):
                    break
                x, y, z, qx, qy, qz, qw = _pose_dict_to_values(mean_pose)
                return x, y, z, qx, qy, qz, qw, frame_id
        raise ValueError(
            f"{yaml_path}: offset_from_nearest={offset_from_nearest} の "
            "per_scan_summary エントリが見つかりません"
        )

    pose_block = data.get("pose")
    if pose_block is None:
        raise ValueError(f"{yaml_path}: pose ブロックがありません")
    pose = pose_block.get("pose", pose_block)
    if not isinstance(pose, dict):
        raise ValueError(f"{yaml_path}: pose.pose の形式が不正です")
    x, y, z, qx, qy, qz, qw = _pose_dict_to_values(pose)
    return x, y, z, qx, qy, qz, qw, frame_id


class PosePublisher(Node):
    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        frame_id: str,
        topic: str,
        rate_hz: float,
    ):
        super().__init__("pose_publisher")
        self._x = x
        self._y = y
        self._z = z
        self._qx = qx
        self._qy = qy
        self._qz = qz
        self._qw = qw
        self._frame_id = frame_id
        self.publisher_ = self.create_publisher(PoseStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(
            f"Publishing pose from YAML: frame_id={frame_id}, "
            f"x={x:.5f}, y={y:.5f}, z={z:.5f}, topic={topic}"
        )

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = self._z
        msg.pose.orientation.x = self._qx
        msg.pose.orientation.y = self._qy
        msg.pose.orientation.z = self._qz
        msg.pose.orientation.w = self._qw
        self.publisher_.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="YAML の pose を visualization_pose に publish する",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # initial_pose.yaml の pose
  pose_publisher.py /path/to/initial_pose.yaml

  # mean_ndt_pose.yaml の offset_from_nearest=0 の mean_pose
  pose_publisher.py /path/to/mean_ndt_pose.yaml --offset-from-nearest 0
        """,
    )
    parser.add_argument("yaml", type=Path, help="initial_pose.yaml または mean_ndt_pose.yaml")
    parser.add_argument(
        "--offset-from-nearest",
        type=int,
        default=None,
        metavar="N",
        help="mean_ndt_pose.yaml の aggregated.per_scan_summary[N].mean_pose を使う",
    )
    parser.add_argument(
        "--topic",
        default="visualization_pose",
        help="publish 先トピック (default: visualization_pose)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="publish 周期 [Hz] (default: 1.0)",
    )
    return parser.parse_args()


def main(args=None):
    cli_args = parse_args()
    if not cli_args.yaml.is_file():
        print(f"Error: ファイルがありません: {cli_args.yaml}", file=sys.stderr)
        sys.exit(1)

    try:
        x, y, z, qx, qy, qz, qw, frame_id = load_pose_from_yaml(
            cli_args.yaml,
            offset_from_nearest=cli_args.offset_from_nearest,
        )
    except (KeyError, TypeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = PosePublisher(
        x, y, z, qx, qy, qz, qw, frame_id, cli_args.topic, cli_args.rate
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
