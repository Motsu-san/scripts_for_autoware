#!/usr/bin/env python3
"""
複数の記録 rosbag から、指定 UNIX 時刻に最も近い geometry_msgs/PoseWithCovarianceStamped(または PoseStamped)
を各 bag で抽出し、位置の算術平均・四元数符号揃え平均を求める。

--align-pointcloud-topic を指定した場合は、各 bag でまず点群 (PointCloud2) の header.stamp が target に最も近い
フレームを選び、その時刻に最も近い pose を採用する(run 間で「同じ点群フレーム」基準に揃える)。

PoseWithCovarianceStamped のみ各 run で共分散 36 要素の算術平均も計算し、mean_pose.yaml に反映可能。

mean_pose.yaml の aggregated.deviation_about_mean に、平均 pose 基準の縦・横・ヨーの標準偏差・最大絶対偏差と各 run のずれを追記する。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from scipy.spatial.transform import Rotation as R


def find_rosbag2_directory(bag_path: str) -> str:
    p = Path(bag_path)
    if p.is_dir() and (p / "metadata.yaml").exists():
        return str(p)
    if p.is_dir():
        cand = [d for d in p.iterdir() if d.is_dir() and (d / "metadata.yaml").exists()]
        if cand:
            return str(sorted(cand, key=lambda x: x.name, reverse=True)[0])
    parent = p.parent
    if (
        p.is_file()
        and parent.is_dir()
        and (parent / "metadata.yaml").exists()
    ):
        return str(parent)
    if parent.is_dir():
        cand = [d for d in parent.iterdir() if d.is_dir() and (d / "metadata.yaml").exists()]
        if cand:
            return str(sorted(cand, key=lambda x: x.name, reverse=True)[0])
    return str(p)


def stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def float_sec_to_stamp_dict(t: float) -> Dict[str, int]:
    sec = int(math.floor(t))
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return {"sec": sec, "nanosec": nsec}


def msg_pose_to_arrays(msg: Any) -> Tuple[np.ndarray, np.ndarray]:
    if isinstance(msg, PoseWithCovarianceStamped):
        pose = msg.pose.pose
    elif isinstance(msg, PoseStamped):
        pose = msg.pose
    else:
        raise TypeError(type(msg))
    p = pose.position
    q = pose.orientation
    pos = np.array([p.x, p.y, p.z], dtype=float)
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    n = np.linalg.norm(quat)
    if n > 1e-9:
        quat /= n
    return pos, quat


def extract_covariance(msg: Any) -> Optional[List[float]]:
    if isinstance(msg, PoseWithCovarianceStamped):
        return [float(x) for x in msg.pose.covariance[:36]]
    return None


def extract_nearest_pointcloud_header(
    bag_dir: str, topic: str, target_sec: float
) -> Optional[Dict[str, Any]]:
    """PointCloud2 の header.stamp が target_sec に最も近いメッセージを選ぶ。"""
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    if topic not in type_map:
        return None
    msg_type = get_message(type_map[topic])
    best: Optional[Tuple[float, PointCloud2, int]] = None
    while reader.has_next():
        tname, data, ts_ns = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, PointCloud2):
            continue
        hs = stamp_to_sec(msg.header.stamp)
        dt = abs(hs - target_sec)
        if best is None or dt < best[0]:
            best = (dt, msg, ts_ns)
    if best is None:
        return None
    dt, msg, ts_ns = best
    return {
        "target_pointcloud_header_stamp_sec": stamp_to_sec(msg.header.stamp),
        "pointcloud_header_frame_id": getattr(msg.header, "frame_id", "") or "",
        "dt_pointcloud_header_from_target_sec": dt,
        "pointcloud_bag_receive_time_ns": int(ts_ns),
    }


def extract_nearest_pose(
    bag_dir: str, topic: str, target_sec: float
) -> Optional[Dict[str, Any]]:
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    if topic not in type_map:
        # SequentialReader に close() が無い環境がある(rosbag2_py 版差)
        return None
    msg_type = get_message(type_map[topic])
    best: Optional[Tuple[float, Any, int]] = None
    while reader.has_next():
        tname, data, ts_ns = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, (PoseStamped, PoseWithCovarianceStamped)):
            continue
        hs = stamp_to_sec(msg.header.stamp)
        dt = abs(hs - target_sec)
        if best is None or dt < best[0]:
            best = (dt, msg, ts_ns)
    if best is None:
        return None
    dt, msg, ts_ns = best
    pos, quat = msg_pose_to_arrays(msg)
    cov = extract_covariance(msg)
    out: Dict[str, Any] = {
        "message_type": type(msg).__name__,
        "pose_header_stamp_sec": stamp_to_sec(msg.header.stamp),
        "header_frame_id": getattr(msg.header, "frame_id", "") or "",
        "dt_from_pose_ref_sec": dt,
        "bag_receive_time_ns": int(ts_ns),
        "position": {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])},
        "orientation": {
            "x": float(quat[0]),
            "y": float(quat[1]),
            "z": float(quat[2]),
            "w": float(quat[3]),
        },
    }
    if cov is not None:
        out["covariance"] = cov
    return out


def average_quaternions(quats: np.ndarray) -> np.ndarray:
    if len(quats) == 0:
        raise ValueError("empty quats")
    ref = quats[0].copy()
    aligned = []
    for q in quats:
        q = q.copy()
        if np.dot(q, ref) < 0.0:
            q = -q
        aligned.append(q)
    m = np.mean(np.stack(aligned, axis=0), axis=0)
    n = np.linalg.norm(m)
    if n < 1e-12:
        return ref
    return m / n


def yaw_from_quat_xyzw(q: np.ndarray) -> float:
    return float(R.from_quat(q).as_euler("xyz", degrees=False)[2])


def wrap_angle_rad(a: float) -> float:
    return float(math.atan2(math.sin(a), math.cos(a)))


def deviation_about_mean_pose(
    mean_pos: np.ndarray,
    mean_quat: np.ndarray,
    P: np.ndarray,
    Q: np.ndarray,
) -> Tuple[Dict[str, Any], List[Dict[str, float]]]:
    """
    平均位置・平均四元数を基準に、各 run の縦(前進)・横(左)・ヨー差を求め、
    ばらつき(標準偏差)と最大絶対偏差を返す。
    ヨー基準は平均四元数からの yaw(円平均 yaw とは微小差の可能性あり)。
    """
    n = P.shape[0]
    if n == 0:
        raise ValueError("no runs")
    r_mean = R.from_quat(mean_quat)
    yaw_ref = yaw_from_quat_xyzw(mean_quat)

    longs: List[float] = []
    lats: List[float] = []
    yaw_errs: List[float] = []
    per_run: List[Dict[str, float]] = []

    for i in range(n):
        d_map = P[i] - mean_pos
        d_body = r_mean.inv().apply(d_map)
        lon = float(d_body[0])
        lat = float(d_body[1])
        yi = yaw_from_quat_xyzw(Q[i])
        ye = wrap_angle_rad(yi - yaw_ref)
        longs.append(lon)
        lats.append(lat)
        yaw_errs.append(ye)
        per_run.append(
            {
                "longitudinal_m": lon,
                "lateral_m": lat,
                "yaw_error_rad": ye,
                "yaw_error_deg": float(math.degrees(ye)),
            }
        )

    longs_a = np.array(longs, dtype=float)
    lats_a = np.array(lats, dtype=float)
    yaws_e = np.array(yaw_errs, dtype=float)
    if n > 1:
        std_lon = float(np.std(longs_a, ddof=1))
        std_lat = float(np.std(lats_a, ddof=1))
        std_yaw = float(np.std(yaws_e, ddof=1))
    else:
        std_lon = 0.0
        std_lat = 0.0
        std_yaw = 0.0

    stats: Dict[str, Any] = {
        "description": "平均 pose(位置算術平均・姿勢は四元数平均)を基準。車体軸はその平均四元数(x=前進,y=左)。",
        "reference_yaw_rad": float(yaw_ref),
        "reference_yaw_deg": float(math.degrees(yaw_ref)),
        "longitudinal_m": {
            "std": std_lon,
            "max_abs": float(np.max(np.abs(longs_a))),
        },
        "lateral_m": {
            "std": std_lat,
            "max_abs": float(np.max(np.abs(lats_a))),
        },
        "yaw": {
            "std_rad": std_yaw,
            "std_deg": float(math.degrees(std_yaw)),
            "max_abs_rad": float(np.max(np.abs(yaws_e))),
            "max_abs_deg": float(math.degrees(float(np.max(np.abs(yaws_e))))),
        },
        "horizontal_in_mean_body_m": {
            "max": float(np.max(np.hypot(longs_a, lats_a))),
            "description": "各 run の √(縦²+横²) の最大",
        },
    }
    return stats, per_run


def load_optional_yaml(path: Path) -> Optional[Any]:
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_mean_pose_yaml(
    *,
    initial_pose_yaml: Optional[Path],
    target_unix_sec: float,
    pose_topic: str,
    mean_pos: np.ndarray,
    mean_quat: np.ndarray,
    mean_cov: Optional[np.ndarray],
    frame_id: str,
    per_run: List[Dict[str, Any]],
    align_pointcloud_topic: str,
    exclusion_max_dt: Optional[float] = None,
    n_runs_before_exclusion: int = 0,
    excluded_runs: Optional[List[Dict[str, Any]]] = None,
    deviation_about_mean: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    stamp = float_sec_to_stamp_dict(target_unix_sec)
    pose_block: Dict[str, Any] = {
        "pose": {
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
        }
    }
    if mean_cov is not None:
        pose_block["covariance"] = [float(x) for x in mean_cov.flatten().tolist()]

    doc: Dict[str, Any] = {
        "mean_pose_header_stamp": {
            **stamp,
            "frame_id": frame_id or "map",
        },
        "aggregated": {
            "pose_topic": pose_topic,
            "target_unix_sec": float(target_unix_sec),
            "n_runs": len(per_run),
            "per_run_pose_header_stamp_sec": [r["pose_header_stamp_sec"] for r in per_run],
        },
        "pose": pose_block,
    }
    if exclusion_max_dt is not None:
        agg = doc["aggregated"]
        agg["n_runs_candidates"] = int(n_runs_before_exclusion)
        agg["max_pose_pointcloud_dt_sec_for_mean"] = float(exclusion_max_dt)
        agg["n_runs_excluded_from_mean"] = len(excluded_runs or [])
        if excluded_runs:
            agg["excluded_from_mean"] = excluded_runs
    if align_pointcloud_topic:
        agg = doc["aggregated"]
        agg["align_pointcloud_topic"] = align_pointcloud_topic
        agg["per_run_target_pointcloud_header_stamp_sec"] = [
            float(r["target_pointcloud_header_stamp_sec"]) for r in per_run
        ]
        agg["per_run_pointcloud_header_frame_id"] = [
            str(r["pointcloud_header_frame_id"]) for r in per_run
        ]
        agg["per_run_pointcloud_dt_from_target_sec"] = [
            float(r["dt_pointcloud_header_from_target_sec"]) for r in per_run
        ]
        agg["per_run_pose_dt_from_pointcloud_header_sec"] = [
            float(r["dt_pose_from_pointcloud_header_sec"]) for r in per_run
        ]
        agg["per_run_dt_pose_from_user_target_sec"] = [
            float(r["dt_from_target_sec"]) for r in per_run
        ]
    if deviation_about_mean is not None:
        doc["aggregated"]["deviation_about_mean"] = deviation_about_mean
    if initial_pose_yaml is not None:
        loaded = load_optional_yaml(initial_pose_yaml)
        if loaded is not None:
            doc["initial_pose_source"] = loaded
            doc["initial_pose_source_file"] = str(initial_pose_yaml)
    return doc


def main() -> None:
    p = argparse.ArgumentParser(
        description="複数記録 bag から指定時刻近傍の pose を平均(EKF/NDT 等の PoseWithCovarianceStamped 向け)"
    )
    p.add_argument("--target-unix-sec", type=float, required=True)
    p.add_argument(
        "--pose-topic",
        default="/localization/pose_estimator/pose_with_covariance",
        help="各記録 bag 内の PoseStamped または PoseWithCovarianceStamped",
    )
    p.add_argument(
        "--bags",
        nargs="+",
        required=True,
        help="記録 rosbag2 ディレクトリ(metadata.yaml があるパス)を複数指定",
    )
    p.add_argument(
        "--output-json",
        default="",
        help="集計結果 JSON(省略可)",
    )
    p.add_argument(
        "--output-mean-pose-yaml",
        default="",
        help="set_initial_pose.py 互換の pose ブロック + メタデータを書く YAML",
    )
    p.add_argument(
        "--initial-pose-yaml",
        default="",
        help="元の initial_pose.yaml を読み込み、mean_pose.yaml に initial_pose_source として含める",
    )
    p.add_argument(
        "--mean-pose-frame-id",
        default="map",
        help="mean_pose_header_stamp.frame_id および pose 用の座標系(既定 map)",
    )
    p.add_argument(
        "--align-pointcloud-topic",
        default="",
        help="指定時: 各 bag で点群(PointCloud2)の header が target に最も近いフレームを選び、"
        "その header 時刻に最も近い pose を平均する。空なら従来どおり target に最も近い pose のみ。",
    )
    p.add_argument(
        "--max-pose-pointcloud-dt-sec",
        type=float,
        default=None,
        metavar="SEC",
        help="点群アライン時のみ有効: dt_pose_from_pointcloud_header_sec がこの秒数より大きい run は平均から除外。"
        "例: 0.01〜0.02 は厳しめ、0.05 は緩め(約0.1sズレの欠けを通す可能性あり)。省略時は除外しない。",
    )
    args = p.parse_args()

    align_pc = bool(args.align_pointcloud_topic)
    max_dt_thr: Optional[float] = args.max_pose_pointcloud_dt_sec
    if max_dt_thr is not None and not align_pc:
        print(
            "Warning: --max-pose-pointcloud-dt-sec は --align-pointcloud-topic 指定時のみ有効です。無視します。",
            file=sys.stderr,
        )
        max_dt_thr = None

    candidate_rows: List[Dict[str, Any]] = []

    for raw in args.bags:
        bag_dir = find_rosbag2_directory(raw)
        ref_sec = float(args.target_unix_sec)
        pc_info: Optional[Dict[str, Any]] = None
        if align_pc:
            pc_info = extract_nearest_pointcloud_header(
                bag_dir, args.align_pointcloud_topic, float(args.target_unix_sec)
            )
            if pc_info is None:
                print(
                    json.dumps(
                        {
                            "status": "error",
                            "reason": f"no PointCloud2 on {args.align_pointcloud_topic}",
                            "bag": bag_dir,
                        },
                        indent=2,
                    )
                )
                sys.exit(2)
            ref_sec = float(pc_info["target_pointcloud_header_stamp_sec"])

        row = extract_nearest_pose(bag_dir, args.pose_topic, ref_sec)
        if row is None:
            print(
                json.dumps(
                    {
                        "status": "error",
                        "reason": f"no pose on {args.pose_topic}",
                        "bag": bag_dir,
                    },
                    indent=2,
                )
            )
            sys.exit(2)
        dt_pr = float(row.pop("dt_from_pose_ref_sec"))
        row["pose_ref_unix_sec"] = ref_sec
        row["dt_from_target_sec"] = abs(
            float(row["pose_header_stamp_sec"]) - float(args.target_unix_sec)
        )
        if align_pc:
            row.update(pc_info)
            row["dt_pose_from_pointcloud_header_sec"] = dt_pr
        else:
            row["dt_pose_from_pointcloud_header_sec"] = dt_pr
        row["bag_dir"] = bag_dir
        candidate_rows.append(row)

    n_candidates = len(candidate_rows)
    excluded: List[Dict[str, Any]] = []
    per_run: List[Dict[str, Any]] = []
    for row in candidate_rows:
        if max_dt_thr is not None:
            dt_pc = float(row["dt_pose_from_pointcloud_header_sec"])
            if dt_pc > max_dt_thr:
                excluded.append(
                    {
                        "bag_dir": row["bag_dir"],
                        "dt_pose_from_pointcloud_header_sec": dt_pc,
                        "pose_header_stamp_sec": row["pose_header_stamp_sec"],
                        "target_pointcloud_header_stamp_sec": row.get(
                            "target_pointcloud_header_stamp_sec"
                        ),
                    }
                )
                continue
        per_run.append(row)

    if not per_run:
        print(
            json.dumps(
                {
                    "status": "error",
                    "reason": "平均に使える run が0件(全 run が dt_pose_from_pointcloud_header_sec 閾値超過、"
                    "または bags が空)",
                    "n_candidates": n_candidates,
                    "max_pose_pointcloud_dt_sec": max_dt_thr,
                    "excluded": excluded,
                },
                indent=2,
            )
        )
        sys.exit(3)

    if excluded:
        print(
            f"Info: {len(excluded)} run を平均から除外しました(dt_pose_from_pointcloud_header_sec > "
            f"{max_dt_thr})。残り {len(per_run)} run。",
            file=sys.stderr,
        )

    positions: List[np.ndarray] = []
    quats: List[np.ndarray] = []
    covs: List[np.ndarray] = []
    for row in per_run:
        positions.append(
            np.array(
                [
                    row["position"]["x"],
                    row["position"]["y"],
                    row["position"]["z"],
                ],
                dtype=float,
            )
        )
        quats.append(
            np.array(
                [
                    row["orientation"]["x"],
                    row["orientation"]["y"],
                    row["orientation"]["z"],
                    row["orientation"]["w"],
                ],
                dtype=float,
            )
        )
        if "covariance" in row:
            covs.append(np.array(row["covariance"], dtype=float))

    P = np.stack(positions, axis=0)
    mean_pos = np.mean(P, axis=0)
    std_pos = np.std(P, axis=0)
    Q = np.stack(quats, axis=0)
    mean_quat = average_quaternions(Q)
    yaws = np.array([yaw_from_quat_xyzw(q) for q in Q])
    mean_yaw = math.atan2(
        float(np.mean(np.sin(yaws))), float(np.mean(np.cos(yaws)))
    )

    mean_cov: Optional[np.ndarray] = None
    if len(covs) == len(per_run) and len(covs) > 0:
        mean_cov = np.mean(np.stack(covs, axis=0), axis=0)
    elif covs:
        print(
            "Warning: 一部の run にのみ covariance がありました。平均共分散は省略します。",
            file=sys.stderr,
        )

    dev_stats, dev_per_run = deviation_about_mean_pose(mean_pos, mean_quat, P, Q)
    yaml_deviation: Dict[str, Any] = {**dev_stats}
    yaml_deviation["per_run_longitudinal_m"] = [x["longitudinal_m"] for x in dev_per_run]
    yaml_deviation["per_run_lateral_m"] = [x["lateral_m"] for x in dev_per_run]
    yaml_deviation["per_run_yaw_error_rad"] = [x["yaw_error_rad"] for x in dev_per_run]
    yaml_deviation["per_run_yaw_error_deg"] = [x["yaw_error_deg"] for x in dev_per_run]

    for i, row in enumerate(per_run):
        row["deviation_about_mean"] = dev_per_run[i]

    out: Dict[str, Any] = {
        "status": "ok",
        "n_runs": len(per_run),
        "n_runs_candidates": n_candidates,
        "target_unix_sec": args.target_unix_sec,
        "pose_topic": args.pose_topic,
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
    if max_dt_thr is not None:
        out["max_pose_pointcloud_dt_sec_for_mean"] = max_dt_thr
    if excluded:
        out["excluded_from_mean"] = excluded
    if align_pc:
        out["align_pointcloud_topic"] = args.align_pointcloud_topic
    if mean_cov is not None:
        out["mean_covariance"] = [float(x) for x in mean_cov.tolist()]

    text = json.dumps(out, indent=2)
    print(text)
    if args.output_json:
        Path(args.output_json).write_text(text, encoding="utf-8")

    if args.output_mean_pose_yaml:
        init_path = Path(args.initial_pose_yaml) if args.initial_pose_yaml else None
        yaml_doc = build_mean_pose_yaml(
            initial_pose_yaml=init_path,
            target_unix_sec=args.target_unix_sec,
            pose_topic=args.pose_topic,
            mean_pos=mean_pos,
            mean_quat=mean_quat,
            mean_cov=mean_cov,
            frame_id=args.mean_pose_frame_id,
            per_run=per_run,
            align_pointcloud_topic=args.align_pointcloud_topic if align_pc else "",
            exclusion_max_dt=max_dt_thr,
            n_runs_before_exclusion=n_candidates,
            excluded_runs=excluded if excluded else None,
            deviation_about_mean=yaml_deviation,
        )
        yaml_text = yaml.dump(
            yaml_doc,
            default_flow_style=False,
            allow_unicode=True,
            sort_keys=False,
        )
        Path(args.output_mean_pose_yaml).write_text(yaml_text, encoding="utf-8")
        print(f"Wrote mean pose YAML: {args.output_mean_pose_yaml}", file=sys.stderr)


if __name__ == "__main__":
    main()
