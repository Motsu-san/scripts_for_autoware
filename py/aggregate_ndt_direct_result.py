#!/usr/bin/env python3
"""Aggregate repeated ndt_direct_measure results.

`ndt_direct_measure_node --n-runs N` writes one JSON containing per-run
`scan_matching_pose`.  This script computes the mean pose, positional variance,
standard deviation, and max deviation around the mean pose.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from aggregate_pose_mean_from_bags import (  # noqa: E402
    average_quaternions,
    build_mean_pose_yaml,
    deviation_about_mean_pose,
    yaw_from_quat_xyzw,
)


def _pose_to_arrays(pose: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    pos = pose["position"]
    ori = pose["orientation"]
    p = np.array([pos["x"], pos["y"], pos["z"]], dtype=float)
    q = np.array([ori["x"], ori["y"], ori["z"], ori["w"]], dtype=float)
    n = np.linalg.norm(q)
    if n > 1e-12:
        q /= n
    return p, q


def _pose_dict(pos: np.ndarray, quat: np.ndarray) -> Dict[str, Any]:
    return {
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


def _score_stats(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}
    a = np.array(values, dtype=float)
    return {
        "mean": float(np.mean(a)),
        "std": float(np.std(a, ddof=1)) if len(a) > 1 else 0.0,
        "min": float(np.min(a)),
        "max": float(np.max(a)),
    }


def _aggregate_scan_runs(
    runs: List[Dict[str, Any]],
    *,
    pc_header: Dict[str, Any],
    offset_from_nearest: int,
) -> Dict[str, Any]:
    if not runs:
        return {
            "offset_from_nearest": offset_from_nearest,
            "pointcloud_header": pc_header,
            "n_runs": 0,
            "converged_count": 0,
            "score_threshold_pass_count": 0,
        }

    positions: List[np.ndarray] = []
    quats: List[np.ndarray] = []
    for run in runs:
        pos, quat = _pose_to_arrays(run["scan_matching_pose"])
        positions.append(pos)
        quats.append(quat)

    P = np.stack(positions, axis=0)
    Q = np.stack(quats, axis=0)
    mean_pos = np.mean(P, axis=0)
    mean_quat = average_quaternions(Q)
    dev_stats, dev_per_run = deviation_about_mean_pose(mean_pos, mean_quat, P, Q)

    score_nvtl = [float(r.get("score_nvtl", 0.0)) for r in runs]
    score_tp = [float(r.get("score_tp", 0.0)) for r in runs]
    iterations = [float(r.get("iteration", 0)) for r in runs]

    return {
        "offset_from_nearest": offset_from_nearest,
        "pointcloud_header": pc_header,
        "n_runs": len(runs),
        "mean_pose": _pose_dict(mean_pos, mean_quat),
        "position_std_xyz_m": {
            "x": float(np.std(P[:, 0])),
            "y": float(np.std(P[:, 1])),
            "z": float(np.std(P[:, 2])),
        },
        "deviation_about_mean": {**dev_stats, "per_run": dev_per_run},
        "score_stats": {
            "score_nvtl": _score_stats(score_nvtl),
            "score_tp": _score_stats(score_tp),
            "iteration": _score_stats(iterations),
            "converged_count": int(
                sum(bool(r.get("has_converged", False)) for r in runs)
            ),
            "score_threshold_pass_count": int(
                sum(bool(r.get("passes_score_threshold", False)) for r in runs)
            ),
        },
    }


def aggregate_direct_result(
    data: Dict[str, Any],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    runs = list(data.get("per_run") or [])
    if not runs:
        raise ValueError("input JSON has no per_run entries")

    positions: List[np.ndarray] = []
    quats: List[np.ndarray] = []
    per_run_for_yaml: List[Dict[str, Any]] = []

    pc_header = data.get("pointcloud_header", {})
    pc_stamp = float(
        pc_header.get("stamp_sec", data.get("target_unix_sec", 0.0))
    )
    pc_frame = str(pc_header.get("frame_id", ""))
    pc_dt = float(pc_header.get("dt_from_target_sec", 0.0))

    for run in runs:
        pos, quat = _pose_to_arrays(run["scan_matching_pose"])
        positions.append(pos)
        quats.append(quat)
        per_run_for_yaml.append(
            {
                "run_index": int(run.get("run_index", len(per_run_for_yaml))),
                "pose_header_stamp_sec": pc_stamp,
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
                "target_pointcloud_header_stamp_sec": pc_stamp,
                "pointcloud_header_frame_id": pc_frame,
                "dt_pointcloud_header_from_target_sec": pc_dt,
                "dt_pose_from_pointcloud_header_sec": 0.0,
                "dt_from_target_sec": pc_dt,
                "has_converged": bool(run.get("has_converged", False)),
                "passes_score_threshold": bool(
                    run.get("passes_score_threshold", False)
                ),
                "iteration": int(run.get("iteration", 0)),
                "score_nvtl": float(run.get("score_nvtl", 0.0)),
                "score_tp": float(run.get("score_tp", 0.0)),
            }
        )

    P = np.stack(positions, axis=0)
    Q = np.stack(quats, axis=0)
    mean_pos = np.mean(P, axis=0)
    mean_quat = average_quaternions(Q)
    mean_yaw = yaw_from_quat_xyzw(mean_quat)

    # Population variance/std describes the repeated outputs themselves.
    var_pos = np.var(P, axis=0)
    std_pos = np.std(P, axis=0)
    dev_stats, dev_per_run = deviation_about_mean_pose(
        mean_pos, mean_quat, P, Q
    )
    for row, dev in zip(per_run_for_yaml, dev_per_run):
        row["deviation_about_mean"] = dev

    score_nvtl = [float(r.get("score_nvtl", 0.0)) for r in runs]
    score_tp = [float(r.get("score_tp", 0.0)) for r in runs]
    iterations = [float(r.get("iteration", 0)) for r in runs]

    result = dict(data)
    result["method"] = "ndt_direct_align_single_scan_mean"
    result["n_runs"] = len(runs)
    result["mean_pose"] = {
        **_pose_dict(mean_pos, mean_quat),
        "yaw_rad_circular_mean_from_each_run": float(
            math.atan2(
                float(np.mean(np.sin([yaw_from_quat_xyzw(q) for q in Q]))),
                float(np.mean(np.cos([yaw_from_quat_xyzw(q) for q in Q]))),
            )
        ),
        "yaw_deg_from_mean_quaternion": float(math.degrees(mean_yaw)),
    }
    result["position_variance_xyz_m2"] = {
        "x": float(var_pos[0]),
        "y": float(var_pos[1]),
        "z": float(var_pos[2]),
    }
    result["position_std_xyz_m"] = {
        "x": float(std_pos[0]),
        "y": float(std_pos[1]),
        "z": float(std_pos[2]),
    }
    result["deviation_about_mean"] = {**dev_stats, "per_run": dev_per_run}
    result["score_stats"] = {
        "score_nvtl": _score_stats(score_nvtl),
        "score_tp": _score_stats(score_tp),
        "iteration": _score_stats(iterations),
        "converged_count": int(
            sum(bool(r.get("has_converged", False)) for r in runs)
        ),
        "score_threshold_pass_count": int(
            sum(bool(r.get("passes_score_threshold", False)) for r in runs)
        ),
    }

    per_scan = list(data.get("per_scan") or [])
    if per_scan:
        result["neighbor_scans"] = int(data.get("neighbor_scans", 0))
        result["per_scan_summary"] = [
            {
                **_aggregate_scan_runs(
                    list(scan.get("per_run") or []),
                    pc_header=dict(scan.get("pointcloud_header") or {}),
                    offset_from_nearest=int(scan.get("offset_from_nearest", 0)),
                ),
                "scan_initial_pose": scan.get("scan_initial_pose"),
            }
            for scan in per_scan
        ]

    yaml_doc = build_mean_pose_yaml(
        initial_pose_yaml=Path(str(data["initial_pose_yaml"]))
        if data.get("initial_pose_yaml")
        else None,
        target_unix_sec=float(data.get("target_unix_sec", pc_stamp)),
        pose_topic="/localization/pose_estimator/pose_with_covariance",
        mean_pos=mean_pos,
        mean_quat=mean_quat,
        mean_cov=None,
        frame_id="map",
        per_run=per_run_for_yaml,
        align_pointcloud_topic=str(data.get("pointcloud_topic", "")),
        deviation_about_mean={**dev_stats, "per_run": dev_per_run},
    )
    yaml_doc["aggregated"]["method"] = "ndt_direct_align_single_scan_mean"
    yaml_doc["aggregated"]["position_variance_xyz_m2"] = result[
        "position_variance_xyz_m2"
    ]
    yaml_doc["aggregated"]["position_std_xyz_m"] = result["position_std_xyz_m"]
    yaml_doc["aggregated"]["score_stats"] = result["score_stats"]
    yaml_doc["aggregated"]["map_load"] = data.get("map_load", {})
    if result.get("per_scan_summary"):
        yaml_doc["aggregated"]["neighbor_scans"] = result.get("neighbor_scans", 0)
        yaml_doc["aggregated"]["per_scan_summary"] = result["per_scan_summary"]

    return result, yaml_doc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Aggregate ndt_direct_measure JSON into mean pose and deviation stats"
    )
    parser.add_argument("--input-json", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-mean-pose-yaml", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    in_path = Path(args.input_json)
    data = json.loads(in_path.read_text(encoding="utf-8"))
    result, yaml_doc = aggregate_direct_result(data)

    out_json = Path(args.output_json)
    out_json.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    out_yaml = Path(args.output_mean_pose_yaml)
    out_yaml.write_text(
        yaml.dump(yaml_doc, default_flow_style=False, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    print(
        "mean pose: "
        f"x={result['mean_pose']['position']['x']:.6f} "
        f"y={result['mean_pose']['position']['y']:.6f} "
        f"z={result['mean_pose']['position']['z']:.6f} "
        f"yaw={result['mean_pose']['yaw_deg_from_mean_quaternion']:.6f}deg"
    )
    dam = result["deviation_about_mean"]
    print(
        "max deviation about mean: "
        f"longitudinal={dam['longitudinal_m']['max_abs']:.6f}m "
        f"lateral={dam['lateral_m']['max_abs']:.6f}m "
        f"horizontal={dam['horizontal_in_mean_body_m']['max']:.6f}m "
        f"yaw={dam['yaw']['max_abs_deg']:.6f}deg"
    )
    print(f"Wrote: {out_json}")
    print(f"Wrote: {out_yaml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
