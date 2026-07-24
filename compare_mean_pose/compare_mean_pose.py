#!/usr/bin/env python3
"""
ndt_mean_pose.yaml(NDT 平均姿勢)を基準に、mean_pose 集計 JSON の各 run 測定 pose との
縦(前後)・横(左右)・ヨー差を求め、平均・ばらつき(標準偏差)・最大絶対偏差を YAML に出力する。

per_scan_summary に前後スキャンの参照 pose がある場合、各 run の測定 pose の pose_header_stamp_sec に
最も近い参照 pose を基準に差分を計算する(指定時刻のルート pose のみを使わない)。

dt_pose_from_pointcloud_header_sec が --max-pose-pointcloud-dt-sec(既定 0.21)を超える run は
平均・標準偏差・合格率などの集計から除外する(measure_pose_mean の MAX_POSE_POINTCLOUD_DT_SEC と同義)。

基準の車体座標系: x=前進、y=左(各参照 pose の姿勢で定義)
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
from scipy.spatial.transform import Rotation as R

LONGITUDINAL_PASS_THRESHOLDS_M = (0.3, 0.5, 1.0)


def longitudinal_pass_stats(
    longs_a: np.ndarray, thresholds_m: tuple[float, ...] = LONGITUDINAL_PASS_THRESHOLDS_M
) -> Dict[str, Any]:
    """各閾値での縦誤差合格率を返す。"""
    n = int(longs_a.size)
    abs_longs = np.abs(longs_a)
    by_threshold: Dict[str, Any] = {}
    for thr in thresholds_m:
        n_pass = int(np.sum(abs_longs <= thr))
        success_rate = float(n_pass / n) if n else 0.0
        key = f"{thr:g}"
        by_threshold[key] = {
            "max_abs_m": thr,
            "n_pass": n_pass,
            "n_fail": n - n_pass,
            "success_rate": success_rate,
            "success_rate_percent": float(success_rate * 100.0),
        }
    return {
        "description": "縦(前後)誤差の絶対値が閾値以下で合格",
        "thresholds_m": list(thresholds_m),
        "by_threshold_m": by_threshold,
    }


def pose_dict_to_arrays(pose_block: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    pos = pose_block["position"]
    ori = pose_block["orientation"]
    pos_v = np.array([float(pos["x"]), float(pos["y"]), float(pos["z"])], dtype=float)
    quat = np.array(
        [float(ori["x"]), float(ori["y"]), float(ori["z"]), float(ori["w"])], dtype=float
    )
    n = np.linalg.norm(quat)
    if n < 1e-12:
        raise ValueError("orientation のノルムがゼロに近いです")
    quat /= n
    return pos_v, quat


def load_ndt_scan_reference_poses(doc: Dict[str, Any]) -> List[Dict[str, Any]]:
    """aggregated.per_scan_summary から前後スキャン時刻の参照 mean_pose を読む。"""
    agg = doc.get("aggregated") or {}
    per_scan = agg.get("per_scan_summary") or []
    refs: List[Dict[str, Any]] = []
    for scan in per_scan:
        if not isinstance(scan, dict):
            continue
        pc_hdr = scan.get("pointcloud_header") or {}
        stamp = pc_hdr.get("stamp_sec")
        mean_pose = scan.get("mean_pose")
        if stamp is None or not isinstance(mean_pose, dict):
            continue
        pos_v, quat = pose_dict_to_arrays(mean_pose)
        refs.append(
            {
                "stamp_sec": float(stamp),
                "offset_from_nearest": scan.get("offset_from_nearest"),
                "position": pos_v,
                "quaternion": quat,
            }
        )
    refs.sort(key=lambda r: r["stamp_sec"])
    return refs


def run_pose_header_stamp_sec(run: Dict[str, Any]) -> float | None:
    """比較に使う run 側のヘッダ時刻(測定 pose の pose_header_stamp_sec)。"""
    if "pose_header_stamp_sec" in run:
        return float(run["pose_header_stamp_sec"])
    return None


def run_pose_pointcloud_dt_sec(run: Dict[str, Any]) -> float | None:
    """pose と点群 header の時刻差(aggregate_pose_mean_from_bags と同じ指標)。"""
    if "dt_pose_from_pointcloud_header_sec" in run:
        return float(run["dt_pose_from_pointcloud_header_sec"])
    if "dt_from_target_sec" in run:
        return float(run["dt_from_target_sec"])
    return None


def split_runs_by_max_pose_pointcloud_dt(
    per_run: List[Dict[str, Any]], max_dt: float | None
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """dt_pose_from_pointcloud_header_sec が max_dt 超の run を集計対象から除外する。"""
    if max_dt is None:
        return list(per_run), []
    included: List[Dict[str, Any]] = []
    excluded: List[Dict[str, Any]] = []
    for i, run in enumerate(per_run):
        dt = run_pose_pointcloud_dt_sec(run)
        if dt is None:
            excluded.append(
                {
                    "run_index": int(run.get("run_index", i)),
                    "reason": "missing_dt_pose_from_pointcloud_header_sec",
                    "bag_dir": run.get("bag_dir"),
                }
            )
            continue
        if dt > max_dt:
            excluded.append(
                {
                    "run_index": int(run.get("run_index", i)),
                    "dt_pose_from_pointcloud_header_sec": float(dt),
                    "pose_header_stamp_sec": run.get("pose_header_stamp_sec"),
                    "target_pointcloud_header_stamp_sec": run.get(
                        "target_pointcloud_header_stamp_sec"
                    ),
                    "bag_dir": run.get("bag_dir"),
                }
            )
            continue
        included.append(run)
    return included, excluded


def pick_nearest_scan_reference(
    refs: List[Dict[str, Any]], stamp_sec: float
) -> Tuple[int, Dict[str, Any]]:
    best_i = 0
    best_dt = abs(float(refs[0]["stamp_sec"]) - stamp_sec)
    for i, ref in enumerate(refs[1:], start=1):
        dt = abs(float(ref["stamp_sec"]) - stamp_sec)
        if dt < best_dt:
            best_dt = dt
            best_i = i
    return best_i, refs[best_i]


def load_ndt_mean_pose_block(path: Path) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """mean_pose.yaml 形式(ルート pose)または ndt_mean_pose キーから基準 pose を読む。"""
    with path.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    if doc is None:
        raise ValueError(f"{path}: 空の YAML です")
    if "ndt_mean_pose" in doc:
        block = doc["ndt_mean_pose"]
        if "pose" in block:
            pp = block["pose"]
        else:
            pp = block
    elif "pose" in doc:
        pblock = doc["pose"]
        if "pose" not in pblock:
            raise ValueError(f"{path}: pose.pose がありません")
        pp = pblock["pose"]
    else:
        raise ValueError(
            f"{path}: ルートに pose または ndt_mean_pose がありません(ndt mean_pose.yaml 想定)"
        )
    pos_v, quat = pose_dict_to_arrays(pp)
    meta = {
        "path": str(path),
        "aggregated": doc.get("aggregated"),
        "mean_pose_header_stamp": doc.get("mean_pose_header_stamp"),
        "scan_reference_poses": load_ndt_scan_reference_poses(doc),
    }
    return pos_v, quat, meta


def load_mean_pose_per_run(path: Path) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """aggregate_pose_mean_from_bags 等の JSON から per_run 測定 pose を読む。"""
    with path.open("r", encoding="utf-8") as f:
        doc = json.load(f)
    if doc is None:
        raise ValueError(f"{path}: 空の JSON です")
    per_run = doc.get("per_run")
    if not per_run:
        raise ValueError(f"{path}: per_run がありません(mean_pose 集計 JSON 想定)")
    rows: List[Dict[str, Any]] = []
    for i, run in enumerate(per_run):
        if "position" not in run or "orientation" not in run:
            raise ValueError(f"{path}: per_run[{i}] に position/orientation がありません")
        rows.append(run)
    meta = {
        "path": str(path),
        "status": doc.get("status"),
        "n_runs": doc.get("n_runs"),
        "target_unix_sec": doc.get("target_unix_sec"),
        "pose_topic": doc.get("pose_topic"),
        "mean_pose": doc.get("mean_pose"),
    }
    return rows, meta


def yaw_from_quat_xyzw(q: np.ndarray) -> float:
    return float(R.from_quat(q).as_euler("xyz", degrees=False)[2])


def wrap_angle_rad(a: float) -> float:
    return float(math.atan2(math.sin(a), math.cos(a)))


def deviation_about_ndt_mean_pose(
    ndt_pos: np.ndarray,
    ndt_quat: np.ndarray,
    per_run: List[Dict[str, Any]],
    *,
    scan_reference_poses: List[Dict[str, Any]] | None = None,
    max_pose_pointcloud_dt_sec: float | None = None,
    n_runs_candidates: int | None = None,
    excluded_from_comparison: List[Dict[str, Any]] | None = None,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """ndt_mean_pose を基準に各 run の縦・横・ヨー差と統計量を返す。

    scan_reference_poses がある場合、各 run のヘッダ時刻に最も近い
    per_scan_summary の参照 pose を基準に差分を計算する。
    """
    n = len(per_run)
    if n == 0:
        raise ValueError("per_run が空です")
    scan_refs = list(scan_reference_poses or [])
    use_scan_refs = len(scan_refs) > 0

    longs: List[float] = []
    lats: List[float] = []
    yaw_errs: List[float] = []
    per_run_out: List[Dict[str, Any]] = []
    n_matched_by_scan = 0

    for i, run in enumerate(per_run):
        pos = run["position"]
        ori = run["orientation"]
        pos_v = np.array([float(pos["x"]), float(pos["y"]), float(pos["z"])], dtype=float)
        quat = np.array(
            [float(ori["x"]), float(ori["y"]), float(ori["z"]), float(ori["w"])],
            dtype=float,
        )
        qn = np.linalg.norm(quat)
        if qn < 1e-12:
            raise ValueError(f"per_run[{i}]: orientation のノルムがゼロに近いです")
        quat /= qn

        ref_pos = ndt_pos
        ref_quat = ndt_quat
        ref_meta: Dict[str, Any] = {"source": "ndt_mean_pose_root"}
        run_stamp = run_pose_header_stamp_sec(run)
        if use_scan_refs and run_stamp is not None:
            ref_i, ref = pick_nearest_scan_reference(scan_refs, run_stamp)
            ref_pos = ref["position"]
            ref_quat = ref["quaternion"]
            ref_meta = {
                "source": "per_scan_summary",
                "scan_reference_index": ref_i,
                "ndt_reference_stamp_sec": float(ref["stamp_sec"]),
                "dt_from_ndt_reference_sec": abs(float(ref["stamp_sec"]) - run_stamp),
            }
            if ref.get("offset_from_nearest") is not None:
                ref_meta["ndt_reference_offset_from_nearest"] = int(
                    ref["offset_from_nearest"]
                )
            n_matched_by_scan += 1

        r_ref = R.from_quat(ref_quat)
        yaw_ref = yaw_from_quat_xyzw(ref_quat)
        d_map = pos_v - ref_pos
        d_body = r_ref.inv().apply(d_map)
        lon = float(d_body[0])
        lat = float(d_body[1])
        ye = wrap_angle_rad(yaw_from_quat_xyzw(quat) - yaw_ref)
        longs.append(lon)
        lats.append(lat)
        yaw_errs.append(ye)
        abs_lon = abs(lon)
        pass_by_thr = {
            f"{thr:g}": abs_lon <= thr for thr in LONGITUDINAL_PASS_THRESHOLDS_M
        }
        entry: Dict[str, Any] = {
            "run_index": int(run.get("run_index", i)),
            "longitudinal_m": lon,
            "lateral_m": lat,
            "yaw_error_rad": ye,
            "yaw_error_deg": float(math.degrees(ye)),
            "longitudinal_pass_by_threshold_m": pass_by_thr,
            "longitudinal_pass": pass_by_thr[f"{LONGITUDINAL_PASS_THRESHOLDS_M[0]:g}"],
            **ref_meta,
        }
        if run_stamp is not None:
            entry["run_pose_header_stamp_sec"] = run_stamp
        if "pose_header_stamp_sec" in run:
            entry["pose_header_stamp_sec"] = float(run["pose_header_stamp_sec"])
        if "target_pointcloud_header_stamp_sec" in run:
            entry["target_pointcloud_header_stamp_sec"] = float(
                run["target_pointcloud_header_stamp_sec"]
            )
        if "bag_dir" in run:
            entry["bag_dir"] = str(run["bag_dir"])
        per_run_out.append(entry)

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

    yaw_ref_root = yaw_from_quat_xyzw(ndt_quat)
    if use_scan_refs:
        description = (
            "ndt_mean_pose.yaml の aggregated.per_scan_summary にある参照 pose を基準。"
            "各 run は測定 pose の pose_header_stamp_sec に最も近い参照 pose を選び、"
            "その車体軸(x=前進,y=左)で縦・横・ヨー誤差を計算。"
            "pose_header_stamp_sec が無い run はルート ndt_mean_pose を使用。"
        )
    else:
        description = (
            "ndt_mean_pose を基準。車体軸はその姿勢(x=前進,y=左)。"
            "各値は mean_pose 集計 JSON の per_run 測定 pose 相対の縦・横・ヨー誤差。"
        )

    n_candidates = int(n_runs_candidates if n_runs_candidates is not None else n)
    stats: Dict[str, Any] = {
        "description": description,
        "reference_yaw_rad": float(yaw_ref_root),
        "reference_yaw_deg": float(math.degrees(yaw_ref_root)),
        "n_runs": n,
        "n_runs_candidates": n_candidates,
        "longitudinal_m": {
            "mean": float(np.mean(longs_a)),
            "std": std_lon,
            "max_abs": float(np.max(np.abs(longs_a))),
        },
        "lateral_m": {
            "mean": float(np.mean(lats_a)),
            "std": std_lat,
            "max_abs": float(np.max(np.abs(lats_a))),
        },
        "yaw": {
            "mean_rad": float(np.mean(yaws_e)),
            "mean_deg": float(math.degrees(float(np.mean(yaws_e)))),
            "std_rad": std_yaw,
            "std_deg": float(math.degrees(std_yaw)),
            "max_abs_rad": float(np.max(np.abs(yaws_e))),
            "max_abs_deg": float(math.degrees(float(np.max(np.abs(yaws_e))))),
        },
        "horizontal_in_ndt_body_m": {
            "mean": float(np.mean(np.hypot(longs_a, lats_a))),
            "max": float(np.max(np.hypot(longs_a, lats_a))),
            "description": "各 run の √(縦²+横²)",
        },
        "longitudinal_pass_criteria": longitudinal_pass_stats(longs_a),
        "per_run_longitudinal_m": longs,
        "per_run_lateral_m": lats,
        "per_run_yaw_error_rad": yaw_errs,
        "per_run_yaw_error_deg": [float(math.degrees(y)) for y in yaw_errs],
        "per_run": per_run_out,
    }
    if max_pose_pointcloud_dt_sec is not None:
        stats["max_pose_pointcloud_dt_sec_for_comparison"] = float(
            max_pose_pointcloud_dt_sec
        )
        stats["n_runs_excluded_from_comparison"] = len(excluded_from_comparison or [])
        if excluded_from_comparison:
            stats["excluded_from_comparison"] = excluded_from_comparison
    if use_scan_refs:
        stats["reference_pose_selection"] = {
            "method": "nearest_per_scan_by_pose_header_stamp",
            "n_scan_references": len(scan_refs),
            "n_runs_matched_to_scan_reference": n_matched_by_scan,
            "scan_reference_stamp_sec": [float(r["stamp_sec"]) for r in scan_refs],
        }
    return stats, per_run_out


def build_output_yaml(
    *,
    ndt_yaml: Path,
    mean_pose_json: Path,
    yaml_out: Path,
    ndt_pos: np.ndarray,
    ndt_quat: np.ndarray,
    ndt_meta: Dict[str, Any],
    mean_meta: Dict[str, Any],
    deviation: Dict[str, Any],
) -> Dict[str, Any]:
    yaw_ref = deviation["reference_yaw_rad"]
    doc: Dict[str, Any] = {
        "comparison_vs_ndt_mean_pose": {
            **deviation,
            "ndt_mean_pose_yaml": str(ndt_yaml.resolve()),
            "mean_pose_json": str(mean_pose_json.resolve()),
            "output_yaml": str(yaml_out.resolve()),
        },
        "ndt_mean_pose": {
            "position": {
                "x": float(ndt_pos[0]),
                "y": float(ndt_pos[1]),
                "z": float(ndt_pos[2]),
            },
            "orientation": {
                "x": float(ndt_quat[0]),
                "y": float(ndt_quat[1]),
                "z": float(ndt_quat[2]),
                "w": float(ndt_quat[3]),
            },
            "yaw_rad": float(yaw_ref),
            "yaw_deg": float(math.degrees(yaw_ref)),
        },
    }
    agg = ndt_meta.get("aggregated")
    if isinstance(agg, dict):
        doc["ndt_mean_pose_aggregated"] = {
            k: agg[k]
            for k in ("pose_topic", "target_unix_sec", "n_runs", "method")
            if k in agg
        }
    if ndt_meta.get("mean_pose_header_stamp"):
        doc["ndt_mean_pose_header_stamp"] = ndt_meta["mean_pose_header_stamp"]
    if mean_meta.get("target_unix_sec") is not None:
        doc["mean_pose_target_unix_sec"] = mean_meta["target_unix_sec"]
    if mean_meta.get("pose_topic"):
        doc["mean_pose_topic"] = mean_meta["pose_topic"]
    if mean_meta.get("mean_pose"):
        doc["mean_pose_aggregate"] = mean_meta["mean_pose"]
    return doc


def default_yaml_out_path(mean_pose_json: Path) -> Path:
    stem = mean_pose_json.stem
    return mean_pose_json.with_name(f"{stem}_vs_ndt_mean_pose.yaml")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=(
            "ndt_mean_pose.yaml を基準に mean_pose 集計 JSON の各 run との"
            "縦・横・ヨー誤差の平均・ばらつきを YAML に出力"
        )
    )
    ap.add_argument(
        "ndt_mean_pose_yaml",
        type=Path,
        help="基準となる NDT 平均姿勢 YAML(ルート pose または ndt_mean_pose)",
    )
    ap.add_argument(
        "mean_pose_json",
        type=Path,
        help="mean_pose 各 run 測定を含む集計 JSON(aggregate_pose_mean_from_bags 出力)",
    )
    ap.add_argument(
        "--yaml-out",
        type=Path,
        default=None,
        help="結果 YAML の出力パス(省略時は mean_pose_json と同ディレクトリに *_vs_ndt_mean_pose.yaml)",
    )
    ap.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="同内容を JSON でも書く(省略可)",
    )
    ap.add_argument(
        "--max-pose-pointcloud-dt-sec",
        type=float,
        default=0.21,
        metavar="SEC",
        help="dt_pose_from_pointcloud_header_sec がこの秒数より大きい run は集計から除外。"
        "既定: 0.21(measure_pose_mean の MAX_POSE_POINTCLOUD_DT_SEC と同義)。"
        "除外なしにする場合は --no-pose-pointcloud-dt-filter を指定。",
    )
    ap.add_argument(
        "--no-pose-pointcloud-dt-filter",
        action="store_true",
        help="pose/点群 dt による run 除外を行わない",
    )
    args = ap.parse_args()

    max_dt_thr: float | None = None if args.no_pose_pointcloud_dt_filter else args.max_pose_pointcloud_dt_sec

    ndt_pos, ndt_quat, ndt_meta = load_ndt_mean_pose_block(args.ndt_mean_pose_yaml)
    per_run_all, mean_meta = load_mean_pose_per_run(args.mean_pose_json)
    n_candidates = len(per_run_all)
    per_run, excluded = split_runs_by_max_pose_pointcloud_dt(per_run_all, max_dt_thr)
    if excluded and max_dt_thr is not None:
        print(
            f"Info: {len(excluded)} run を比較集計から除外しました"
            f"(dt_pose_from_pointcloud_header_sec > {max_dt_thr})。"
            f"残り {len(per_run)} run。",
            file=sys.stderr,
        )
    if not per_run:
        print(
            json.dumps(
                {
                    "status": "error",
                    "reason": "比較に使える run が0件(全 run が dt_pose_from_pointcloud_header_sec 閾値超過、"
                    "または dt フィールド欠落)",
                    "n_candidates": n_candidates,
                    "max_pose_pointcloud_dt_sec_for_comparison": max_dt_thr,
                    "excluded_from_comparison": excluded,
                },
                indent=2,
            ),
            file=sys.stderr,
        )
        sys.exit(3)

    scan_refs = ndt_meta.get("scan_reference_poses") or []
    deviation, _ = deviation_about_ndt_mean_pose(
        ndt_pos,
        ndt_quat,
        per_run,
        scan_reference_poses=scan_refs if scan_refs else None,
        max_pose_pointcloud_dt_sec=max_dt_thr,
        n_runs_candidates=n_candidates,
        excluded_from_comparison=excluded if excluded else None,
    )

    yaml_out = args.yaml_out or default_yaml_out_path(args.mean_pose_json)
    doc = build_output_yaml(
        ndt_yaml=args.ndt_mean_pose_yaml,
        mean_pose_json=args.mean_pose_json,
        yaml_out=yaml_out,
        ndt_pos=ndt_pos,
        ndt_quat=ndt_quat,
        ndt_meta=ndt_meta,
        mean_meta=mean_meta,
        deviation=deviation,
    )

    yaml_text = yaml.dump(doc, default_flow_style=False, allow_unicode=True, sort_keys=False)
    yaml_out.write_text(yaml_text, encoding="utf-8")

    lon = deviation["longitudinal_m"]
    lat = deviation["lateral_m"]
    yaw = deviation["yaw"]
    pass_criteria = deviation["longitudinal_pass_criteria"]
    pass_lines = "\n".join(
        f"縦誤差合格(|longitudinal_m| <= {stats['max_abs_m']:.1f} m): "
        f"{stats['n_pass']}/{deviation['n_runs']}  "
        f"成功率={stats['success_rate_percent']:.1f}%"
        for stats in pass_criteria["by_threshold_m"].values()
    )
    excluded_n = int(deviation.get("n_runs_excluded_from_comparison", 0))
    excluded_line = ""
    if excluded_n:
        excluded_line = (
            f"候補 run 数: {deviation.get('n_runs_candidates', deviation['n_runs'])}  "
            f"除外: {excluded_n}  "
            f"(dt_pose_from_pointcloud_header_sec > "
            f"{deviation.get('max_pose_pointcloud_dt_sec_for_comparison')})\n"
        )
    print(
        "\n--- 要約(ndt_mean_pose 基準) ---\n"
        f"{excluded_line}"
        f"run 数: {deviation['n_runs']}\n"
        f"縦(前後)誤差 [m]: mean={lon['mean']:+.4f}  std={lon['std']:.4f}  "
        f"max_abs={lon['max_abs']:.4f}\n"
        f"横(左右)誤差 [m]: mean={lat['mean']:+.4f}  std={lat['std']:.4f}  "
        f"max_abs={lat['max_abs']:.4f}\n"
        f"ヨー誤差 [deg]: mean={yaw['mean_deg']:+.4f}  std={yaw['std_deg']:.4f}  "
        f"max_abs={yaw['max_abs_deg']:.4f}\n"
        f"水平 √(縦²+横²) [m]: mean={deviation['horizontal_in_ndt_body_m']['mean']:.4f}  "
        f"max={deviation['horizontal_in_ndt_body_m']['max']:.4f}\n"
        f"{pass_lines}\n",
        file=sys.stderr,
    )
    print(f"Wrote: {yaml_out}", file=sys.stderr)

    if args.json_out:
        args.json_out.write_text(
            json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        print(f"Wrote: {args.json_out}", file=sys.stderr)


if __name__ == "__main__":
    main()
