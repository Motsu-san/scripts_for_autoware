#!/usr/bin/env python3
"""
calibrate_odom_from_bag.py が出力した *_odom_calibration.yaml から
vehicle_velocity_converter / imu_corrector の param YAML を更新する。
"""

from __future__ import annotations

import argparse
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from calibrate_odom_from_bag import CORRECTION_MODES, load_ros2_parameters_block  # noqa: E402


def load_calibration_doc(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: YAML ルートが dict ではありません")
    if "odom_calibration" in doc:
        return doc["odom_calibration"]
    return doc


def resolve_correction_mode(
    doc: Dict[str, Any],
    cli_mode: Optional[str],
) -> str:
    if cli_mode is not None:
        return cli_mode
    conv = doc.get("convergence", {})
    mode = conv.get("correction_mode_applied")
    if mode in CORRECTION_MODES:
        return mode
    methods = doc.get("correction_methods", {})
    mode = methods.get("correction_mode_applied")
    if mode in CORRECTION_MODES:
        return mode
    return "both"


def backup_param_file(path: Path, *, force: bool) -> Optional[Path]:
    if not force:
        existing = sorted(path.parent.glob(f"{path.name}.bak_fine_tune_*"))
        if existing:
            return existing[0]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_name(f"{path.name}.bak_fine_tune_{stamp}")
    shutil.copy2(path, backup)
    return backup


def update_param_yaml_value(path: Path, key: str, value: float) -> None:
    with path.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: YAML ルートが dict ではありません")

    updated = False
    for block_key in ("/**", "/**:"):
        if block_key not in doc or not isinstance(doc[block_key], dict):
            continue
        block = doc[block_key]
        if "ros__parameters" in block and isinstance(block["ros__parameters"], dict):
            block["ros__parameters"][key] = float(value)
            updated = True
            break

    if not updated and isinstance(doc.get("ros__parameters"), dict):
        doc["ros__parameters"][key] = float(value)
        updated = True
    if not updated:
        raise ValueError(f"{path}: ros__parameters ブロックが見つかりません")

    with path.open("w", encoding="utf-8") as f:
        yaml.dump(doc, f, default_flow_style=False, allow_unicode=True, sort_keys=False)


def apply_calibration(
    *,
    calib_doc: Dict[str, Any],
    vehicle_velocity_param_yaml: Path,
    imu_corrector_param_yaml: Path,
    correction_mode: str,
    backup: bool,
) -> Dict[str, Any]:
    if correction_mode not in CORRECTION_MODES:
        raise ValueError(
            f"未知の correction_mode: {correction_mode!r} "
            f"(有効: {', '.join(CORRECTION_MODES)})"
        )

    corr = calib_doc.get("corrections", {})
    sf = corr.get("speed_scale_factor", {})
    bias = corr.get("angular_velocity_offset_z", {})

    apply_sf = correction_mode in ("both", "speed_only") and sf.get("apply", True)
    apply_bias = correction_mode in ("both", "yaw_only") and bias.get("apply", True)

    result: Dict[str, Any] = {
        "correction_mode": correction_mode,
        "vehicle_velocity_param_yaml": str(vehicle_velocity_param_yaml.resolve()),
        "imu_corrector_param_yaml": str(imu_corrector_param_yaml.resolve()),
        "applied": {},
        "backups": {},
    }

    if apply_sf:
        new_sf = float(sf["recommended"])
        old_sf = float(load_ros2_parameters_block(vehicle_velocity_param_yaml).get(
            "speed_scale_factor", sf.get("current", 1.0)
        ))
        if backup:
            bak = backup_param_file(vehicle_velocity_param_yaml, force=False)
            if bak is not None:
                result["backups"]["vehicle_velocity_param"] = str(bak)
        update_param_yaml_value(vehicle_velocity_param_yaml, "speed_scale_factor", new_sf)
        result["applied"]["speed_scale_factor"] = {
            "old": old_sf,
            "new": new_sf,
        }
    else:
        result["applied"]["speed_scale_factor"] = "skipped"

    if apply_bias:
        new_bias = float(bias["recommended"])
        old_bias = float(load_ros2_parameters_block(imu_corrector_param_yaml).get(
            "angular_velocity_offset_z", bias.get("current", 0.0)
        ))
        if backup:
            bak = backup_param_file(imu_corrector_param_yaml, force=False)
            if bak is not None:
                result["backups"]["imu_corrector_param"] = str(bak)
        update_param_yaml_value(
            imu_corrector_param_yaml, "angular_velocity_offset_z", new_bias
        )
        result["applied"]["angular_velocity_offset_z"] = {
            "old": old_bias,
            "new": new_bias,
        }
    else:
        result["applied"]["angular_velocity_offset_z"] = "skipped"

    return result


def main() -> int:
    ap = argparse.ArgumentParser(
        description="odom_calibration YAML から param YAML を更新"
    )
    ap.add_argument("calibration_yaml", type=Path, help="*_odom_calibration.yaml")
    ap.add_argument(
        "--vehicle-velocity-param-yaml",
        type=Path,
        required=True,
        help="更新する vehicle_velocity_converter.param.yaml",
    )
    ap.add_argument(
        "--imu-corrector-param-yaml",
        type=Path,
        required=True,
        help="更新する imu_corrector.param.yaml",
    )
    ap.add_argument(
        "--correction-mode",
        choices=CORRECTION_MODES,
        default=None,
        help="適用モード（省略時は calibration YAML の correction_mode_applied）",
    )
    ap.add_argument(
        "--no-backup",
        action="store_true",
        help="初回バックアップを作成しない",
    )
    args = ap.parse_args()

    if not args.calibration_yaml.is_file():
        print(f"Error: ファイルがありません: {args.calibration_yaml}", file=sys.stderr)
        return 2
    for p in (args.vehicle_velocity_param_yaml, args.imu_corrector_param_yaml):
        if not p.is_file():
            print(f"Error: ファイルがありません: {p}", file=sys.stderr)
            return 2

    try:
        calib_doc = load_calibration_doc(args.calibration_yaml)
        mode = resolve_correction_mode(calib_doc, args.correction_mode)
        result = apply_calibration(
            calib_doc=calib_doc,
            vehicle_velocity_param_yaml=args.vehicle_velocity_param_yaml,
            imu_corrector_param_yaml=args.imu_corrector_param_yaml,
            correction_mode=mode,
            backup=not args.no_backup,
        )
    except (ValueError, KeyError, TypeError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 3

    print(f"Applied correction_mode={mode}", file=sys.stderr)
    for key, val in result["applied"].items():
        if val == "skipped":
            print(f"  {key}: skipped", file=sys.stderr)
        else:
            print(f"  {key}: {val['old']:.6f} -> {val['new']:.6f}", file=sys.stderr)
    for key, path in result.get("backups", {}).items():
        print(f"  backup[{key}]: {path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
