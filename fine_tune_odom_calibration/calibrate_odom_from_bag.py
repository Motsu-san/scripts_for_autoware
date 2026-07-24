#!/usr/bin/env python3
"""
mean_ndt_pose.yaml を真値基準に、オドメトリのみ走行 rosbag から
initial_pose 時刻 → target_unix_sec 区間の走行距離・位置差分・ヨー差を算出し、
speed_scale_factor と yaw_rate バイアスの修正値を推定する。

補正方針（曲線走行でも過補正しにくい）:
  - speed_scale: 終点の NDT 車体座標 縦位置誤差 / 走行距離（クリップなし）
  - yaw_rate offset: 開始点方位角差 / 区間時間（start_bearing_geometry のみ）
    endpoint_yaw は診断表示のみ。auto も geometry と同じ。
  wheel 積分・heading_rate 積分は診断のみ。
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu

try:
    from autoware_vehicle_msgs.msg import VelocityReport  # type: ignore[import-not-found]
except ImportError:
    VelocityReport = None  # type: ignore[misc, assignment]


def _read_velocity_report_fields(msg: Any) -> Optional[Tuple[float, float]]:
    """VelocityReport 相当の longitudinal_velocity / heading_rate を読む。"""
    v_long = getattr(msg, "longitudinal_velocity", None)
    if v_long is None:
        return None
    heading_rate = getattr(msg, "heading_rate", None)
    hr = float(heading_rate) if heading_rate is not None else None
    return float(v_long), hr

_SCRIPT_DIR = Path(__file__).resolve().parent
_MEASURE_DIR = _SCRIPT_DIR.parent / "measure_ndt_pose_mean"
_COMPARE_DIR = _SCRIPT_DIR.parent / "compare_mean_pose"
for _p in (_SCRIPT_DIR, _MEASURE_DIR, _COMPARE_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from aggregate_pose_mean_from_bags import (  # noqa: E402
    extract_nearest_pose,
    find_rosbag2_directory,
    msg_pose_to_arrays,
    stamp_to_sec,
    yaw_from_quat_xyzw,
)
from compare_mean_pose import (  # noqa: E402
    load_ndt_mean_pose_block,
    wrap_angle_rad,
)

DEFAULT_POSE_TOPIC = "/localization/pose_twist_fusion_filter/biased_pose_with_covariance"
DEFAULT_VELOCITY_TOPIC = "/vehicle/status/velocity_status"
DEFAULT_IMU_TOPIC = "/sensing/imu/tamagawa/imu_raw"
MIN_DURATION_SEC = 1.0
CURVATURE_RATIO_WARN = 0.08
YAW_BIAS_METHODS = ("auto", "endpoint_yaw", "start_bearing_geometry")
CORRECTION_MODES = ("both", "yaw_only", "speed_only")
DEFAULT_CONVERGENCE_LAT_THRESHOLD_M = 0.10
DEFAULT_CONVERGENCE_LON_THRESHOLD_M = 0.10
DEFAULT_YAW_BIAS_METHOD = "start_bearing_geometry"
EXIT_NOT_CONVERGED = 4
VEHICLE_CONFIGS_SH = _SCRIPT_DIR.parent / "launch_replay_localization" / "vehicle_configs.sh"
VEHICLE_VELOCITY_PARAM_NAME = "vehicle_velocity_converter.param.yaml"
IMU_CORRECTOR_PARAM_NAME = "imu_corrector.param.yaml"


def load_ros2_parameters_block(path: Path) -> Dict[str, Any]:
    """Autoware /**: ros__parameters: 形式の YAML から parameters dict を返す。"""
    with path.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: YAML ルートが dict ではありません")
    for key in ("/**", "/**:"):
        if key in doc and isinstance(doc[key], dict):
            block = doc[key]
            params = block.get("ros__parameters", block)
            if isinstance(params, dict):
                return params
    if isinstance(doc.get("ros__parameters"), dict):
        return doc["ros__parameters"]
    return doc


def parse_vehicle_configs_sh(path: Path) -> List[Tuple[str, str, str, str]]:
    """vehicle_configs.sh の VEHICLE_CONFIGS 行を (fragment, model, id, sensor) で返す。"""
    if not path.is_file():
        return []
    rows: List[Tuple[str, str, str, str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line.startswith('"') or "|" not in line:
            continue
        if line.endswith('"'):
            line = line[1:-1]
        else:
            line = line.strip('"')
        parts = line.split("|")
        if len(parts) != 4:
            continue
        rows.append(tuple(p.strip() for p in parts))  # type: ignore[misc]
    return rows


def detect_vehicle_config_from_path(
    rosbag_path: str,
    *,
    configs_path: Path = VEHICLE_CONFIGS_SH,
) -> Optional[Tuple[str, str, str, str]]:
    """rosbag パスに含まれる fragment から vehicle_id / sensor_model を推定。"""
    for fragment, model, vehicle_id, sensor_model in parse_vehicle_configs_sh(configs_path):
        if fragment and fragment in rosbag_path:
            return fragment, model, vehicle_id, sensor_model
    return None


def resolve_individual_param_paths(
    *,
    individual_params_root: Path,
    vehicle_id: str,
    sensor_model: str,
) -> Tuple[Path, Path]:
    config_dir = (
        individual_params_root
        / "individual_params"
        / "config"
        / vehicle_id
        / sensor_model
    )
    vel = config_dir / VEHICLE_VELOCITY_PARAM_NAME
    imu = config_dir / IMU_CORRECTOR_PARAM_NAME
    return vel, imu


def load_current_calibration_params(
    *,
    vehicle_velocity_param_yaml: Optional[Path],
    imu_corrector_param_yaml: Optional[Path],
    fallback_speed_scale: float,
    fallback_ang_vel_offset_z: float,
) -> Tuple[float, float, Dict[str, Any]]:
    """現在の speed_scale_factor / angular_velocity_offset_z を param YAML から読む。"""
    meta: Dict[str, Any] = {
        "vehicle_velocity_param_yaml": None,
        "imu_corrector_param_yaml": None,
        "speed_scale_source": "cli_default",
        "angular_velocity_offset_z_source": "cli_default",
    }
    speed_scale = float(fallback_speed_scale)
    ang_vel_offset_z = float(fallback_ang_vel_offset_z)

    if vehicle_velocity_param_yaml is not None:
        params = load_ros2_parameters_block(vehicle_velocity_param_yaml)
        if "speed_scale_factor" not in params:
            raise ValueError(
                f"{vehicle_velocity_param_yaml}: speed_scale_factor がありません"
            )
        speed_scale = float(params["speed_scale_factor"])
        meta["vehicle_velocity_param_yaml"] = str(vehicle_velocity_param_yaml.resolve())
        meta["speed_scale_source"] = "vehicle_velocity_converter.param.yaml"

    if imu_corrector_param_yaml is not None:
        params = load_ros2_parameters_block(imu_corrector_param_yaml)
        if "angular_velocity_offset_z" not in params:
            raise ValueError(
                f"{imu_corrector_param_yaml}: angular_velocity_offset_z がありません"
            )
        ang_vel_offset_z = float(params["angular_velocity_offset_z"])
        meta["imu_corrector_param_yaml"] = str(imu_corrector_param_yaml.resolve())
        meta["angular_velocity_offset_z_source"] = "imu_corrector.param.yaml"

    return speed_scale, ang_vel_offset_z, meta


def format_rad_deg_pair(rad: float, *, digits_rad: int = 4, digits_deg: int = 4) -> str:
    return f"{math.degrees(rad):+.{digits_deg}f} deg  ({rad:+.{digits_rad}f} rad)"


def print_calibration_summary(result: Dict[str, Any], param_meta: Dict[str, Any]) -> None:
    dist = result["distance"]
    yaw = result["yaw"]
    corr = result["corrections"]
    diag = result.get("diagnostics", {})
    methods = result.get("correction_methods", {})
    tw = result["time_window"]
    sf = corr["speed_scale_factor"]
    bias = corr["angular_velocity_offset_z"]

    lines = [
        "\n--- オドメトリキャリブレーション要約 ---",
        (
            f"区間: {tw['start_unix_sec']:.3f} → {tw['end_unix_sec']:.3f} s "
            f"({tw['duration_sec']:.1f} s)"
        ),
        (
            f"距離 [m]: d_path={dist['d_path_m']:.3f}  d_net={dist['d_net_m']:.3f}  "
            f"d_excess={dist['d_excess_m']:.3f}  d_wheel={dist['d_wheel_m']:.3f}"
        ),
        (
            f"終点位置誤差 (NDT車体): 縦={yaw['longitudinal_error_at_ref_m']:+.3f} m  "
            f"横={yaw['lateral_error_at_ref_m']:+.3f} m"
        ),
        "ヨー:",
        f"  Δψ_odom (start→ref):       {format_rad_deg_pair(yaw['delta_yaw_odom_rad'])}",
        f"  ∫ω_z dt ({yaw['yaw_rate_source']}): {format_rad_deg_pair(yaw['delta_yaw_integrated_rad'])}",
        (
            f"  終点ヨー誤差 (odom vs NDT): {format_rad_deg_pair(yaw['yaw_error_at_ref_rad'])}"
        ),
        (
            f"  開始点方位角差 (odom-NDT): {format_rad_deg_pair(yaw['delta_bearing_geometry_rad'])}"
            f"  → 横誤差近似 {yaw.get('lateral_from_bearing_m', 0.0):+.3f} m"
        ),
        (
            f"  [診断] bias (endpoint_yaw):     "
            f"{yaw.get('yaw_rate_bias_endpoint_rad_s', 0.0):+.6f} rad/s"
            f"  ({math.degrees(yaw.get('yaw_rate_bias_endpoint_rad_s', 0.0)):+.4f} deg/s)"
        ),
        (
            f"  [診断] bias (start_bearing):    "
            f"{yaw.get('yaw_rate_bias_geometry_rad_s', 0.0):+.6f} rad/s"
            f"  ({math.degrees(yaw.get('yaw_rate_bias_geometry_rad_s', 0.0)):+.4f} deg/s)"
        ),
        (
            f"  採用 yaw_rate バイアス Δ:  {bias['delta']:+.6f} rad/s"
            f"  ({math.degrees(bias['delta']):+.4f} deg/s)"
            f"  [start_bearing_geometry_over_duration]"
        ),
        (
            f"  [診断] 積分_residual (odom-∫ω): "
            f"{format_rad_deg_pair(yaw.get('yaw_integrated_residual_odom_rad', 0.0))}"
        ),
        "パラメータ置換:",
    ]

    sf_path = sf.get("param_file") or param_meta.get("vehicle_velocity_param_yaml") or "(未指定)"
    lines.append("  vehicle_velocity_converter.param.yaml")
    lines.append(f"    path: {sf_path}")
    lines.append(
        f"    speed_scale_factor: {sf['current']:.6f}  →  {sf['recommended']:.6f}"
        f"  (×{sf['multiplier']:.6f})"
        f"  [{methods.get('speed_scale_factor', '?')}]"
    )

    bias_path = bias.get("param_file") or param_meta.get("imu_corrector_param_yaml") or "(未指定)"
    lines.append("  imu_corrector.param.yaml")
    lines.append(f"    path: {bias_path}")
    lines.append(
        f"    angular_velocity_offset_z: {bias['current']:.6f}  →  "
        f"{bias['recommended']:.6f} rad/s  ({math.degrees(bias['recommended']):+.4f} deg/s)"
    )
    lines.append(
        f"      (current + delta: {bias['current']:.6f} + {bias['delta']:.6f} rad/s"
        f" / {math.degrees(bias['delta']):+.4f} deg/s)"
    )

    if diag.get("wheel_based_scale_multiplier") is not None:
        lines.append(
            f"  [診断] wheel 基準 multiplier (非推奨・曲線で過補正): "
            f"{diag['wheel_based_scale_multiplier']:.6f}"
        )
    if diag.get("curvature_ratio_d_excess_over_d_path") is not None:
        lines.append(
            f"  [診断] 曲率指標 d_excess/d_path: "
            f"{diag['curvature_ratio_d_excess_over_d_path']:.4f}"
        )
    conv = result.get("convergence")
    if conv:
        lines.append("収束判定:")
        lines.append(
            f"  縦={conv['longitudinal_error_m']:+.3f} m  横={conv['lateral_error_m']:+.3f} m"
            f"  (閾値: |lon|<={conv['lon_threshold_m']:.2f}, |lat|<={conv['lat_threshold_m']:.2f})"
        )
        lines.append(
            f"  converged={conv['converged']}"
            f"  mode={conv.get('correction_mode_applied', '?')}"
        )
        if conv.get("yaw_threshold_deg") is not None:
            lines.append(
                f"  ヨー={conv['yaw_error_deg']:+.3f} deg"
                f"  (監視閾値 {conv['yaw_threshold_deg']:.2f} deg,"
                f" within={conv.get('yaw_within_threshold')})"
            )

    for w in result.get("warnings", []):
        lines.append(f"  Warning: {w}")

    if param_meta.get("speed_scale_source") == "cli_default":
        lines.append(
            "  Note: 現在の speed_scale_factor は CLI 既定値 (1.0)。"
            " --vehicle-velocity-param-yaml または --individual-params-root を指定してください。"
        )
    if param_meta.get("angular_velocity_offset_z_source") == "cli_default":
        lines.append(
            "  Note: 現在の angular_velocity_offset_z は CLI 既定値 (0.0)。"
            " --imu-corrector-param-yaml または --individual-params-root を指定してください。"
        )

    print("\n".join(lines) + "\n", file=sys.stderr)


def load_initial_pose_yaml(
    path: Path,
) -> Tuple[np.ndarray, np.ndarray, Optional[float], Dict[str, Any]]:
    """initial_pose.yaml / ndt_start_pose.yaml から pose と任意の開始時刻を読む。"""
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        raise ValueError(f"{path}: 空の YAML です")

    pose_block = data.get("pose")
    if pose_block is None:
        raise ValueError(f"{path}: pose ブロックがありません")
    pose = pose_block.get("pose", pose_block)
    pos = pose["position"]
    ori = pose["orientation"]
    pos_v = np.array([float(pos["x"]), float(pos["y"]), float(pos["z"])], dtype=float)
    quat = np.array(
        [float(ori["x"]), float(ori["y"]), float(ori["z"]), float(ori["w"])], dtype=float
    )
    n = np.linalg.norm(quat)
    if n < 1e-12:
        raise ValueError(f"{path}: orientation のノルムがゼロに近いです")
    quat /= n

    start_sec: Optional[float] = None
    for key in ("mean_pose_header_stamp", "header_stamp"):
        block = data.get(key)
        if isinstance(block, dict) and "sec" in block:
            start_sec = float(block["sec"]) + float(block.get("nanosec", 0)) * 1e-9
            break
    hdr = pose_block.get("header") if isinstance(pose_block, dict) else None
    if start_sec is None and isinstance(hdr, dict) and "stamp" in hdr:
        stamp = hdr["stamp"]
        if isinstance(stamp, dict) and "sec" in stamp:
            start_sec = float(stamp["sec"]) + float(stamp.get("nanosec", 0)) * 1e-9
    if start_sec is None:
        root_hdr = data.get("header")
        if isinstance(root_hdr, dict) and "stamp" in root_hdr:
            stamp = root_hdr["stamp"]
            if isinstance(stamp, dict) and "sec" in stamp:
                start_sec = float(stamp["sec"]) + float(stamp.get("nanosec", 0)) * 1e-9

    meta = {"path": str(path), "start_unix_sec_from_yaml": start_sec}
    return pos_v, quat, start_sec, meta


def target_unix_sec_from_ndt_meta(ndt_meta: Dict[str, Any]) -> Optional[float]:
    agg = ndt_meta.get("aggregated")
    if isinstance(agg, dict) and agg.get("target_unix_sec") is not None:
        return float(agg["target_unix_sec"])
    return None


def _message_stamp_sec(msg: Any, bag_receive_ns: int) -> float:
    hdr = getattr(msg, "header", None)
    if hdr is not None and hasattr(hdr, "stamp"):
        t = stamp_to_sec(hdr.stamp)
        if t > 1.0:
            return t
    return float(bag_receive_ns) * 1e-9


def scan_pose_series(
    bag_dir: str,
    topic: str,
    t_start: float,
    t_end: float,
) -> List[Tuple[float, np.ndarray, np.ndarray]]:
    """[t_start, t_end] 内の pose 系列を (stamp_sec, pos, quat) のリストで返す。"""
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
        raise RuntimeError(f"bag に pose トピックがありません: {topic}")
    msg_type = get_message(type_map[topic])

    series: List[Tuple[float, np.ndarray, np.ndarray]] = []
    while reader.has_next():
        tname, data, ts_ns = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, (PoseStamped, PoseWithCovarianceStamped)):
            continue
        stamp_sec = _message_stamp_sec(msg, ts_ns)
        if stamp_sec < t_start or stamp_sec > t_end:
            continue
        pos, quat = msg_pose_to_arrays(msg)
        series.append((stamp_sec, pos, quat))
    series.sort(key=lambda x: x[0])
    return series


def scan_velocity_series(
    bag_dir: str,
    topic: str,
    t_start: float,
    t_end: float,
) -> Tuple[List[Tuple[float, float, Optional[float]]], Dict[str, Any]]:
    """(stamp_sec, longitudinal_velocity, heading_rate) の系列。heading_rate は無ければ None。"""
    meta: Dict[str, Any] = {
        "topic": topic,
        "topic_present": False,
        "messages_total": 0,
        "messages_in_window": 0,
        "skipped_no_fields": 0,
    }

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
        return [], meta
    meta["topic_present"] = True
    meta["message_type"] = type_map[topic]
    try:
        msg_type = get_message(type_map[topic])
    except (AttributeError, ModuleNotFoundError, ValueError) as exc:
        meta["deserialize_error"] = str(exc)
        return [], meta

    series: List[Tuple[float, float, Optional[float]]] = []
    while reader.has_next():
        tname, data, ts_ns = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        meta["messages_total"] += 1
        fields = _read_velocity_report_fields(msg)
        if fields is None:
            meta["skipped_no_fields"] += 1
            continue
        v_long, hr = fields
        stamp_sec = _message_stamp_sec(msg, ts_ns)
        if stamp_sec < t_start or stamp_sec > t_end:
            continue
        meta["messages_in_window"] += 1
        series.append((stamp_sec, v_long, hr))
    series.sort(key=lambda x: x[0])
    return series, meta


def scan_kinematic_yaw_rate_series(
    bag_dir: str,
    topic: str,
    t_start: float,
    t_end: float,
) -> List[Tuple[float, float]]:
    """kinematic_state (Odometry) の twist.angular.z 系列。"""
    from nav_msgs.msg import Odometry

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
        return []
    msg_type = get_message(type_map[topic])

    series: List[Tuple[float, float]] = []
    while reader.has_next():
        tname, data, ts_ns = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, Odometry):
            continue
        stamp_sec = _message_stamp_sec(msg, ts_ns)
        if stamp_sec < t_start or stamp_sec > t_end:
            continue
        series.append((stamp_sec, float(msg.twist.twist.angular.z)))
    series.sort(key=lambda x: x[0])
    return series


def scan_kinematic_longitudinal_velocity_series(
    bag_dir: str,
    topic: str,
    t_start: float,
    t_end: float,
) -> List[Tuple[float, float]]:
    """kinematic_state (Odometry) の twist.linear.x 系列（velocity 欠落時のフォールバック）。"""
    from nav_msgs.msg import Odometry

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
        return []
    msg_type = get_message(type_map[topic])

    series: List[Tuple[float, float]] = []
    while reader.has_next():
        tname, data, ts_ns = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, Odometry):
            continue
        stamp_sec = _message_stamp_sec(msg, ts_ns)
        if stamp_sec < t_start or stamp_sec > t_end:
            continue
        series.append((stamp_sec, float(msg.twist.twist.linear.x)))
    series.sort(key=lambda x: x[0])
    return series


def scan_imu_yaw_rate_series(
    bag_dir: str,
    topic: str,
    t_start: float,
    t_end: float,
) -> List[Tuple[float, float]]:
    """(stamp_sec, angular_velocity_z) の系列。"""
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
        return []
    msg_type = get_message(type_map[topic])

    series: List[Tuple[float, float]] = []
    while reader.has_next():
        tname, data, ts_ns = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, Imu):
            continue
        stamp_sec = _message_stamp_sec(msg, ts_ns)
        if stamp_sec < t_start or stamp_sec > t_end:
            continue
        series.append((stamp_sec, float(msg.angular_velocity.z)))
    series.sort(key=lambda x: x[0])
    return series


def trapezoid_integrate(samples: List[Tuple[float, float]]) -> float:
    """(t, value) 系列を台形積分する。"""
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for i in range(len(samples) - 1):
        t0, v0 = samples[i]
        t1, v1 = samples[i + 1]
        dt = t1 - t0
        if dt <= 0.0:
            continue
        total += 0.5 * (v0 + v1) * dt
    return total


def compute_path_length(series: List[Tuple[float, np.ndarray, np.ndarray]]) -> float:
    if len(series) < 2:
        return 0.0
    total = 0.0
    for i in range(len(series) - 1):
        p0 = series[i][1][:2]
        p1 = series[i + 1][1][:2]
        total += float(np.linalg.norm(p1 - p0))
    return total


def nearest_pose_in_series(
    series: List[Tuple[float, np.ndarray, np.ndarray]],
    target_sec: float,
) -> Optional[Tuple[float, np.ndarray, np.ndarray]]:
    if not series:
        return None
    best = min(series, key=lambda x: abs(x[0] - target_sec))
    return best


def bearing_planar_rad(p_from: np.ndarray, p_to: np.ndarray) -> float:
    """平面 XY で p_from → p_to の方位角 [rad]。"""
    d = p_to - p_from
    return float(math.atan2(d[1], d[0]))


def body_frame_deviation(
    ref_pos: np.ndarray,
    ref_quat: np.ndarray,
    meas_pos: np.ndarray,
    meas_quat: np.ndarray,
) -> Dict[str, float]:
    """ref 姿勢の車体座標系での縦・横・ヨー差（compare_mean_pose と同様）。"""
    r_ref = R.from_quat(ref_quat)
    yaw_ref = yaw_from_quat_xyzw(ref_quat)
    d_map = meas_pos - ref_pos
    d_body = r_ref.inv().apply(d_map)
    yaw_meas = yaw_from_quat_xyzw(meas_quat)
    yaw_err = wrap_angle_rad(yaw_meas - yaw_ref)
    return {
        "longitudinal_m": float(d_body[0]),
        "lateral_m": float(d_body[1]),
        "yaw_error_rad": float(yaw_err),
        "yaw_error_deg": float(math.degrees(yaw_err)),
    }


def format_velocity_scan_hint(
    *,
    wheel_bag: Path,
    velocity_topic: str,
    kinematic_topic: str,
    t_start: float,
    t_ref: float,
    vel_meta: Dict[str, Any],
    kin_vel_count: int,
) -> str:
    lines = [
        f"  wheel bag: {wheel_bag}",
        f"  積分区間: [{t_start:.3f}, {t_ref:.3f}]",
        f"  {velocity_topic}: present={vel_meta.get('topic_present')} "
        f"total={vel_meta.get('messages_total')} in_window={vel_meta.get('messages_in_window')}",
    ]
    if vel_meta.get("message_type"):
        lines.append(f"    type: {vel_meta['message_type']}")
    if vel_meta.get("deserialize_error"):
        lines.append(f"    deserialize_error: {vel_meta['deserialize_error']}")
        lines.append(
            "    → Autoware WS を source してください "
            "(例: source ~/autoware/install/setup.bash)"
        )
    if vel_meta.get("skipped_no_fields"):
        lines.append(f"    skipped_no_fields: {vel_meta['skipped_no_fields']}")
    lines.append(f"  {kinematic_topic} (fallback): in_window={kin_vel_count}")
    return "\n".join(lines)


def resolve_start_unix_sec(
    *,
    cli_start: Optional[float],
    yaml_start: Optional[float],
    pose_series: List[Tuple[float, np.ndarray, np.ndarray]],
) -> Tuple[float, str]:
    if cli_start is not None:
        return float(cli_start), "cli"
    if yaml_start is not None:
        return float(yaml_start), "initial_pose_yaml"
    if pose_series:
        print(
            "Warning: 開始時刻が未指定のため bag 内 pose の最初の stamp を使用します",
            file=sys.stderr,
        )
        return float(pose_series[0][0]), "first_pose_in_bag"
    raise ValueError("開始時刻を決定できません（--start-unix-sec または initial_pose の stamp を指定してください）")


def default_yaml_out_path(mean_ndt_pose_yaml: Path, odom_bag: Path) -> Path:
    bag_stem = Path(odom_bag).stem
    return mean_ndt_pose_yaml.with_name(f"{bag_stem}_odom_calibration.yaml")


def apply_yaw_bias_method_to_result(
    result: Dict[str, Any],
    yaw_bias_method: str,
) -> Dict[str, Any]:
    """推定済みの endpoint / geometry バイアスから採用値を切り替える。"""
    yaw = result["yaw"]
    corr = result["corrections"]
    bias = dict(corr["angular_velocity_offset_z"])
    current = float(bias["current"])

    if yaw_bias_method in ("endpoint_yaw", "endpoint_yaw_error_over_duration"):
        delta = float(yaw["yaw_rate_bias_endpoint_rad_s"])
        method_used = "endpoint_yaw_error_over_duration"
        method_key = "endpoint_yaw"
    elif yaw_bias_method in (
        "start_bearing_geometry",
        "start_bearing_geometry_over_duration",
    ):
        delta = float(yaw["yaw_rate_bias_geometry_rad_s"])
        method_used = "start_bearing_geometry_over_duration"
        method_key = "start_bearing_geometry"
    else:
        raise ValueError(
            f"未知の yaw_bias_method: {yaw_bias_method!r} "
            f"(有効: {', '.join(YAW_BIAS_METHODS)} または *_over_duration)"
        )

    recommended = current + delta
    bias.update(
        {
            "delta": float(delta),
            "delta_rad_s": float(delta),
            "delta_deg_s": float(math.degrees(delta)),
            "recommended": float(recommended),
            "recommended_rad_s": float(recommended),
            "recommended_deg_s": float(math.degrees(recommended)),
            "method": method_used,
        }
    )

    yaw_out = dict(yaw)
    yaw_out["yaw_rate_bias_z_rad_s"] = float(delta)
    yaw_out["yaw_rate_bias_z_deg_s"] = float(math.degrees(delta))

    result = dict(result)
    result["yaw"] = yaw_out
    result["corrections"] = {
        "speed_scale_factor": dict(corr["speed_scale_factor"]),
        "angular_velocity_offset_z": bias,
    }
    methods = dict(result.get("correction_methods", {}))
    methods["angular_velocity_offset_z"] = method_used
    methods["yaw_bias_method_applied"] = method_key
    result["correction_methods"] = methods
    return result


def apply_correction_mode(
    result: Dict[str, Any],
    correction_mode: str,
) -> Dict[str, Any]:
    """correction_mode に応じて corrections の適用値を調整する。"""
    if correction_mode not in CORRECTION_MODES:
        raise ValueError(
            f"未知の correction_mode: {correction_mode!r} "
            f"(有効: {', '.join(CORRECTION_MODES)})"
        )

    corr = result["corrections"]
    sf = dict(corr["speed_scale_factor"])
    bias = dict(corr["angular_velocity_offset_z"])

    if correction_mode == "yaw_only":
        sf["multiplier"] = 1.0
        sf["recommended"] = float(sf["current"])
        sf["apply"] = False
        bias["apply"] = True
    elif correction_mode == "speed_only":
        bias["delta"] = 0.0
        bias["delta_rad_s"] = 0.0
        bias["delta_deg_s"] = 0.0
        bias["recommended"] = float(bias["current"])
        bias["recommended_rad_s"] = float(bias["current"])
        bias["recommended_deg_s"] = float(math.degrees(bias["current"]))
        bias["apply"] = False
        sf["apply"] = True
    else:
        sf["apply"] = True
        bias["apply"] = True

    result = dict(result)
    result["corrections"] = {
        "speed_scale_factor": sf,
        "angular_velocity_offset_z": bias,
    }
    methods = dict(result.get("correction_methods", {}))
    methods["correction_mode_applied"] = correction_mode
    result["correction_methods"] = methods
    return result


def build_convergence_block(
    result: Dict[str, Any],
    *,
    lat_threshold_m: float,
    lon_threshold_m: float,
    correction_mode: str,
    yaw_threshold_deg: Optional[float] = None,
) -> Dict[str, Any]:
    yaw = result["yaw"]
    lat_err = float(yaw["lateral_error_at_ref_m"])
    lon_err = float(yaw["longitudinal_error_at_ref_m"])
    yaw_err_deg = float(yaw["yaw_error_at_ref_deg"])
    converged = (
        abs(lat_err) <= lat_threshold_m and abs(lon_err) <= lon_threshold_m
    )
    block: Dict[str, Any] = {
        "lat_threshold_m": float(lat_threshold_m),
        "lon_threshold_m": float(lon_threshold_m),
        "lateral_error_m": lat_err,
        "longitudinal_error_m": lon_err,
        "yaw_error_deg": yaw_err_deg,
        "converged": bool(converged),
        "correction_mode_applied": correction_mode,
    }
    if yaw_threshold_deg is not None:
        block["yaw_threshold_deg"] = float(yaw_threshold_deg)
        block["yaw_within_threshold"] = abs(yaw_err_deg) <= yaw_threshold_deg
    return block


def finalize_calibration_result(
    result: Dict[str, Any],
    *,
    correction_mode: str,
    lat_threshold_m: float,
    lon_threshold_m: float,
    yaw_threshold_deg: Optional[float] = None,
    yaw_bias_method: str = DEFAULT_YAW_BIAS_METHOD,
) -> Dict[str, Any]:
    result = apply_yaw_bias_method_to_result(result, yaw_bias_method)
    result = apply_correction_mode(result, correction_mode)
    result["convergence"] = build_convergence_block(
        result,
        lat_threshold_m=lat_threshold_m,
        lon_threshold_m=lon_threshold_m,
        correction_mode=correction_mode,
        yaw_threshold_deg=yaw_threshold_deg,
    )
    return result


def calibrate(
    *,
    ndt_pos: np.ndarray,
    ndt_quat: np.ndarray,
    init_pos: np.ndarray,
    init_quat: np.ndarray,
    t_start: float,
    t_ref: float,
    pose_series: List[Tuple[float, np.ndarray, np.ndarray]],
    vel_series: List[Tuple[float, float, Optional[float]]],
    imu_series: List[Tuple[float, float]],
    wheel_distance_source: str,
    pose_at_start: Optional[Dict[str, Any]],
    pose_at_ref: Optional[Dict[str, Any]],
    current_speed_scale: float,
    current_ang_vel_offset_z: float,
    yaw_bias_method: str = DEFAULT_YAW_BIAS_METHOD,
    vehicle_velocity_param_file: Optional[str] = None,
    imu_corrector_param_file: Optional[str] = None,
) -> Dict[str, Any]:
    duration = t_ref - t_start
    if duration < MIN_DURATION_SEC:
        raise ValueError(
            f"積分区間が短すぎます: {duration:.3f}s < {MIN_DURATION_SEC}s"
        )

    d_path = compute_path_length(pose_series)

    if pose_at_start is not None and pose_at_ref is not None:
        p_start = np.array(
            [
                pose_at_start["position"]["x"],
                pose_at_start["position"]["y"],
            ],
            dtype=float,
        )
        p_ref = np.array(
            [
                pose_at_ref["position"]["x"],
                pose_at_ref["position"]["y"],
            ],
            dtype=float,
        )
        odom_quat_start = np.array(
            [
                pose_at_start["orientation"]["x"],
                pose_at_start["orientation"]["y"],
                pose_at_start["orientation"]["z"],
                pose_at_start["orientation"]["w"],
            ],
            dtype=float,
        )
        odom_quat_ref = np.array(
            [
                pose_at_ref["orientation"]["x"],
                pose_at_ref["orientation"]["y"],
                pose_at_ref["orientation"]["z"],
                pose_at_ref["orientation"]["w"],
            ],
            dtype=float,
        )
    else:
        near_start = nearest_pose_in_series(pose_series, t_start)
        near_ref = nearest_pose_in_series(pose_series, t_ref)
        if near_start is None or near_ref is None:
            raise ValueError("区間内に pose メッセージがありません")
        p_start = near_start[1][:2]
        p_ref = near_ref[1][:2]
        odom_quat_start = near_start[2]
        odom_quat_ref = near_ref[2]

    d_net = float(np.linalg.norm(p_ref - p_start))
    d_excess = d_path - d_net

    p_ndt = ndt_pos[:2]
    d_truth_net = float(np.linalg.norm(p_ndt - p_start))

    r_start = R.from_quat(odom_quat_start)
    d_truth_map = ndt_pos - np.array([p_start[0], p_start[1], float(ndt_pos[2])])
    d_odom_map = np.array([p_ref[0], p_ref[1], float(ndt_pos[2])]) - np.array(
        [p_start[0], p_start[1], float(ndt_pos[2])]
    )
    d_truth_body = r_start.inv().apply(d_truth_map)
    d_odom_body = r_start.inv().apply(d_odom_map)
    d_truth_long = float(d_truth_body[0])
    d_odom_long = float(d_odom_body[0])

    wheel_samples = [(t, v) for t, v, _ in vel_series]
    d_wheel = trapezoid_integrate(wheel_samples)

    # EKF / gyro_odometer は IMU 角速度を主に使うため IMU を優先
    yaw_rate_samples: List[Tuple[float, float]] = []
    yaw_rate_source = "none"
    if imu_series:
        yaw_rate_samples = list(imu_series)
        yaw_rate_source = "imu_angular_velocity_z"
    elif vel_series:
        yaw_rate_samples = [
            (t, float(hr)) for t, _, hr in vel_series if hr is not None
        ]
        if yaw_rate_samples:
            yaw_rate_source = "velocity_heading_rate"

    delta_yaw_integrated = trapezoid_integrate(yaw_rate_samples)
    yaw_odom_start = yaw_from_quat_xyzw(odom_quat_start)
    yaw_odom_ref = yaw_from_quat_xyzw(odom_quat_ref)
    yaw_ndt = yaw_from_quat_xyzw(ndt_quat)
    delta_yaw_odom = wrap_angle_rad(yaw_odom_ref - yaw_odom_start)
    delta_yaw_truth = wrap_angle_rad(yaw_ndt - yaw_odom_start)
    yaw_integrated_residual_odom = wrap_angle_rad(delta_yaw_odom - delta_yaw_integrated)
    yaw_integrated_residual_truth = wrap_angle_rad(delta_yaw_truth - delta_yaw_integrated)

    if pose_at_ref:
        odom_pos_ref = np.array(
            [
                pose_at_ref["position"]["x"],
                pose_at_ref["position"]["y"],
                pose_at_ref["position"]["z"],
            ],
            dtype=float,
        )
    else:
        near_ref = nearest_pose_in_series(pose_series, t_ref)
        if near_ref is None:
            raise ValueError("参照時刻近傍の pose を取得できません")
        odom_pos_ref = near_ref[1]
    deviation_ref = body_frame_deviation(ndt_pos, ndt_quat, odom_pos_ref, odom_quat_ref)

    lat_err = float(deviation_ref["lateral_m"])
    lon_err = float(deviation_ref["longitudinal_m"])
    yaw_err = float(deviation_ref["yaw_error_rad"])
    travel_denom = max(d_path, d_net, 1.0)
    curvature_ratio = float(d_excess / travel_denom) if travel_denom > 1e-9 else 0.0

    # 終点の NDT 車体座標誤差から補正（クリップなし）
    # 縦 + → odom が進みすぎ → scale を下げる
    scale_multiplier = 1.0 - lon_err / travel_denom

    # imu_corrector: corrected = raw + offset
    bearing_odom = bearing_planar_rad(p_start, p_ref)
    bearing_ndt = bearing_planar_rad(p_start, p_ndt)
    delta_bearing_geom = wrap_angle_rad(bearing_odom - bearing_ndt)
    yaw_rate_bias_endpoint = -yaw_err / duration
    yaw_rate_bias_geometry = -delta_bearing_geom / duration
    lateral_from_bearing = float(d_net * math.sin(delta_bearing_geom))

    if yaw_bias_method == "endpoint_yaw":
        yaw_bias_method_used = "endpoint_yaw_error_over_duration"
        yaw_rate_bias_z = yaw_rate_bias_endpoint
    elif yaw_bias_method in ("auto", "start_bearing_geometry"):
        yaw_bias_method_used = "start_bearing_geometry_over_duration"
        yaw_rate_bias_z = yaw_rate_bias_geometry
    else:
        raise ValueError(
            f"未知の yaw_bias_method: {yaw_bias_method!r} "
            f"(有効: {', '.join(YAW_BIAS_METHODS)})"
        )

    warnings: List[str] = []
    if curvature_ratio > CURVATURE_RATIO_WARN:
        warnings.append(
            f"曲率が大きい区間です (d_excess/d_path={curvature_ratio:.3f})。"
            " wheel 基準 multiplier は診断のみ参照してください。"
        )
    integrated_vs_odom = abs(wrap_angle_rad(delta_yaw_integrated - delta_yaw_odom))
    if math.degrees(integrated_vs_odom) > 30.0:
        warnings.append(
            f"∫ω_z ({yaw_rate_source}) と odom yaw 変化が大きく乖離しています "
            f"({math.degrees(integrated_vs_odom):.1f} deg)。"
            " バイアス推定は終点誤差ベースを使用しています。"
        )
    if (
        yaw_bias_method_used == "start_bearing_geometry_over_duration"
        and curvature_ratio > CURVATURE_RATIO_WARN
    ):
        warnings.append(
            "開始点方位角差ベースのバイアス推定は曲線区間では弦の方位と姿勢ヨーが一致しません。"
            " endpoint_yaw との差を確認してください。"
        )

    if abs(d_wheel) < 1e-6:
        raise ValueError("車輪速積分距離 d_wheel がゼロに近いです")
    wheel_based_scale_multiplier = d_truth_net / d_wheel

    speed_scale_recommended = current_speed_scale * scale_multiplier
    ang_vel_offset_z_recommended = current_ang_vel_offset_z + yaw_rate_bias_z

    net_ratio_truth_odom = d_truth_net / d_net if abs(d_net) > 1e-9 else None

    return {
        "time_window": {
            "start_unix_sec": float(t_start),
            "end_unix_sec": float(t_ref),
            "duration_sec": float(duration),
        },
        "distance": {
            "d_path_m": float(d_path),
            "d_net_m": float(d_net),
            "d_excess_m": float(d_excess),
            "d_wheel_m": float(d_wheel),
            "d_truth_net_m": float(d_truth_net),
            "d_truth_longitudinal_m": float(d_truth_long),
            "d_odom_longitudinal_m": float(d_odom_long),
            "net_ratio_truth_over_odom": net_ratio_truth_odom,
            "n_pose_samples": len(pose_series),
            "n_velocity_samples": len(vel_series),
            "wheel_distance_source": wheel_distance_source,
            "curvature_ratio_d_excess_over_d_path": float(curvature_ratio),
        },
        "yaw": {
            "delta_yaw_odom_rad": float(delta_yaw_odom),
            "delta_yaw_odom_deg": float(math.degrees(delta_yaw_odom)),
            "delta_yaw_truth_rad": float(delta_yaw_truth),
            "delta_yaw_truth_deg": float(math.degrees(delta_yaw_truth)),
            "delta_yaw_integrated_rad": float(delta_yaw_integrated),
            "delta_yaw_integrated_deg": float(math.degrees(delta_yaw_integrated)),
            "yaw_rate_source": yaw_rate_source,
            "n_yaw_rate_samples": len(yaw_rate_samples),
            "bearing_odom_rad": float(bearing_odom),
            "bearing_odom_deg": float(math.degrees(bearing_odom)),
            "bearing_ndt_rad": float(bearing_ndt),
            "bearing_ndt_deg": float(math.degrees(bearing_ndt)),
            "delta_bearing_geometry_rad": float(delta_bearing_geom),
            "delta_bearing_geometry_deg": float(math.degrees(delta_bearing_geom)),
            "lateral_from_bearing_m": float(lateral_from_bearing),
            "yaw_rate_bias_endpoint_rad_s": float(yaw_rate_bias_endpoint),
            "yaw_rate_bias_endpoint_deg_s": float(math.degrees(yaw_rate_bias_endpoint)),
            "yaw_rate_bias_geometry_rad_s": float(yaw_rate_bias_geometry),
            "yaw_rate_bias_geometry_deg_s": float(math.degrees(yaw_rate_bias_geometry)),
            "yaw_rate_bias_z_rad_s": float(yaw_rate_bias_z),
            "yaw_rate_bias_z_deg_s": float(math.degrees(yaw_rate_bias_z)),
            "yaw_integrated_residual_odom_rad": float(yaw_integrated_residual_odom),
            "yaw_integrated_residual_odom_deg": float(
                math.degrees(yaw_integrated_residual_odom)
            ),
            "yaw_integrated_residual_truth_rad": float(yaw_integrated_residual_truth),
            "yaw_integrated_residual_truth_deg": float(
                math.degrees(yaw_integrated_residual_truth)
            ),
            "yaw_error_at_ref_rad": deviation_ref["yaw_error_rad"],
            "yaw_error_at_ref_deg": deviation_ref["yaw_error_deg"],
            "longitudinal_error_at_ref_m": deviation_ref["longitudinal_m"],
            "lateral_error_at_ref_m": deviation_ref["lateral_m"],
        },
        "correction_methods": {
            "speed_scale_factor": "endpoint_longitudinal_error_over_path",
            "angular_velocity_offset_z": yaw_bias_method_used,
            "yaw_bias_method_requested": yaw_bias_method,
        },
        "diagnostics": {
            "wheel_based_scale_multiplier": float(wheel_based_scale_multiplier),
            "endpoint_lat_err_m": float(lat_err),
            "endpoint_lon_err_m": float(lon_err),
            "endpoint_yaw_err_rad": float(yaw_err),
            "delta_bearing_geometry_rad": float(delta_bearing_geom),
            "lateral_from_bearing_m": float(lateral_from_bearing),
            "travel_denom_m": float(travel_denom),
            "curvature_ratio_d_excess_over_d_path": float(curvature_ratio),
        },
        "warnings": warnings,
        "corrections": {
            "speed_scale_factor": {
                "param_file": vehicle_velocity_param_file,
                "param_key": "speed_scale_factor",
                "method": "endpoint_longitudinal_error_over_path",
                "current": float(current_speed_scale),
                "multiplier": float(scale_multiplier),
                "recommended": float(speed_scale_recommended),
            },
            "angular_velocity_offset_z": {
                "param_file": imu_corrector_param_file,
                "param_key": "angular_velocity_offset_z",
                "method": yaw_bias_method_used,
                "current": float(current_ang_vel_offset_z),
                "delta_rad_s": float(yaw_rate_bias_z),
                "delta_deg_s": float(math.degrees(yaw_rate_bias_z)),
                "recommended_rad_s": float(ang_vel_offset_z_recommended),
                "recommended_deg_s": float(math.degrees(ang_vel_offset_z_recommended)),
                "delta": float(yaw_rate_bias_z),
                "recommended": float(ang_vel_offset_z_recommended),
            },
        },
    }


def build_output_doc(
    *,
    mean_ndt_pose_yaml: Path,
    odom_bag: Path,
    initial_pose_yaml: Path,
    ndt_pos: np.ndarray,
    ndt_quat: np.ndarray,
    init_pos: np.ndarray,
    init_quat: np.ndarray,
    pose_topic: str,
    velocity_topic: str,
    imu_topic: str,
    wheel_rosbag: Optional[Path],
    start_source: str,
    param_meta: Dict[str, Any],
    result: Dict[str, Any],
    yaml_out: Path,
) -> Dict[str, Any]:
    return {
        "odom_calibration": {
            "description": (
                "mean_ndt_pose 基準・initial_pose→target 区間のオドメトリ補正推定"
            ),
            "inputs": {
                "mean_ndt_pose_yaml": str(mean_ndt_pose_yaml.resolve()),
                "odom_rosbag": str(Path(odom_bag).resolve()),
                "initial_pose_yaml": str(initial_pose_yaml.resolve()),
                "pose_topic": pose_topic,
                "velocity_topic": velocity_topic,
                "imu_topic": imu_topic,
                "wheel_rosbag": str(wheel_rosbag.resolve()) if wheel_rosbag else None,
                "start_time_source": start_source,
                "current_param_sources": param_meta,
            },
            "output_yaml": str(yaml_out.resolve()),
            **result,
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
        },
        "initial_pose": {
            "position": {
                "x": float(init_pos[0]),
                "y": float(init_pos[1]),
                "z": float(init_pos[2]),
            },
            "orientation": {
                "x": float(init_quat[0]),
                "y": float(init_quat[1]),
                "z": float(init_quat[2]),
                "w": float(init_quat[3]),
            },
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "mean_ndt_pose.yaml を基準にオドメトリのみ rosbag から"
            " speed_scale_factor / yaw_rate バイアス補正を推定"
        )
    )
    ap.add_argument(
        "mean_ndt_pose_yaml",
        type=Path,
        help="NDT 真値 pose を含む mean_ndt_pose.yaml",
    )
    ap.add_argument("odom_rosbag", type=Path, help="オドメトリのみ走行の記録 rosbag")
    ap.add_argument(
        "--initial-pose-yaml",
        type=Path,
        required=True,
        help="積分始点の initial_pose.yaml（または ndt_start_pose.yaml）",
    )
    ap.add_argument(
        "--start-unix-sec",
        type=float,
        default=None,
        help="積分開始 UNIX 時刻（省略時は initial_pose YAML または bag 先頭 pose）",
    )
    ap.add_argument(
        "--target-unix-sec",
        type=float,
        default=None,
        help="積分終了 UNIX 時刻（省略時は mean_ndt_pose.yaml の aggregated.target_unix_sec）",
    )
    ap.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC)
    ap.add_argument("--velocity-topic", default=DEFAULT_VELOCITY_TOPIC)
    ap.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    ap.add_argument(
        "--wheel-rosbag",
        type=Path,
        default=None,
        help="車速・角速度を読む bag（省略時は odom_rosbag。記録 bag に velocity が無い場合に元 bag を指定）",
    )
    ap.add_argument(
        "--kinematic-topic",
        default="/localization/pose_twist_fusion_filter/kinematic_state",
        help="velocity_status 欠落時のフォールバック（twist.linear.x / angular.z）",
    )
    ap.add_argument(
        "--vehicle-velocity-param-yaml",
        type=Path,
        default=None,
        help="現在の speed_scale_factor を読む vehicle_velocity_converter.param.yaml",
    )
    ap.add_argument(
        "--imu-corrector-param-yaml",
        type=Path,
        default=None,
        help="現在の angular_velocity_offset_z を読む imu_corrector.param.yaml",
    )
    ap.add_argument(
        "--individual-params-root",
        type=Path,
        default=None,
        help=(
            "individual_params リポジトリ/パッケージのルート。"
            " rosbag パスから vehicle を推定し param YAML を自動解決"
            "（--vehicle-velocity-param-yaml / --imu-corrector-param-yaml 未指定時）"
        ),
    )
    ap.add_argument(
        "--current-speed-scale-factor",
        type=float,
        default=1.0,
        help="param YAML 未指定時の speed_scale_factor 既定値",
    )
    ap.add_argument(
        "--current-angular-velocity-offset-z",
        type=float,
        default=0.0,
        help="param YAML 未指定時の angular_velocity_offset_z 既定値 [rad/s]",
    )
    ap.add_argument(
        "--yaw-bias-method",
        choices=YAW_BIAS_METHODS,
        default=DEFAULT_YAW_BIAS_METHOD,
        help=(
            "yaw_rate バイアス推定: 既定 start_bearing_geometry（開始点方位角差）。"
            " endpoint_yaw は診断用。auto は geometry と同じ"
        ),
    )
    ap.add_argument(
        "--correction-mode",
        choices=CORRECTION_MODES,
        default="both",
        help="適用する補正: both / yaw_only / speed_only",
    )
    ap.add_argument(
        "--convergence-lat-threshold",
        type=float,
        default=DEFAULT_CONVERGENCE_LAT_THRESHOLD_M,
        help="収束判定の横位置誤差閾値 [m]",
    )
    ap.add_argument(
        "--convergence-lon-threshold",
        type=float,
        default=DEFAULT_CONVERGENCE_LON_THRESHOLD_M,
        help="収束判定の縦位置誤差閾値 [m]",
    )
    ap.add_argument(
        "--convergence-yaw-threshold-deg",
        type=float,
        default=None,
        help="ヨー誤差の監視閾値 [deg]（収束判定には含めない）",
    )
    ap.add_argument(
        "--check-convergence",
        action="store_true",
        help=f"収束時 exit 0、未収束 exit {EXIT_NOT_CONVERGED}",
    )
    ap.add_argument("--yaml-out", type=Path, default=None)
    ap.add_argument("--json-out", type=Path, default=None)
    args = ap.parse_args()

    if not args.mean_ndt_pose_yaml.is_file():
        print(f"Error: ファイルがありません: {args.mean_ndt_pose_yaml}", file=sys.stderr)
        return 2
    if not args.initial_pose_yaml.is_file():
        print(f"Error: ファイルがありません: {args.initial_pose_yaml}", file=sys.stderr)
        return 2

    ndt_pos, ndt_quat, ndt_meta = load_ndt_mean_pose_block(args.mean_ndt_pose_yaml)
    t_ref = args.target_unix_sec
    if t_ref is None:
        t_ref = target_unix_sec_from_ndt_meta(ndt_meta)
    if t_ref is None:
        print(
            "Error: target_unix_sec を決定できません（--target-unix-sec または YAML の aggregated.target_unix_sec）",
            file=sys.stderr,
        )
        return 2

    init_pos, init_quat, yaml_start, _ = load_initial_pose_yaml(args.initial_pose_yaml)

    vel_param_yaml = args.vehicle_velocity_param_yaml
    imu_param_yaml = args.imu_corrector_param_yaml
    if args.individual_params_root is not None:
        detected = detect_vehicle_config_from_path(str(args.odom_rosbag))
        if detected is None:
            print(
                "Warning: rosbag パスから vehicle 設定を推定できませんでした。"
                " --vehicle-velocity-param-yaml / --imu-corrector-param-yaml を直接指定してください",
                file=sys.stderr,
            )
        else:
            _frag, _model, vehicle_id, sensor_model = detected
            auto_vel, auto_imu = resolve_individual_param_paths(
                individual_params_root=args.individual_params_root,
                vehicle_id=vehicle_id,
                sensor_model=sensor_model,
            )
            if vel_param_yaml is None:
                vel_param_yaml = auto_vel
            if imu_param_yaml is None:
                imu_param_yaml = auto_imu
            print(
                f"Info: individual_params 自動解決: vehicle_id={vehicle_id} "
                f"sensor_model={sensor_model}",
                file=sys.stderr,
            )

    try:
        current_speed_scale, current_ang_vel_offset_z, param_meta = (
            load_current_calibration_params(
                vehicle_velocity_param_yaml=vel_param_yaml
                if vel_param_yaml is not None and vel_param_yaml.is_file()
                else None,
                imu_corrector_param_yaml=imu_param_yaml
                if imu_param_yaml is not None and imu_param_yaml.is_file()
                else None,
                fallback_speed_scale=args.current_speed_scale_factor,
                fallback_ang_vel_offset_z=args.current_angular_velocity_offset_z,
            )
        )
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2
    if vel_param_yaml is not None and not vel_param_yaml.is_file():
        print(f"Warning: vehicle velocity param が見つかりません: {vel_param_yaml}", file=sys.stderr)
    if imu_param_yaml is not None and not imu_param_yaml.is_file():
        print(f"Warning: imu corrector param が見つかりません: {imu_param_yaml}", file=sys.stderr)

    bag_dir = find_rosbag2_directory(str(args.odom_rosbag))
    wheel_bag_path = args.wheel_rosbag or args.odom_rosbag
    wheel_bag_dir = find_rosbag2_directory(str(wheel_bag_path))

    pose_series = scan_pose_series(bag_dir, args.pose_topic, 0.0, float(t_ref))
    t_start, start_source = resolve_start_unix_sec(
        cli_start=args.start_unix_sec,
        yaml_start=yaml_start,
        pose_series=pose_series,
    )
    if t_start >= float(t_ref):
        print(
            f"Error: 開始時刻 {t_start} が終了時刻 {t_ref} 以上です",
            file=sys.stderr,
        )
        return 2

    pose_series = scan_pose_series(bag_dir, args.pose_topic, t_start, float(t_ref))
    vel_series, vel_meta = scan_velocity_series(
        wheel_bag_dir, args.velocity_topic, t_start, float(t_ref)
    )
    wheel_distance_source = "velocity_status"
    kin_vel_count = 0
    if not vel_series:
        kin_vel = scan_kinematic_longitudinal_velocity_series(
            bag_dir, args.kinematic_topic, t_start, float(t_ref)
        )
        kin_vel_count = len(kin_vel)
        if kin_vel:
            vel_series = [(t, v, None) for t, v in kin_vel]
            wheel_distance_source = "kinematic_state_twist_linear_x"
            print(
                "Warning: velocity_status が無いため kinematic_state.twist.linear.x を使用します",
                file=sys.stderr,
            )
    imu_series = scan_imu_yaw_rate_series(
        wheel_bag_dir, args.imu_topic, t_start, float(t_ref)
    )

    if not pose_series:
        print(
            f"Error: 区間 [{t_start}, {t_ref}] に pose メッセージがありません",
            file=sys.stderr,
        )
        return 3
    if not vel_series:
        print(
            f"Error: 区間 [{t_start}, {t_ref}] に velocity / kinematic twist メッセージがありません\n"
            + format_velocity_scan_hint(
                wheel_bag=wheel_bag_path,
                velocity_topic=args.velocity_topic,
                kinematic_topic=args.kinematic_topic,
                t_start=t_start,
                t_ref=float(t_ref),
                vel_meta=vel_meta,
                kin_vel_count=kin_vel_count,
            ),
            file=sys.stderr,
        )
        return 3

    kin_yaw: List[Tuple[float, float]] = []
    if not vel_series or not any(hr is not None for _, _, hr in vel_series):
        kin_yaw = scan_kinematic_yaw_rate_series(
            bag_dir, args.kinematic_topic, t_start, float(t_ref)
        )

    pose_at_start = extract_nearest_pose(bag_dir, args.pose_topic, t_start)
    pose_at_ref = extract_nearest_pose(bag_dir, args.pose_topic, float(t_ref))

    try:
        result = calibrate(
            ndt_pos=ndt_pos,
            ndt_quat=ndt_quat,
            init_pos=init_pos,
            init_quat=init_quat,
            t_start=t_start,
            t_ref=float(t_ref),
            pose_series=pose_series,
            vel_series=vel_series,
            imu_series=imu_series if imu_series else kin_yaw,
            pose_at_start=pose_at_start,
            pose_at_ref=pose_at_ref,
            current_speed_scale=current_speed_scale,
            current_ang_vel_offset_z=current_ang_vel_offset_z,
            yaw_bias_method=args.yaw_bias_method,
            wheel_distance_source=wheel_distance_source,
            vehicle_velocity_param_file=param_meta.get("vehicle_velocity_param_yaml"),
            imu_corrector_param_file=param_meta.get("imu_corrector_param_yaml"),
        )
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 3

    try:
        result = finalize_calibration_result(
            result,
            correction_mode=args.correction_mode,
            lat_threshold_m=args.convergence_lat_threshold,
            lon_threshold_m=args.convergence_lon_threshold,
            yaw_threshold_deg=args.convergence_yaw_threshold_deg,
        )
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    yaml_out = args.yaml_out or default_yaml_out_path(
        args.mean_ndt_pose_yaml, args.odom_rosbag
    )
    doc = build_output_doc(
        mean_ndt_pose_yaml=args.mean_ndt_pose_yaml,
        odom_bag=args.odom_rosbag,
        initial_pose_yaml=args.initial_pose_yaml,
        ndt_pos=ndt_pos,
        ndt_quat=ndt_quat,
        init_pos=init_pos,
        init_quat=init_quat,
        pose_topic=args.pose_topic,
        velocity_topic=args.velocity_topic,
        imu_topic=args.imu_topic,
        wheel_rosbag=args.wheel_rosbag,
        start_source=start_source,
        param_meta=param_meta,
        result=result,
        yaml_out=yaml_out,
    )

    yaml_text = yaml.dump(doc, default_flow_style=False, allow_unicode=True, sort_keys=False)
    yaml_out.write_text(yaml_text, encoding="utf-8")

    print_calibration_summary(result, param_meta)
    print(f"Wrote: {yaml_out}", file=sys.stderr)

    if args.json_out:
        args.json_out.write_text(
            json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        print(f"Wrote: {args.json_out}", file=sys.stderr)

    if args.check_convergence:
        if result.get("convergence", {}).get("converged"):
            return 0
        return EXIT_NOT_CONVERGED
    return 0


if __name__ == "__main__":
    sys.exit(main())
