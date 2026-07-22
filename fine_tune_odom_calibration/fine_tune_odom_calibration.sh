#!/usr/bin/env bash
# オドメトリ ファインチューニング: 再生&記録 → 補正値計算 → param 更新 を最大 N 回ループ。
#
# Usage:
#   cd <autoware_ws>
#   GNSS_RECEIVER=septentrio ./fine_tune_odom_calibration.sh [options] \
#     <MAP_PATH> <SOURCE_ROSBAG> \
#     --mean-ndt-pose-yaml <mean_ndt_pose.yaml>
#
# Options:
#   --max-iterations N        既定: 10
#   --rate R                  launch_autoware の再生速度（既定: 1.0）
#   --force-sample-vehicle    launch に --force-sample-vehicle を明示的に付与
#   --no-force-sample-vehicle 明示的に付与しない（既定は WS 名で自動判定）
#   --vehicle-velocity-param-yaml / --imu-corrector-param-yaml
#                             省略時は AUTOWARE_WS 内の sample_sensor_kit 設定を自動解決
#   --convergence-lat-threshold M   既定: 0.10
#   --convergence-lon-threshold M   既定: 0.10
#   --resume / --reset        状態ファイルから再開 / 削除して最初から
#   --dry-run                 launch/apply をスキップ（SKIP_LAUNCH=1 時は calib のみ）
#   --abort-on-divergence     2 連続で位置誤差が悪化したら停止
#   --initial-pose-yaml PATH  省略時は dirname(SOURCE_ROSBAG)/initial_pose.yaml
#   --pose-topic TOPIC        キャリブ用 pose（既定: EKF biased_pose / オドメトリのみ走行 POSE_SOURCE_ID=99）
#
# rosbag 再生は initial_pose 時刻 (-t) から mean_ndt_pose の target_unix_sec を
# 1の位（整数秒）切り上げした時刻 (-T) まで（キャリブ区間の終端を確実に含める）。
# calibrate 自体は target_unix_sec そのものを使う。EXTRA_LAUNCH_ARGS の -t/-T は優先される。
# apply 後は編集した param YAML を install 側へ同期し、次回 launch 前に値を検証する
# （launch は find-pkg-share 経由で install を読むため）。
#
# 環境変数:
#   AUTOWARE_WS, GNSS_RECEIVER, POSE_TOPIC, EXTRA_LAUNCH_ARGS, SKIP_LAUNCH=1, RECORD_BAG=...

DEFAULT_POSE_TOPIC="/localization/pose_twist_fusion_filter/biased_pose_with_covariance"

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_SCRIPT="$SCRIPT_DIR/../launch_replay_localization/launch_autoware.sh"
CALIBRATE_SCRIPT="$SCRIPT_DIR/calibrate_odom_from_bag.sh"
APPLY_PY="$SCRIPT_DIR/apply_odom_calibration.py"
AUTOWARE_WS="${AUTOWARE_WS:-$(pwd)}"

MAX_ITERATIONS=10
PLAYBACK_RATE="1.0"
FORCE_SAMPLE_VEHICLE=""  # 空=自動（autoware WS のみ付与）、1=強制ON、0=強制OFF
RESUME=0
RESET=0
DRY_RUN=0
ABORT_ON_DIVERGENCE=0
LAT_THRESHOLD="0.10"
LON_THRESHOLD="0.10"
CORRECTION_MODE="both"
MEAN_NDT_POSE_YAML=""
VEL_PARAM_YAML=""
IMU_PARAM_YAML=""
INITIAL_POSE_YAML=""
POSE_TOPIC="${POSE_TOPIC:-$DEFAULT_POSE_TOPIC}"
EXTRA_LAUNCH_ARGS="${EXTRA_LAUNCH_ARGS:-}"

SAMPLE_SENSOR_KIT_CONFIG_SRC_REL="src/launcher/autoware_launch/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/config"
SAMPLE_SENSOR_KIT_CONFIG_INSTALL_REL="install/sample_sensor_kit_description/share/sample_sensor_kit_description/config"

usage() {
    echo "Usage: AUTOWARE_WS=<ws> $0 [options] <MAP_PATH> <SOURCE_ROSBAG> \\" >&2
    echo "  --mean-ndt-pose-yaml <yaml>" >&2
    echo "" >&2
    echo "  param YAML 省略時: AUTOWARE_WS 内 sample_sensor_kit の vehicle_velocity_converter / imu_corrector" >&2
    echo "  pose-topic 既定: $DEFAULT_POSE_TOPIC" >&2
    echo "  --force-sample-vehicle: WS 名が autoware のときのみ既定で付与（pilot-auto 等では付与しない）" >&2
}

resolve_param_yaml_in_ws() {
    local ws="$1"
    local filename="$2"
    local candidate
    candidate="$ws/$SAMPLE_SENSOR_KIT_CONFIG_SRC_REL/$filename"
    if [[ -f "$candidate" ]]; then
        echo "$candidate"
        return 0
    fi
    candidate="$ws/$SAMPLE_SENSOR_KIT_CONFIG_INSTALL_REL/$filename"
    if [[ -f "$candidate" ]]; then
        echo "$candidate"
        return 0
    fi
    return 1
}

resolve_install_param_yaml_in_ws() {
    local ws="$1"
    local filename="$2"
    local candidate="$ws/$SAMPLE_SENSOR_KIT_CONFIG_INSTALL_REL/$filename"
    if [[ -f "$candidate" ]]; then
        echo "$candidate"
        return 0
    fi
    return 1
}

sync_param_yaml_to_install() {
    local src="$1"
    local dst="$2"
    local label="$3"
    if [[ -z "$dst" || "$src" == "$dst" ]]; then
        return 0
    fi
    if [[ ! -f "$src" ]]; then
        echo "Error: 同期元 param がありません: $src ($label)" >&2
        return 1
    fi
    mkdir -p "$(dirname "$dst")"
    cp -a "$src" "$dst"
    echo "Info: $label を install に同期: $dst" >&2
}

# launch 前: 編集ファイル→install 同期のあと、install 上の値が期待値と一致するか検証
ensure_launch_params_ready() {
    local expected_sf="${1:-}"
    local expected_bias="${2:-}"
    local mode="${3:-$CORRECTION_MODE}"

    if [[ -n "$VEL_PARAM_INSTALL_YAML" ]]; then
        sync_param_yaml_to_install "$VEL_PARAM_YAML" "$VEL_PARAM_INSTALL_YAML" \
            "vehicle_velocity_converter.param.yaml" || return 1
    fi
    if [[ -n "$IMU_PARAM_INSTALL_YAML" ]]; then
        sync_param_yaml_to_install "$IMU_PARAM_YAML" "$IMU_PARAM_INSTALL_YAML" \
            "imu_corrector.param.yaml" || return 1
    fi

    local verify_args=()
    if [[ "$mode" == "both" || "$mode" == "speed_only" ]] && [[ -n "$expected_sf" ]]; then
        verify_args+=(--expect-sf "$expected_sf")
    fi
    if [[ "$mode" == "both" || "$mode" == "yaw_only" ]] && [[ -n "$expected_bias" ]]; then
        verify_args+=(--expect-bias-z "$expected_bias")
    fi

    PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" python3 - \
        "$VEL_PARAM_YAML" \
        "$IMU_PARAM_YAML" \
        "${VEL_PARAM_INSTALL_YAML:-}" \
        "${IMU_PARAM_INSTALL_YAML:-}" \
        "$mode" \
        "${verify_args[@]}" <<'PY'
import sys
from pathlib import Path
from typing import Optional

from calibrate_odom_from_bag import load_ros2_parameters_block

vel_edit = Path(sys.argv[1])
imu_edit = Path(sys.argv[2])
vel_install_s = sys.argv[3].strip()
imu_install_s = sys.argv[4].strip()
mode = sys.argv[5]
extra = sys.argv[6:]

expect_sf = None
expect_bias = None
i = 0
while i < len(extra):
    if extra[i] == "--expect-sf" and i + 1 < len(extra):
        expect_sf = float(extra[i + 1])
        i += 2
    elif extra[i] == "--expect-bias-z" and i + 1 < len(extra):
        expect_bias = float(extra[i + 1])
        i += 2
    else:
        i += 1

vel_install = Path(vel_install_s) if vel_install_s else None
imu_install = Path(imu_install_s) if imu_install_s else None
tol = 1e-9

def read_key(path: Path, key: str) -> float:
    params = load_ros2_parameters_block(path)
    if key not in params:
        raise SystemExit(f"Error: {path} に {key} がありません")
    return float(params[key])

def check_pair(label: str, key: str, edit_path: Path, install_path: Optional[Path]) -> float:
    edit_val = read_key(edit_path, key)
    if install_path is None or install_path == edit_path:
        return edit_val
    if not install_path.is_file():
        raise SystemExit(f"Error: launch 用 install param がありません: {install_path}")
    install_val = read_key(install_path, key)
    if abs(install_val - edit_val) > tol:
        raise SystemExit(
            f"Error: {label} が edit/install で不一致です\n"
            f"  edit:    {edit_path} -> {edit_val}\n"
            f"  install: {install_path} -> {install_val}"
        )
    return install_val

checks: list = []
if mode in ("both", "speed_only"):
    checks.append(
        (
            "speed_scale_factor",
            "speed_scale_factor",
            vel_edit,
            vel_install,
            expect_sf,
        )
    )
if mode in ("both", "yaw_only"):
    checks.append(
        (
            "angular_velocity_offset_z",
            "angular_velocity_offset_z",
            imu_edit,
            imu_install,
            expect_bias,
        )
    )

for label, key, edit_path, install_path, expected in checks:
    actual = check_pair(label, key, edit_path, install_path)
    target = install_path if install_path is not None and install_path.is_file() else edit_path
    if expected is not None and abs(actual - expected) > tol:
        raise SystemExit(
            f"Error: launch 用 {label} が期待値と不一致です\n"
            f"  expected: {expected}\n"
            f"  actual:   {actual} ({target})"
        )
    print(f"OK {label}={actual} ({target})", file=sys.stderr)
PY
}

is_autoware_ws() {
    local ws="$1"
    local base
    base="$(basename "$(cd "$ws" && pwd)")"
    [[ "$base" == "autoware" ]]
}

resolve_force_sample_vehicle_default() {
    if [[ -n "$FORCE_SAMPLE_VEHICLE" ]]; then
        return 0
    fi
    if is_autoware_ws "$AUTOWARE_WS"; then
        FORCE_SAMPLE_VEHICLE=1
        echo "Info: AUTOWARE_WS が autoware のため --force-sample-vehicle を付与します" >&2
    else
        FORCE_SAMPLE_VEHICLE=0
        echo "Info: AUTOWARE_WS が autoware 以外 ($(basename "$AUTOWARE_WS")) のため --force-sample-vehicle は付与しません" >&2
    fi
}

POSITIONAL=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --max-iterations)
            MAX_ITERATIONS="$2"
            shift 2
            ;;
        --rate)
            PLAYBACK_RATE="$2"
            shift 2
            ;;
        --force-sample-vehicle)
            FORCE_SAMPLE_VEHICLE=1
            shift
            ;;
        --no-force-sample-vehicle)
            FORCE_SAMPLE_VEHICLE=0
            shift
            ;;
        --mean-ndt-pose-yaml)
            MEAN_NDT_POSE_YAML="$2"
            shift 2
            ;;
        --vehicle-velocity-param-yaml)
            VEL_PARAM_YAML="$2"
            shift 2
            ;;
        --imu-corrector-param-yaml)
            IMU_PARAM_YAML="$2"
            shift 2
            ;;
        --initial-pose-yaml)
            INITIAL_POSE_YAML="$2"
            shift 2
            ;;
        --pose-topic)
            POSE_TOPIC="$2"
            shift 2
            ;;
        --convergence-lat-threshold)
            LAT_THRESHOLD="$2"
            shift 2
            ;;
        --convergence-lon-threshold)
            LON_THRESHOLD="$2"
            shift 2
            ;;
        --resume)
            RESUME=1
            shift
            ;;
        --reset)
            RESET=1
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --abort-on-divergence)
            ABORT_ON_DIVERGENCE=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "Error: 不明なオプション: $1" >&2
            usage
            exit 1
            ;;
        *)
            POSITIONAL+=("$1")
            shift
            ;;
    esac
done
set -- "${POSITIONAL[@]}"

if [[ $# -lt 2 ]]; then
    usage
    exit 2
fi

MAP_PATH="$1"
SOURCE_ROSBAG="$2"

AUTOWARE_WS="$(cd "$AUTOWARE_WS" && pwd)"

if [[ -z "$MEAN_NDT_POSE_YAML" ]]; then
    echo "Error: --mean-ndt-pose-yaml は必須です" >&2
    exit 2
fi

if [[ -z "$VEL_PARAM_YAML" ]]; then
    if ! VEL_PARAM_YAML="$(resolve_param_yaml_in_ws "$AUTOWARE_WS" "vehicle_velocity_converter.param.yaml")"; then
        echo "Error: vehicle_velocity_converter.param.yaml を AUTOWARE_WS から解決できません: $AUTOWARE_WS" >&2
        echo "  --vehicle-velocity-param-yaml を指定してください" >&2
        exit 2
    fi
    echo "Info: vehicle_velocity_converter.param.yaml を自動解決: $VEL_PARAM_YAML" >&2
fi
if [[ -z "$IMU_PARAM_YAML" ]]; then
    if ! IMU_PARAM_YAML="$(resolve_param_yaml_in_ws "$AUTOWARE_WS" "imu_corrector.param.yaml")"; then
        echo "Error: imu_corrector.param.yaml を AUTOWARE_WS から解決できません: $AUTOWARE_WS" >&2
        echo "  --imu-corrector-param-yaml を指定してください" >&2
        exit 2
    fi
    echo "Info: imu_corrector.param.yaml を自動解決: $IMU_PARAM_YAML" >&2
fi

VEL_PARAM_INSTALL_YAML=""
IMU_PARAM_INSTALL_YAML=""
if VEL_PARAM_INSTALL_YAML="$(resolve_install_param_yaml_in_ws "$AUTOWARE_WS" "vehicle_velocity_converter.param.yaml")"; then
    echo "Info: launch 用 vehicle_velocity_converter.param.yaml (install): $VEL_PARAM_INSTALL_YAML" >&2
fi
if IMU_PARAM_INSTALL_YAML="$(resolve_install_param_yaml_in_ws "$AUTOWARE_WS" "imu_corrector.param.yaml")"; then
    echo "Info: launch 用 imu_corrector.param.yaml (install): $IMU_PARAM_INSTALL_YAML" >&2
fi
if [[ -n "$VEL_PARAM_INSTALL_YAML" && "$VEL_PARAM_YAML" != "$VEL_PARAM_INSTALL_YAML" ]]; then
    echo "Info: apply 先 (edit): $VEL_PARAM_YAML" >&2
fi
if [[ -n "$IMU_PARAM_INSTALL_YAML" && "$IMU_PARAM_YAML" != "$IMU_PARAM_INSTALL_YAML" ]]; then
    echo "Info: apply 先 (edit): $IMU_PARAM_YAML" >&2
fi

resolve_force_sample_vehicle_default
echo "Info: calibrate pose-topic: $POSE_TOPIC" >&2

for f in "$LAUNCH_SCRIPT" "$CALIBRATE_SCRIPT" "$APPLY_PY"; do
    if [[ ! -f "$f" ]]; then
        echo "Error: ファイルがありません: $f" >&2
        exit 2
    fi
done
for f in "$MAP_PATH" "$SOURCE_ROSBAG" "$MEAN_NDT_POSE_YAML" "$VEL_PARAM_YAML" "$IMU_PARAM_YAML"; do
    if [[ ! -e "$f" ]]; then
        echo "Error: パスが存在しません: $f" >&2
        exit 2
    fi
done

ROSBAG_DIR="$(cd "$(dirname "$SOURCE_ROSBAG")" && pwd)"
STATE_DIR="$ROSBAG_DIR/fine_tune_state"
STATE_FILE="$STATE_DIR/state.tsv"
mkdir -p "$STATE_DIR"

if [[ -z "$INITIAL_POSE_YAML" ]]; then
    INITIAL_POSE_YAML="$ROSBAG_DIR/initial_pose.yaml"
fi
if [[ ! -f "$INITIAL_POSE_YAML" ]]; then
    echo "Error: initial_pose.yaml がありません: $INITIAL_POSE_YAML" >&2
    exit 2
fi

resolve_playback_unix_window() {
    local out
    out=$(PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" python3 - \
        "$INITIAL_POSE_YAML" "$MEAN_NDT_POSE_YAML" <<'PY'
import math
import sys
from pathlib import Path

from calibrate_odom_from_bag import (
    load_initial_pose_yaml,
    load_ndt_mean_pose_block,
    target_unix_sec_from_ndt_meta,
)

init_path = Path(sys.argv[1])
ndt_path = Path(sys.argv[2])

_, _, start_sec, _ = load_initial_pose_yaml(init_path)
if start_sec is None:
    print(
        f"Error: initial_pose の開始時刻を読めません "
        f"(mean_pose_header_stamp / header_stamp / pose.header.stamp / header.stamp): {init_path}",
        file=sys.stderr,
    )
    sys.exit(2)

_, _, ndt_meta = load_ndt_mean_pose_block(ndt_path)
target_sec = target_unix_sec_from_ndt_meta(ndt_meta)
if target_sec is None:
    print(
        f"Error: target_unix_sec を mean_ndt_pose から読めません "
        f"(aggregated.target_unix_sec): {ndt_path}",
        file=sys.stderr,
    )
    sys.exit(2)

if target_sec <= start_sec:
    print(
        f"Error: target_unix_sec ({target_sec}) が initial_pose 時刻 ({start_sec}) 以下です",
        file=sys.stderr,
    )
    sys.exit(2)

# 指定時刻のメッセージを確実に含めるため、再生終了のみ整数秒切り上げ
playback_end_sec = math.ceil(target_sec)
print(f"{start_sec}\t{playback_end_sec}\t{target_sec}")
PY
    )
    IFS=$'\t' read -r PLAYBACK_START_UNIX PLAYBACK_END_UNIX PLAYBACK_TARGET_UNIX <<<"$out"
    echo "Info: rosbag 再生区間: -t $PLAYBACK_START_UNIX -T $PLAYBACK_END_UNIX (target_unix_sec=$PLAYBACK_TARGET_UNIX, 終了は整数秒切り上げ)" >&2
}

resolve_playback_unix_window

cleanup_on_exit() {
    if [[ -x "$SCRIPT_DIR/../launch_replay_localization/kill_autoware.sh" ]]; then
        "$SCRIPT_DIR/../launch_replay_localization/kill_autoware.sh" 2>/dev/null || true
    fi
}
trap cleanup_on_exit EXIT INT TERM HUP

write_state_header() {
    cat >"$STATE_FILE" <<EOF
# META map=$MAP_PATH bag=$SOURCE_ROSBAG mean_ndt_pose=$MEAN_NDT_POSE_YAML initial_pose=$INITIAL_POSE_YAML playback_t=$PLAYBACK_START_UNIX playback_T=$PLAYBACK_END_UNIX playback_target=$PLAYBACK_TARGET_UNIX pose_topic=$POSE_TOPIC vel_param=$VEL_PARAM_YAML vel_param_install=$VEL_PARAM_INSTALL_YAML imu_param=$IMU_PARAM_YAML imu_param_install=$IMU_PARAM_INSTALL_YAML max_iter=$MAX_ITERATIONS lat_th=$LAT_THRESHOLD lon_th=$LON_THRESHOLD correction_mode=$CORRECTION_MODE rate=$PLAYBACK_RATE force_sample=$FORCE_SAMPLE_VEHICLE
iter	record_bag	calib_yaml	lat_m	lon_m	yaw_deg	mode	sf	bias_z	converged
EOF
}

if [[ "$RESET" == "1" ]]; then
    rm -f "$STATE_FILE"
    echo "Info: 状態ファイルを削除しました: $STATE_FILE" >&2
    RESUME=0
fi

COMPLETED_ITERS=0
if [[ "$RESUME" == "1" ]]; then
    if [[ ! -f "$STATE_FILE" ]]; then
        echo "Error: --resume ですが状態ファイルがありません: $STATE_FILE" >&2
        exit 2
    fi
    COMPLETED_ITERS=$(grep -cv '^#' "$STATE_FILE" || true)
    COMPLETED_ITERS=$((COMPLETED_ITERS > 0 ? COMPLETED_ITERS - 1 : 0))
    echo "Info: --resume: 完了済み ${COMPLETED_ITERS} イテレーション。続きから実行します" >&2
elif [[ ! -f "$STATE_FILE" ]]; then
    write_state_header
fi

read_calib_metrics() {
    local calib_yaml="$1"
    python3 - "$calib_yaml" <<'PY'
import math
import sys
import yaml

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    doc = yaml.safe_load(f)
oc = doc.get("odom_calibration", doc)
yaw = oc["yaw"]
conv = oc.get("convergence", {})
corr = oc["corrections"]
lat = float(yaw["lateral_error_at_ref_m"])
lon = float(yaw["longitudinal_error_at_ref_m"])
yaw_deg = float(yaw["yaw_error_at_ref_deg"])
converged = conv.get("converged", False)
sf = float(corr["speed_scale_factor"]["recommended"])
bias = float(corr["angular_velocity_offset_z"]["recommended"])
print(f"{lat}\t{lon}\t{yaw_deg}\t{converged}\t{sf}\t{bias}")
PY
}

read_correction_mode_from_calib() {
    local calib_yaml="$1"
    python3 - "$calib_yaml" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    doc = yaml.safe_load(f)
oc = doc.get("odom_calibration", doc)
conv = oc.get("convergence", {})
methods = oc.get("correction_methods", {})
mode = conv.get("correction_mode_applied") or methods.get("correction_mode_applied") or "both"
print(mode)
PY
}

position_error_norm() {
    local lat="$1"
    local lon="$2"
    python3 - "$lat" "$lon" <<'PY'
import math
import sys
lat, lon = map(float, sys.argv[1:3])
print(math.hypot(lat, lon))
PY
}

extra_launch_has_playback_time_opt() {
    local opt="$1"
    local arg
    # shellcheck disable=SC2206
    local extra=(${EXTRA_LAUNCH_ARGS:-})
    for arg in "${extra[@]}"; do
        if [[ "$arg" == "$opt" || "$arg" == "--end-time" && "$opt" == "-T" ]]; then
            return 0
        fi
    done
    return 1
}

invoke_launch() {
    local launch_args=(
        "$LAUNCH_SCRIPT"
        "$MAP_PATH"
        "$SOURCE_ROSBAG"
        99
        false
        calibration
        --rate "$PLAYBACK_RATE"
    )
    if [[ "$FORCE_SAMPLE_VEHICLE" == "1" ]]; then
        launch_args+=(--force-sample-vehicle)
    fi
    if [[ -n "${PLAYBACK_START_UNIX:-}" && -n "${PLAYBACK_END_UNIX:-}" ]]; then
        if ! extra_launch_has_playback_time_opt "-t"; then
            launch_args+=(-t "$PLAYBACK_START_UNIX")
        fi
        if ! extra_launch_has_playback_time_opt "-T"; then
            launch_args+=(-T "$PLAYBACK_END_UNIX")
        fi
    fi
    if [[ -n "$EXTRA_LAUNCH_ARGS" ]]; then
        # shellcheck disable=SC2206
        local extra=($EXTRA_LAUNCH_ARGS)
        launch_args+=("${extra[@]}")
    fi
    echo "Info: launch: ${launch_args[*]}" >&2
    (cd "$AUTOWARE_WS" && "${launch_args[@]}")
}

find_latest_record_bag() {
    local latest=""
    latest=$(ls -td "$ROSBAG_DIR"/record_replay_* 2>/dev/null | head -1 || true)
    if [[ -z "$latest" ]]; then
        echo "Error: record_replay_* が見つかりません: $ROSBAG_DIR" >&2
        return 1
    fi
    echo "$latest"
}

PREV_ERROR_NORM=""
WORSEN_COUNT=0
NEXT_LAUNCH_EXPECT_SF=""
NEXT_LAUNCH_EXPECT_BIAS=""

if [[ "$COMPLETED_ITERS" -gt 0 && -f "$STATE_FILE" ]]; then
    last_calib_yaml=$(grep -v '^#' "$STATE_FILE" | tail -1 | cut -f3)
    if [[ -n "$last_calib_yaml" && -f "$last_calib_yaml" ]]; then
        IFS=$'\t' read -r _lat _lon _yaw _conv NEXT_LAUNCH_EXPECT_SF NEXT_LAUNCH_EXPECT_BIAS < <(
            read_calib_metrics "$last_calib_yaml"
        )
        echo "Info: --resume: 次回 launch 期待 param を iter $(printf '%02d' "$COMPLETED_ITERS") calib から復元" >&2
        echo "  speed_scale_factor=$NEXT_LAUNCH_EXPECT_SF angular_velocity_offset_z=$NEXT_LAUNCH_EXPECT_BIAS" >&2
    fi
fi

echo "Info: 初回 launch 前に param install 同期を確認します" >&2
ensure_launch_params_ready "$NEXT_LAUNCH_EXPECT_SF" "$NEXT_LAUNCH_EXPECT_BIAS" \
    "$CORRECTION_MODE" || exit 2

for ((iter = COMPLETED_ITERS + 1; iter <= MAX_ITERATIONS; iter++)); do
    printf '\n========================================\n' >&2
    printf 'Fine-tune iteration %d / %d\n' "$iter" "$MAX_ITERATIONS" >&2
    printf '========================================\n' >&2

    record_bag=""
    if [[ "${SKIP_LAUNCH:-0}" == "1" ]]; then
        if [[ -n "${RECORD_BAG:-}" ]]; then
            record_bag="$RECORD_BAG"
        else
            record_bag=$(find_latest_record_bag) || exit 3
        fi
        echo "Info: SKIP_LAUNCH=1 record_bag=$record_bag" >&2
    elif [[ "$DRY_RUN" == "1" ]]; then
        echo "Info: --dry-run launch をスキップします" >&2
        if [[ -n "${RECORD_BAG:-}" ]]; then
            record_bag="$RECORD_BAG"
        else
            record_bag=$(find_latest_record_bag) || exit 3
        fi
    else
        if [[ -n "$NEXT_LAUNCH_EXPECT_SF" || -n "$NEXT_LAUNCH_EXPECT_BIAS" ]]; then
            echo "Info: launch 前 param 検証 (install 同期済み値 == 前回 apply 推奨値)" >&2
            ensure_launch_params_ready "$NEXT_LAUNCH_EXPECT_SF" "$NEXT_LAUNCH_EXPECT_BIAS" \
                "$CORRECTION_MODE" || exit 3
        else
            echo "Info: launch 前 param install 同期を確認します" >&2
            ensure_launch_params_ready || exit 3
        fi
        invoke_launch || exit 3
        record_bag=$(find_latest_record_bag) || exit 3
    fi

    calib_yaml="$STATE_DIR/iter_$(printf '%02d' "$iter")_calibration.yaml"
    calib_args=(
        "$CALIBRATE_SCRIPT"
        "$MEAN_NDT_POSE_YAML"
        "$record_bag"
        --initial-pose-yaml "$INITIAL_POSE_YAML"
        --vehicle-velocity-param-yaml "$VEL_PARAM_YAML"
        --imu-corrector-param-yaml "$IMU_PARAM_YAML"
        --convergence-lat-threshold "$LAT_THRESHOLD"
        --convergence-lon-threshold "$LON_THRESHOLD"
        --yaml-out "$calib_yaml"
        --correction-mode "$CORRECTION_MODE"
        --yaw-bias-method start_bearing_geometry
        --pose-topic "$POSE_TOPIC"
    )
    "${calib_args[@]}" || exit 3

    PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" python3 - \
        "$calib_yaml" "$CORRECTION_MODE" "$LAT_THRESHOLD" "$LON_THRESHOLD" <<'PY'
import sys
import yaml

from calibrate_odom_from_bag import finalize_calibration_result

path = sys.argv[1]
base_mode = sys.argv[2]
lat_th = float(sys.argv[3])
lon_th = float(sys.argv[4])

with open(path, "r", encoding="utf-8") as f:
    doc = yaml.safe_load(f)
if "odom_calibration" in doc:
    oc = doc["odom_calibration"]
    wrap = True
else:
    oc = doc
    wrap = False

lon_err = abs(float(oc["yaw"]["longitudinal_error_at_ref_m"]))
effective_mode = base_mode
if base_mode == "both" and lon_err <= lon_th:
    effective_mode = "yaw_only"
    print(
        f"Info: 縦誤差 {lon_err:.3f} m <= 閾値 {lon_th:.3f} m — "
        "車速 SF は変更しません (correction_mode=yaw_only)",
        file=sys.stderr,
    )

oc = finalize_calibration_result(
    oc,
    correction_mode=effective_mode,
    lat_threshold_m=lat_th,
    lon_threshold_m=lon_th,
)
if wrap:
    doc["odom_calibration"] = oc
else:
    doc = oc
with open(path, "w", encoding="utf-8") as f:
    yaml.dump(doc, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
PY

    correction_mode=$(read_correction_mode_from_calib "$calib_yaml")

    IFS=$'\t' read -r lat lon yaw_deg converged sf bias_z < <(read_calib_metrics "$calib_yaml")
    err_norm=$(position_error_norm "$lat" "$lon")

    printf 'iter %02d | lat=%+.3f lon=%+.3f yaw=%+.3f deg | mode=%s | sf=%.6f bias_z=%.6f | converged=%s\n' \
        "$iter" "$lat" "$lon" "$yaw_deg" "$correction_mode" "$sf" "$bias_z" "$converged" >&2
    printf '  record_bag: %s\n' "$record_bag" >&2
    printf '  calib_yaml: %s\n' "$calib_yaml" >&2

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$(printf '%02d' "$iter")" "$record_bag" "$calib_yaml" \
        "$lat" "$lon" "$yaw_deg" "$correction_mode" "$sf" "$bias_z" "$converged" \
        >>"$STATE_FILE"

    if [[ "$converged" == "True" ]]; then
        echo "Info: 収束しました (iter=$iter)" >&2
        exit 0
    fi

    if [[ "$ABORT_ON_DIVERGENCE" == "1" && -n "$PREV_ERROR_NORM" ]]; then
        worsened=$(python3 - "$PREV_ERROR_NORM" "$err_norm" <<'PY'
import sys
prev, cur = map(float, sys.argv[1:3])
print("1" if cur > prev else "0")
PY
)
        if [[ "$worsened" == "1" ]]; then
            WORSEN_COUNT=$((WORSEN_COUNT + 1))
        else
            WORSEN_COUNT=0
        fi
        if [[ "$WORSEN_COUNT" -ge 2 ]]; then
            echo "Error: 位置誤差が 2 連続で悪化したため停止します" >&2
            exit 3
        fi
    fi
    PREV_ERROR_NORM="$err_norm"

    if [[ "$iter" -ge "$MAX_ITERATIONS" ]]; then
        echo "Warning: 最大イテレーション $MAX_ITERATIONS に到達（未収束）" >&2
        exit 1
    fi

    if [[ "$DRY_RUN" == "1" ]]; then
        echo "Info: --dry-run のため param 更新と次回 launch をスキップします" >&2
        continue
    fi

    python3 "$APPLY_PY" "$calib_yaml" \
        --vehicle-velocity-param-yaml "$VEL_PARAM_YAML" \
        --imu-corrector-param-yaml "$IMU_PARAM_YAML" \
        --correction-mode "$correction_mode" || exit 3

    NEXT_LAUNCH_EXPECT_SF=""
    NEXT_LAUNCH_EXPECT_BIAS=""
    if [[ "$correction_mode" == "both" || "$correction_mode" == "speed_only" ]]; then
        NEXT_LAUNCH_EXPECT_SF="$sf"
    fi
    if [[ "$correction_mode" == "both" || "$correction_mode" == "yaw_only" ]]; then
        NEXT_LAUNCH_EXPECT_BIAS="$bias_z"
    fi
    echo "Info: apply 後 install 同期と期待値検証" >&2
    ensure_launch_params_ready "$NEXT_LAUNCH_EXPECT_SF" "$NEXT_LAUNCH_EXPECT_BIAS" \
        "$correction_mode" || exit 3
done

exit 1
