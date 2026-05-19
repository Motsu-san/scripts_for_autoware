#!/usr/bin/env bash
# 固定 ndt_start_pose.yaml と指定時刻の LiDAR 1 フレームで NDT を N 回評価し平均する。
#
# ndt_start_pose.yaml は rosbag と同じディレクトリに置く（例: final_merged_0.db3 と同じ階層）。
# （bag 再生用の initial_pose.yaml とは別ファイル）
#
# Usage:
#   cd <autoware_ws>
#   AUTOWARE_WS=$PWD ./measure_ndt_pose_mean.sh [options] <MAP_PATH> <SOURCE_ROSBAG> <TARGET_UNIX_SEC> [N_RUNS]
#
# Options:
#   --force-sample-vehicle  launch に sample 車両を強制
#   -h, --help
#
# 環境変数:
#   AUTOWARE_WS              既定: 実行時 pwd
#   NDT_START_POSE_YAML      省略時: rosbag と同じ階層の ndt_start_pose.yaml
#   NDT_POSE_TOPIC           既定: /localization/pose_estimator/pose_with_covariance
#   POINTCLOUD_TOPIC         既定: /sensing/lidar/concatenated/pointcloud
#   MEAN_NDT_POSE_OUTPUT_DIR 集計出力ディレクトリ
#   MEAN_NDT_POSE_YAML       出力 YAML
#   NDT_AGG_JSON             出力 JSON
#   GNSS_RECEIVER            launch へ渡す（未指定時 bag から推定）
#   EXTRA_LAUNCH_ARGS        launch 末尾に追加
#   SKIP_LAUNCH=1            Autoware 起動済みのとき（/clock は別途必要）
#   TRIAL_TIMEOUT_SEC, SETTLE_SEC, CLOUD_PUBLISH_COUNT, CLOUD_PUBLISH_HZ

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_NDT_SCRIPT="$SCRIPT_DIR/launch_localization_for_ndt_measure.sh"
MEASURE_PY="$SCRIPT_DIR/../py/measure_ndt_pose_mean.py"
AUTOWARE_WS="${AUTOWARE_WS:-$(pwd)}"

detect_gnss_receiver_from_bag() {
    local bag="$1"
    local info
    info=$(ros2 bag info "$bag" 2>&1) || {
        echo "Error: ros2 bag info failed: $bag" >&2
        exit 1
    }
    if echo "$info" | grep -qE 'Topic:.*septentrio/nav_sat_fix'; then
        echo "septentrio"
        return 0
    fi
    if echo "$info" | grep -qE 'Topic:.*ublox/nav_sat_fix'; then
        echo "ublox"
        return 0
    fi
    echo "Warning: GNSS topic not found; default ublox" >&2
    echo "ublox"
}

usage() {
    echo "Usage: AUTOWARE_WS=<ws> $0 [options] <MAP_PATH> <SOURCE_ROSBAG> <TARGET_UNIX_SEC> [N_RUNS]" >&2
    echo "  N_RUNS 既定: 100" >&2
    echo "  ndt_start_pose.yaml: rosbag と同じディレクトリ（NDT_START_POSE_YAML で上書き可）" >&2
}

FORCE_SAMPLE_VEHICLE=0
POSITIONAL=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force-sample-vehicle)
            FORCE_SAMPLE_VEHICLE=1
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

if [[ $# -lt 3 ]]; then
    usage
    exit 1
fi

MAP_PATH="$1"
SOURCE_ROSBAG="$2"
TARGET_UNIX_SEC="$3"
N_RUNS="${4:-100}"

NDT_POSE_TOPIC="${NDT_POSE_TOPIC:-/localization/pose_estimator/pose_with_covariance}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/sensing/lidar/concatenated/pointcloud}"
TRIAL_TIMEOUT_SEC="${TRIAL_TIMEOUT_SEC:-15.0}"
SETTLE_SEC="${SETTLE_SEC:-1.0}"
CLOUD_PUBLISH_COUNT="${CLOUD_PUBLISH_COUNT:-3}"
CLOUD_PUBLISH_HZ="${CLOUD_PUBLISH_HZ:-10.0}"

if [[ ! -f "$LAUNCH_NDT_SCRIPT" ]]; then
    echo "Error: $LAUNCH_NDT_SCRIPT not found" >&2
    exit 1
fi
if [[ ! -f "$MEASURE_PY" ]]; then
    echo "Error: $MEASURE_PY not found" >&2
    exit 1
fi
if [[ ! -d "$AUTOWARE_WS/install" ]]; then
    echo "Error: AUTOWARE_WS install なし: $AUTOWARE_WS/install" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "$AUTOWARE_WS/install/setup.bash"
set -u

SOURCE_ABS="$(realpath -m "$SOURCE_ROSBAG" 2>/dev/null || echo "$SOURCE_ROSBAG")"
MAP_ABS="$(realpath -m "$MAP_PATH" 2>/dev/null || echo "$MAP_PATH")"
GNSS_RECEIVER_DETECTED="$(detect_gnss_receiver_from_bag "$SOURCE_ABS")"

if [[ -d "$SOURCE_ABS" ]]; then
    ROSBAG_DIR="$SOURCE_ABS"
else
    ROSBAG_DIR="$(dirname "$SOURCE_ABS")"
fi
NDT_START_POSE_YAML="${NDT_START_POSE_YAML:-$ROSBAG_DIR/ndt_start_pose.yaml}"

if [[ ! -f "$NDT_START_POSE_YAML" ]]; then
    echo "Error: ndt_start_pose.yaml がありません（rosbag と同じ階層に置いてください）: $NDT_START_POSE_YAML" >&2
    exit 1
fi

TARGET_TAG="${TARGET_UNIX_SEC//./_}"
if [[ -z "${MEAN_NDT_POSE_OUTPUT_DIR:-}" ]]; then
    MEAN_NDT_POSE_OUTPUT_DIR="$ROSBAG_DIR/mean_ndt_pose_${TARGET_TAG}_n${N_RUNS}_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$MEAN_NDT_POSE_OUTPUT_DIR"
MEAN_NDT_POSE_YAML="${MEAN_NDT_POSE_YAML:-$MEAN_NDT_POSE_OUTPUT_DIR/mean_ndt_pose.yaml}"
OUT_JSON="${NDT_AGG_JSON:-$MEAN_NDT_POSE_OUTPUT_DIR/ndt_pose_mean_${TARGET_UNIX_SEC}_n${N_RUNS}.json}"

echo "Info: 点群元 bag: $SOURCE_ABS" >&2
echo "Info: ndt_start_pose.yaml: $NDT_START_POSE_YAML" >&2
echo "Info: 試行回数: $N_RUNS" >&2
echo "Info: 出力: $MEAN_NDT_POSE_OUTPUT_DIR" >&2
echo "Info: GNSS_RECEIVER: $GNSS_RECEIVER_DETECTED" >&2

wait_for_ndt_launch_ready() {
    local deadline=$((SECONDS + 120))
    while (( SECONDS < deadline )); do
        if ros2 service list 2>/dev/null | grep -q '/localization/pose_estimator/trigger_node'; then
            if ros2 service list 2>/dev/null | grep -q '/localization/pose_twist_fusion_filter/trigger_node'; then
                return 0
            fi
        fi
        sleep 2
    done
    echo "Error: localization サービスが 120s 以内に準備できませんでした" >&2
    return 1
}

LAUNCH_PID=""
cleanup_measure() {
    if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup_measure EXIT INT TERM

if [[ "${SKIP_LAUNCH:-0}" != "1" ]]; then
  _launch_env=(
    AUTOWARE_WS="$AUTOWARE_WS"
    GNSS_RECEIVER="$GNSS_RECEIVER_DETECTED"
    FORCE_SAMPLE_VEHICLE="$FORCE_SAMPLE_VEHICLE"
  )
  if [[ "$FORCE_SAMPLE_VEHICLE" == "1" ]]; then
    echo "Info: --force-sample-vehicle" >&2
  fi
  echo "Info: localization 起動中..." >&2
  # shellcheck disable=SC2086
  env "${_launch_env[@]}" bash "$LAUNCH_NDT_SCRIPT" "$MAP_PATH" "$SOURCE_ROSBAG" "$TARGET_UNIX_SEC" &
  LAUNCH_PID=$!

  echo "Info: localization ノード起動待ち (最大 120s)..." >&2
  sleep 15
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Error: launch プロセスが終了しました" >&2
    exit 1
  fi
  wait_for_ndt_launch_ready
else
  echo "Info: SKIP_LAUNCH=1 — 起動済み Autoware を利用します（/clock が TARGET 時刻であること）" >&2
  wait_for_ndt_launch_ready
fi

MEASURE_CMD=(
    python3 "$MEASURE_PY"
    --source-bag "$SOURCE_ABS"
    --target-unix-sec "$TARGET_UNIX_SEC"
    --initial-pose-yaml "$NDT_START_POSE_YAML"
    --n-runs "$N_RUNS"
    --pose-topic "$NDT_POSE_TOPIC"
    --pointcloud-topic "$POINTCLOUD_TOPIC"
    --trial-timeout-sec "$TRIAL_TIMEOUT_SEC"
    --settle-sec "$SETTLE_SEC"
    --cloud-publish-count "$CLOUD_PUBLISH_COUNT"
    --cloud-publish-hz "$CLOUD_PUBLISH_HZ"
    --output-json "$OUT_JSON"
    --output-mean-pose-yaml "$MEAN_NDT_POSE_YAML"
)
if [[ "${SKIP_LAUNCH:-0}" == "1" ]]; then
    MEASURE_CMD+=(--publish-clock)
fi

"${MEASURE_CMD[@]}"

echo "Wrote: $OUT_JSON" >&2
echo "Wrote: $MEAN_NDT_POSE_YAML" >&2
echo "Output dir: $MEAN_NDT_POSE_OUTPUT_DIR" >&2
