#!/usr/bin/env bash
# 指定時刻の LiDAR 1 フレームと ndt_start_pose.yaml で direct NDT align を N 回実行し、
# 平均値・ばらつき・平均値からの最大偏差を出力する。
#
# 旧版は Autoware localization を起動して NDT ノードの pose publish を待っていたが、
# 現版は ~/autoware/src/tools/localization/ndt_direct_measure の direct align 実行体を使う。
#
# Usage:
#   cd <autoware_ws>
#   AUTOWARE_WS=$PWD ./measure_ndt_pose_mean.sh [options] <MAP_PATH> <SOURCE_ROSBAG> <TARGET_UNIX_SEC> [N_RUNS]
#
# Options:
#   --force-sample-vehicle  互換用に受け取るが direct 方式では未使用
#   -h, --help
#
# 環境変数:
#   AUTOWARE_WS              既定: 実行時 pwd
#   NDT_START_POSE_YAML      省略時: rosbag と同じ階層の ndt_start_pose.yaml
#   POINTCLOUD_TOPIC         既定: /sensing/lidar/concatenated/pointcloud
#   MEAN_NDT_POSE_OUTPUT_DIR 集計出力ディレクトリ
#   MEAN_NDT_POSE_YAML       出力 YAML
#   NDT_AGG_JSON             集計 JSON
#   NDT_DIRECT_RAW_JSON      direct 実行体の raw JSON
#   NDT_DIRECT_RAW_CSV       direct 実行体の raw CSV
#   NDT_PARAM_YAML           NDT パラメータ YAML
#   MAP_LOAD_MODE            all | metadata_radius（既定: metadata_radius）
#   MAP_RADIUS_M             metadata_radius の半径 [m]（既定: 150）
#   MAP_METADATA_YAML        pointcloud_map_metadata.yaml
#   NEIGHBOR_SCANS           最近傍の前後に align するスキャン数（既定: 2）
#   BUILD_IF_MISSING=1       ndt_direct_measure 未ビルド時に colcon build

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIRECT_SH="$SCRIPT_DIR/measure_ndt_direct.sh"
AGG_PY="$SCRIPT_DIR/../py/aggregate_ndt_direct_result.py"
AUTOWARE_WS="${AUTOWARE_WS:-$(pwd)}"

usage() {
    echo "Usage: AUTOWARE_WS=<ws> $0 [options] <MAP_PATH> <SOURCE_ROSBAG> <TARGET_UNIX_SEC> [N_RUNS]" >&2
    echo "  N_RUNS 既定: 100" >&2
    echo "  ndt_start_pose.yaml: rosbag と同じ階層（NDT_START_POSE_YAML で上書き可）" >&2
}

POSITIONAL=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force-sample-vehicle)
            echo "Info: --force-sample-vehicle は direct 方式では未使用です" >&2
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

if [[ ! -f "$DIRECT_SH" ]]; then
    echo "Error: $DIRECT_SH not found" >&2
    exit 1
fi
if [[ ! -f "$AGG_PY" ]]; then
    echo "Error: $AGG_PY not found" >&2
    exit 1
fi

SOURCE_ABS="$(realpath -m "$SOURCE_ROSBAG" 2>/dev/null || echo "$SOURCE_ROSBAG")"
MAP_ABS="$(realpath -m "$MAP_PATH" 2>/dev/null || echo "$MAP_PATH")"

if [[ -d "$SOURCE_ABS" ]]; then
    ROSBAG_DIR="$SOURCE_ABS"
else
    ROSBAG_DIR="$(dirname "$SOURCE_ABS")"
fi

NDT_START_POSE_YAML="${NDT_START_POSE_YAML:-$ROSBAG_DIR/ndt_start_pose.yaml}"
if [[ ! -f "$NDT_START_POSE_YAML" ]]; then
    echo "Error: ndt_start_pose.yaml がありません: $NDT_START_POSE_YAML" >&2
    exit 1
fi

TARGET_TAG="${TARGET_UNIX_SEC//./_}"
if [[ -z "${MEAN_NDT_POSE_OUTPUT_DIR:-}" ]]; then
    MEAN_NDT_POSE_OUTPUT_DIR="$ROSBAG_DIR/mean_ndt_pose_direct_${TARGET_TAG}_n${N_RUNS}_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$MEAN_NDT_POSE_OUTPUT_DIR"

RAW_JSON="${NDT_DIRECT_RAW_JSON:-$MEAN_NDT_POSE_OUTPUT_DIR/ndt_direct_runs_${TARGET_UNIX_SEC}_n${N_RUNS}.json}"
RAW_CSV="${NDT_DIRECT_RAW_CSV:-$MEAN_NDT_POSE_OUTPUT_DIR/ndt_direct_runs_${TARGET_UNIX_SEC}_n${N_RUNS}.csv}"
OUT_JSON="${NDT_AGG_JSON:-$MEAN_NDT_POSE_OUTPUT_DIR/ndt_pose_mean_${TARGET_UNIX_SEC}_n${N_RUNS}.json}"
MEAN_NDT_POSE_YAML="${MEAN_NDT_POSE_YAML:-$MEAN_NDT_POSE_OUTPUT_DIR/mean_ndt_pose.yaml}"

echo "Info: direct NDT mean mode" >&2
echo "Info: map: $MAP_ABS" >&2
echo "Info: bag: $SOURCE_ABS" >&2
echo "Info: ndt_start_pose.yaml: $NDT_START_POSE_YAML" >&2
echo "Info: target: $TARGET_UNIX_SEC" >&2
echo "Info: runs: $N_RUNS" >&2
echo "Info: neighbor_scans: ${NEIGHBOR_SCANS:-2}" >&2
echo "Info: output: $MEAN_NDT_POSE_OUTPUT_DIR" >&2

AUTOWARE_WS="$AUTOWARE_WS" \
NDT_START_POSE_YAML="$NDT_START_POSE_YAML" \
N_RUNS="$N_RUNS" \
NEIGHBOR_SCANS="${NEIGHBOR_SCANS:-2}" \
NDT_DIRECT_OUTPUT_DIR="$MEAN_NDT_POSE_OUTPUT_DIR" \
NDT_DIRECT_JSON="$RAW_JSON" \
NDT_DIRECT_CSV="$RAW_CSV" \
"$DIRECT_SH" "$MAP_ABS" "$SOURCE_ABS" "$TARGET_UNIX_SEC"

python3 "$AGG_PY" \
    --input-json "$RAW_JSON" \
    --output-json "$OUT_JSON" \
    --output-mean-pose-yaml "$MEAN_NDT_POSE_YAML"

echo "Wrote raw direct JSON: $RAW_JSON" >&2
echo "Wrote raw direct CSV:  $RAW_CSV" >&2
echo "Wrote aggregate JSON:  $OUT_JSON" >&2
echo "Wrote mean pose YAML:  $MEAN_NDT_POSE_YAML" >&2
echo "Output dir: $MEAN_NDT_POSE_OUTPUT_DIR" >&2
exit 0
