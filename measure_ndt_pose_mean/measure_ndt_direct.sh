#!/usr/bin/env bash
# 指定時刻の bag 点群 1 フレームと ndt_start_pose.yaml で direct NDT align を実行する。
# convergence_evaluator と同様、Autoware NDT ノードを起動せずライブラリを直接呼ぶ。
#
# Usage:
#   cd <autoware_ws>
#   AUTOWARE_WS=$PWD ./measure_ndt_direct.sh <MAP_PATH> <SOURCE_ROSBAG> <TARGET_UNIX_SEC>
#
# 環境変数:
#   AUTOWARE_WS              既定: 実行時 pwd
#   NDT_START_POSE_YAML      省略時: rosbag と同じ階層の ndt_start_pose.yaml
#   POINTCLOUD_TOPIC         既定: /sensing/lidar/concatenated/pointcloud
#   NDT_PARAM_YAML           省略時: ndt_direct_measure の param（max_iterations=100）
#   N_RUNS                   既定: 1
#   MAP_LOAD_MODE            all | metadata_radius (既定 metadata_radius)
#   MAP_RADIUS_M             既定: 150
#   MAP_METADATA_YAML        省略時: $MAP_PATH/pointcloud_map_metadata.yaml（無ければ親）
#   NEIGHBOR_SCANS           最近傍の前後に align するスキャン数（既定: 2）
#   NDT_DIRECT_OUTPUT_DIR    出力ディレクトリ
#   BUILD_IF_MISSING=1       実行体が無いとき colcon build する

set -euo pipefail

AUTOWARE_WS="${AUTOWARE_WS:-$(pwd)}"
TOOLS_PKG="$AUTOWARE_WS/src/tools/localization/ndt_direct_measure"

usage() {
    echo "Usage: AUTOWARE_WS=<ws> $0 <MAP_PATH> <SOURCE_ROSBAG> <TARGET_UNIX_SEC>" >&2
}

if [[ $# -lt 3 ]]; then
    usage
    exit 1
fi

MAP_PATH="$1"
SOURCE_ROSBAG="$2"
TARGET_UNIX_SEC="$3"

POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/sensing/lidar/concatenated/pointcloud}"
N_RUNS="${N_RUNS:-1}"
MAP_LOAD_MODE="${MAP_LOAD_MODE:-metadata_radius}"
MAP_RADIUS_M="${MAP_RADIUS_M:-150.0}"
NEIGHBOR_SCANS="${NEIGHBOR_SCANS:-2}"
BUILD_IF_MISSING="${BUILD_IF_MISSING:-1}"

if [[ ! -d "$AUTOWARE_WS/install" ]]; then
    echo "Error: AUTOWARE_WS install なし: $AUTOWARE_WS/install" >&2
    exit 1
fi

# pilot-auto 等の他 WS が LD_LIBRARY_PATH に残っていると、古い libmultigrid_ndt_omp.so が
# 先に解決され ndt_direct_measure_node が symbol lookup error になる。
filter_home_underlay_paths() {
    local _input="${1:-}"
    [[ -z "$_input" ]] && return 0
    echo "$_input" | tr ':' '\n' | awk -v keep="$AUTOWARE_WS/install" '
        length($0) > 0 && (index($0, keep) == 1 || index($0, ENVIRON["HOME"]) == 0) { print }
    ' | tr '\n' ':' | sed 's/:$//'
}

set +u
LD_LIBRARY_PATH="$(filter_home_underlay_paths "${LD_LIBRARY_PATH:-}")"
export LD_LIBRARY_PATH
AMENT_PREFIX_PATH="$(filter_home_underlay_paths "${AMENT_PREFIX_PATH:-}")"
export AMENT_PREFIX_PATH
# shellcheck disable=SC1090
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$AUTOWARE_WS/install/setup.bash"
LD_LIBRARY_PATH="$(filter_home_underlay_paths "${LD_LIBRARY_PATH:-}")"
export LD_LIBRARY_PATH
NDT_OMP_LIB="$AUTOWARE_WS/install/autoware_ndt_scan_matcher/lib"
if [[ -d "$NDT_OMP_LIB" ]]; then
    export LD_LIBRARY_PATH="$NDT_OMP_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
set -u

EXE="$AUTOWARE_WS/install/ndt_direct_measure/lib/ndt_direct_measure/ndt_direct_measure_node"
if [[ ! -x "$EXE" ]]; then
    if [[ "$BUILD_IF_MISSING" != "1" ]]; then
        echo "Error: $EXE not found. Build ndt_direct_measure first." >&2
        exit 1
    fi
    if [[ ! -f "$TOOLS_PKG/package.xml" ]]; then
        echo "Error: ndt_direct_measure package not found: $TOOLS_PKG" >&2
        exit 1
    fi
    echo "Info: building ndt_direct_measure..." >&2
    (cd "$AUTOWARE_WS" && colcon build --packages-select ndt_direct_measure --cmake-args -DCMAKE_BUILD_TYPE=Release)
    # shellcheck disable=SC1090
    source "$AUTOWARE_WS/install/setup.bash"
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
    echo "Error: ndt_start_pose.yaml not found: $NDT_START_POSE_YAML" >&2
    exit 1
fi

if [[ -z "${MAP_METADATA_YAML:-}" ]]; then
    if [[ -f "$MAP_ABS/pointcloud_map_metadata.yaml" ]]; then
        MAP_METADATA_YAML="$MAP_ABS/pointcloud_map_metadata.yaml"
    else
        MAP_METADATA_YAML="$(dirname "$MAP_ABS")/pointcloud_map_metadata.yaml"
    fi
fi

TARGET_TAG="${TARGET_UNIX_SEC//./_}"
if [[ -z "${NDT_DIRECT_OUTPUT_DIR:-}" ]]; then
    NDT_DIRECT_OUTPUT_DIR="$ROSBAG_DIR/ndt_direct_${TARGET_TAG}_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$NDT_DIRECT_OUTPUT_DIR"
OUT_JSON="${NDT_DIRECT_JSON:-$NDT_DIRECT_OUTPUT_DIR/ndt_direct_${TARGET_UNIX_SEC}.json}"
OUT_CSV="${NDT_DIRECT_CSV:-$NDT_DIRECT_OUTPUT_DIR/ndt_direct_${TARGET_UNIX_SEC}.csv}"

NDT_PARAM_ARGS=()
if [[ -n "${NDT_PARAM_YAML:-}" ]]; then
    NDT_PARAM_ARGS=(--ndt-param-yaml "$NDT_PARAM_YAML")
fi

echo "Info: map: $MAP_ABS" >&2
echo "Info: bag: $SOURCE_ABS" >&2
echo "Info: ndt_start_pose: $NDT_START_POSE_YAML" >&2
echo "Info: target: $TARGET_UNIX_SEC" >&2
echo "Info: map_load_mode=$MAP_LOAD_MODE radius=${MAP_RADIUS_M}m" >&2
echo "Info: neighbor_scans=$NEIGHBOR_SCANS" >&2
echo "Info: output: $NDT_DIRECT_OUTPUT_DIR" >&2

"$EXE" \
    --map-path "$MAP_ABS" \
    --source-bag "$SOURCE_ABS" \
    --target-unix-sec "$TARGET_UNIX_SEC" \
    --initial-pose-yaml "$NDT_START_POSE_YAML" \
    --pointcloud-topic "$POINTCLOUD_TOPIC" \
    --output-json "$OUT_JSON" \
    --output-csv "$OUT_CSV" \
    --n-runs "$N_RUNS" \
    --neighbor-scans "$NEIGHBOR_SCANS" \
    --map-load-mode "$MAP_LOAD_MODE" \
    --map-radius-m "$MAP_RADIUS_M" \
    --metadata-yaml "$MAP_METADATA_YAML" \
    "${NDT_PARAM_ARGS[@]}"

echo "Info: wrote $OUT_JSON" >&2
echo "Info: wrote $OUT_CSV" >&2
