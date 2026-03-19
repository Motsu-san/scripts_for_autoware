#!/bin/bash

# launch_unified_localization.sh — unified_localization (NDT+EKF in one node) 専用ランチスクリプト
# Usage: ./launch_unified_localization.sh <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE] [--compare-bag COMPARE_BAG] [--compare-topics TOPIC1 TOPIC2 ...] [-t UNIX_TIME]
# POSE_SOURCE_ID: 0=ndt, 1=ndt_lidar-marker（ログ等の表記用。unified_localization は単一ノードのため launch には NDT のみ使用）
# SAVE_LAUNCH_LOG: "true" で ros2 launch ログ保存
# TOPIC_TYPE: record_rosbag 用（省略時は録画なし）
# Rosbag には /localization/pose_twist_estimator/twist を含めるか、gyro_odometer を別途起動。点群は /sensing/lidar/concatenated/pointcloud。
#
# sample-rosbag 再生時は use_sim_time=false かつ --clock なしで再生（自動）。
# use_sim_time をオフにする場合: USE_SIM_TIME=false ./launch_unified_localization.sh <MAP_PATH> <ROSBAG_PATH> ...
#
# Requirements (必須・条件付きで参照するファイル。先頭で存在チェックする):
#   - 常時: $SCRIPT_DIR/vehicle_configs.sh
#   - 常時: $HOME/scripts_for_autoware/sh/kill_autoware.sh
#   - 常時: カレントディレクトリが autoware ビルド済みで install/setup.bash が存在すること（引数チェックで検証）
#   - TOPIC_TYPE 指定時: $SCRIPT_DIR/record_rosbag_localization_replay.sh
#   - --compare-bag 指定時: $HOME/scripts_for_autoware/py/play_multiple_rosbags.py
#   - 使用時に存在確認: scripts_for_autoware/py/set_initial_pose.py, gnss_to_initial_pose.py
#   - unified_localization 起動時: autoware_unified_localization パッケージ（launch 内で検証）

CALL_DIR=$(pwd)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f "$SCRIPT_DIR/vehicle_configs.sh" ]; then
    echo "Error: Required file not found: $SCRIPT_DIR/vehicle_configs.sh" >&2
    exit 1
fi
source "$SCRIPT_DIR/vehicle_configs.sh"

# 位置引数の解析
POSITIONAL_ARGS=()
COMPARE_BAG=""
COMPARE_TOPICS=()
START_UNIX_TIME=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --compare-bag)
            COMPARE_BAG="$2"
            shift 2
            ;;
        --compare-topics)
            shift
            while [[ $# -gt 0 ]] && [[ ! "$1" =~ ^-- ]]; do
                COMPARE_TOPICS+=("$1")
                shift
            done
            ;;
        -t)
            START_UNIX_TIME="$2"
            shift 2
            ;;
        *)
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done

set -- "${POSITIONAL_ARGS[@]}"

if [ $# -lt 2 ] || [ $# -gt 5 ] || [ ! -f "$CALL_DIR/install/setup.bash" ]; then
    echo "Usage: $0 <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE] [--compare-bag COMPARE_BAG] [--compare-topics TOPIC1 TOPIC2 ...] [-t UNIX_TIME]"
    echo "Unified localization only. Call from the directory where autoware is located and built (e.g. \$HOME/autoware)."
    echo "POSE_SOURCE_ID: 0=ndt, 1=ndt_lidar-marker (for logging)"
    echo "SAVE_LAUNCH_LOG: 'true' to save ros2 launch log"
    echo "TOPIC_TYPE: default, lidar-marker_replay, full-sensing_replay, output, output_lidar-marker, convergence_evaluation, occlusion_adding (omit to disable recording)"
    echo "--compare-bag, --compare-topics, -t: same as launch_autoware.sh"
    exit 1
fi

MAP_PATH="$1"
ROSBAG="$2"
POSE_SOURCE_ID="${3:-0}"
SAVE_LAUNCH_LOG="${4:-false}"
TOPIC_TYPE="${5:-}"

if [ -n "$COMPARE_BAG" ] && [ ${#COMPARE_TOPICS[@]} -eq 0 ]; then
    echo "Error: --compare-topics must be specified when using --compare-bag" >&2
    exit 1
fi
if [ ${#COMPARE_TOPICS[@]} -gt 0 ] && [ -z "$COMPARE_BAG" ]; then
    echo "Error: --compare-bag must be specified when using --compare-topics" >&2
    exit 1
fi

# 必須ファイルの存在チェック
MISSING=()
[ ! -f "$HOME/scripts_for_autoware/sh/kill_autoware.sh" ] && MISSING+=("$HOME/scripts_for_autoware/sh/kill_autoware.sh")
[ -n "$TOPIC_TYPE" ] && [ ! -f "$SCRIPT_DIR/record_rosbag_localization_replay.sh" ] && MISSING+=("$SCRIPT_DIR/record_rosbag_localization_replay.sh")
[ -n "$COMPARE_BAG" ] && [ ! -f "$HOME/scripts_for_autoware/py/play_multiple_rosbags.py" ] && MISSING+=("$HOME/scripts_for_autoware/py/play_multiple_rosbags.py")
if [ ${#MISSING[@]} -gt 0 ]; then
    echo "Error: Required file(s) not found:" >&2
    printf '  %s\n' "${MISSING[@]}" >&2
    exit 1
fi

LOG_DIR=$HOME/log

LAUNCH_DRIVER="false"
LAUNCH_SENSING="true"
LAUNCH_PERCEPTION="false"
LAUNCH_PLANNING="false"
LAUNCH_CONTROL="false"
# このスクリプトは unified_localization 専用
LAUNCH_UNIFIED_LOCALIZATION="true"
LAUNCH_LOCALIZATION_ARG="false"

parse_jst_to_unix_sec() {
    local s="$1"
    local frac=""
    if [[ "$s" =~ \.[0-9]+$ ]]; then
        frac="${s##*.}"
        s="${s%.*}"
    fi
    s="${s//\//-}"
    local unix_sec
    unix_sec=$(TZ=Asia/Tokyo date -d "$s" +%s 2>/dev/null) || return 1
    [[ -z "$unix_sec" ]] && return 1
    if [[ -n "$frac" ]]; then
        echo "${unix_sec}.${frac}"
    else
        echo "$unix_sec"
    fi
}

get_bag_start_unix_sec() {
    local bag_path="$1"
    local start_line
    start_line=$(ros2 bag info "$bag_path" 2>/dev/null | grep '^Start:')
    [[ -z "$start_line" ]] && return 1
    local unix_sec
    unix_sec=$(echo "$start_line" | sed -n 's/.*(\([0-9][0-9]*\.\?[0-9]*\)).*/\1/p')
    [[ -z "$unix_sec" ]] && return 1
    echo "$unix_sec"
}

get_pose_source() {
    case "$1" in
        0) echo "ndt" ;;
        1) echo "ndt_lidar-marker" ;;
        *)
            echo "Error: Invalid POSE_SOURCE_ID: $1. Valid values are: 0 (ndt), 1 (ndt_lidar-marker)" >&2
            exit 1
            ;;
    esac
}

POSE_SOURCE=$(get_pose_source "$POSE_SOURCE_ID")
USE_SIM_TIME="${USE_SIM_TIME:-true}"
if [[ "$ROSBAG" == *"sample-rosbag"* ]] && [ -z "$COMPARE_BAG" ]; then
  USE_SIM_TIME="false"
fi
RVIZ="true"
RVIZ_CONFIG="$CALL_DIR/install/autoware_launch/share/autoware_launch/rviz/autoware.rviz"

if [ "$SAVE_LAUNCH_LOG" = "true" ]; then
    LAUNCH_LOG_FILE="$LOG_DIR/ros2_launch_$(date '+%Y%m%d_%H%M%S').log"
    mkdir -p "$LOG_DIR"
    touch "$LAUNCH_LOG_FILE"
    echo "ros2 launch log will be saved to $LAUNCH_LOG_FILE" | tee -a "$LAUNCH_LOG_FILE"
else
    LAUNCH_LOG_FILE=""
fi

if [ ! -d "$MAP_PATH" ]; then
    echo "Error: MAP_PATH does not exist: $MAP_PATH"
    exit 1
fi
if [ ! -f "$ROSBAG" ] && [ ! -d "$ROSBAG" ]; then
    echo "Error: ROSBAG path does not exist: $ROSBAG"
    exit 1
fi

START_OFFSET_SEC=""
PLAY_OFFSET_ARGS=()
if [ -n "$START_UNIX_TIME" ]; then
    if [[ "$START_UNIX_TIME" =~ ^[0-9]+\.?[0-9]*$ ]]; then
        :
    else
        START_UNIX_TIME=$(parse_jst_to_unix_sec "$START_UNIX_TIME")
        if [ -z "$START_UNIX_TIME" ]; then
            echo "Error: -t could not parse as JST datetime" >&2
            exit 1
        fi
        echo "Parsed -t as JST -> UNIX time: $START_UNIX_TIME"
    fi
fi

DATETIME=$(date '+%Y%m%d_%H%M%S')

echo "MAP_PATH: $MAP_PATH" | tee -a $LAUNCH_LOG_FILE
echo "ROSBAG: $ROSBAG" | tee -a $LAUNCH_LOG_FILE
echo "DATETIME: $DATETIME" | tee -a $LAUNCH_LOG_FILE
echo "POSE_SOURCE: $POSE_SOURCE (unified_localization)" | tee -a $LAUNCH_LOG_FILE
[ -n "$TOPIC_TYPE" ] && echo "TOPIC_TYPE: $TOPIC_TYPE (rosbag recording enabled)" | tee -a $LAUNCH_LOG_FILE || echo "TOPIC_TYPE: (not specified, rosbag recording disabled)" | tee -a $LAUNCH_LOG_FILE
[ -n "$COMPARE_BAG" ] && echo "COMPARE_BAG: $COMPARE_BAG" | tee -a $LAUNCH_LOG_FILE && echo "COMPARE_TOPICS: ${COMPARE_TOPICS[*]}" | tee -a $LAUNCH_LOG_FILE
[ -n "$START_UNIX_TIME" ] && echo "START_UNIX_TIME (-t): $START_UNIX_TIME" | tee -a $LAUNCH_LOG_FILE

VEHICLE_CONFIG=$(detect_vehicle_config "$ROSBAG")
if [ $? -ne 0 ]; then
    echo "Error: Vehicle configuration not found for ROSBAG path: $ROSBAG"
    echo "Available vehicle ID fragments (edit $SCRIPT_DIR/vehicle_configs.sh to add):"
    for config in "${VEHICLE_CONFIGS[@]}"; do
        [[ -z "$config" || "$config" =~ ^[[:space:]]*# ]] && continue
        IFS='|' read -r vehicle_fragment vehicle_model_val vehicle_id_val sensor_model_val <<< "$config"
        echo "  $vehicle_fragment -> $vehicle_model_val, $vehicle_id_val, $sensor_model_val"
    done
    exit 1
fi

IFS='|' read -r VEHICLE_MODEL VEHICLE_ID SENSOR_MODEL <<< "$VEHICLE_CONFIG"

echo "Detected vehicle configuration:" | tee -a $LAUNCH_LOG_FILE
echo "  VEHICLE_MODEL: $VEHICLE_MODEL" | tee -a $LAUNCH_LOG_FILE
echo "  VEHICLE_ID: $VEHICLE_ID" | tee -a $LAUNCH_LOG_FILE
echo "  SENSOR_MODEL: $SENSOR_MODEL" | tee -a $LAUNCH_LOG_FILE
echo "Launch configuration (unified_localization only):" | tee -a $LAUNCH_LOG_FILE
echo "  LAUNCH_DRIVER: $LAUNCH_DRIVER" | tee -a $LAUNCH_LOG_FILE
echo "  SENSING: $LAUNCH_SENSING" | tee -a $LAUNCH_LOG_FILE
echo "  PERCEPTION: $LAUNCH_PERCEPTION" | tee -a $LAUNCH_LOG_FILE
echo "  PLANNING: $LAUNCH_PLANNING" | tee -a $LAUNCH_LOG_FILE
echo "  CONTROL: $LAUNCH_CONTROL" | tee -a $LAUNCH_LOG_FILE
echo "  localization:=false, then unified_localization" | tee -a $LAUNCH_LOG_FILE
echo "  RVIZ: $RVIZ" | tee -a $LAUNCH_LOG_FILE
echo "  USE_SIM_TIME: $USE_SIM_TIME" | tee -a $LAUNCH_LOG_FILE

trap "$HOME/scripts_for_autoware/sh/kill_autoware.sh" EXIT INT TERM HUP

ROSBAG_DIR=$(dirname "$ROSBAG")
OUTPUT_DIR=$ROSBAG_DIR/record_replay_$DATETIME
if [ -d "$OUTPUT_DIR" ]; then
    echo "Warning: OUTPUT_DIR already exists, removing: $OUTPUT_DIR" | tee -a $LAUNCH_LOG_FILE
    rm -rf "$OUTPUT_DIR"
fi
echo "OUTPUT_DIR will be created by record script: $OUTPUT_DIR" | tee -a $LAUNCH_LOG_FILE

cd $(dirname $0)

AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | awk -v keep="$CALL_DIR/install" '
  index($0, keep) == 1 || index($0, ENVIRON["HOME"]) == 0 { print }
' | tr '\n' ':' | sed 's/:$//')
export AMENT_PREFIX_PATH

source /opt/ros/humble/setup.bash
source $CALL_DIR/install/setup.bash

if [ -n "$START_UNIX_TIME" ]; then
    BAG_START_SEC=$(get_bag_start_unix_sec "$ROSBAG")
    if [ -z "$BAG_START_SEC" ]; then
        echo "Error: Could not get bag start time for -t option" >&2
        exit 1
    fi
    START_OFFSET_SEC=$(echo "$START_UNIX_TIME $BAG_START_SEC" | awk '{s=$1-$2; if(s<0)s=0; printf "%.3f", s}')
    PLAY_OFFSET_ARGS=(--start-offset "$START_OFFSET_SEC")
    echo "Start from UNIX time -t $START_UNIX_TIME (bag start: $BAG_START_SEC) -> --start-offset ${START_OFFSET_SEC}s"
fi

# メイン launch（localization は起動しない）
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$MAP_PATH \
    vehicle_model:=$VEHICLE_MODEL \
    vehicle_id:=$VEHICLE_ID \
    sensor_model:=$SENSOR_MODEL \
    launch_driver:=$LAUNCH_DRIVER \
    sensing:=$LAUNCH_SENSING \
    perception:=$LAUNCH_PERCEPTION \
    planning:=$LAUNCH_PLANNING \
    control:=$LAUNCH_CONTROL \
    localization:=false \
    use_sim_time:=$USE_SIM_TIME \
    rviz:=$RVIZ \
    rviz_config:=$RVIZ_CONFIG \
    pose_source:=$POSE_SOURCE \
    | tee -a $LAUNCH_LOG_FILE &

# unified_localization 用: trigger_node は呼ばない
echo "Unified localization mode: skipping trigger_node." | tee -a $LAUNCH_LOG_FILE
sleep 3

# unified_localization 用: pointcloud util → gyro_odometer → unified_localization
UNIFIED_PARAM=$(ros2 pkg prefix autoware_unified_localization 2>/dev/null)/share/autoware_unified_localization/config/unified_localization_pose_sources_ndt_only_autoware.param.yaml
if [ ! -f "$UNIFIED_PARAM" ]; then
    echo "Warning: unified_localization param not found at $UNIFIED_PARAM; run from autoware workspace with autoware_unified_localization built." | tee -a $LAUNCH_LOG_FILE
else
    echo "Launching localization util (pointcloud downsampling) for unified_localization..." | tee -a $LAUNCH_LOG_FILE
    ros2 launch autoware_launch localization_util_downsample.launch.xml \
      input_pointcloud:=/sensing/lidar/concatenated/pointcloud \
      localization_pointcloud_container_name:=/pointcloud_container \
      2>&1 | tee -a $LAUNCH_LOG_FILE &
    sleep 2

    echo "Launching gyro_odometer for unified_localization twist input..." | tee -a $LAUNCH_LOG_FILE
    ros2 run autoware_gyro_odometer autoware_gyro_odometer_node \
      --ros-args \
      -p use_sim_time:=true \
      -p output_frame:=base_link \
      -p message_timeout_sec:=0.2 \
      --remap vehicle/twist_with_covariance:=/sensing/vehicle_velocity_converter/twist_with_covariance \
      --remap imu:=/sensing/imu/imu_data \
      --remap twist_with_covariance:=/localization/twist_estimator/twist_with_covariance \
      2>&1 | tee -a $LAUNCH_LOG_FILE &
    sleep 1

    echo "Launching unified_localization (param=$UNIFIED_PARAM)..." | tee -a $LAUNCH_LOG_FILE
    ros2 launch autoware_unified_localization unified_localization.launch.py \
      param_file:="$UNIFIED_PARAM" \
      use_sim_time:=true \
      output_kinematic_state:=/localization/kinematic_state \
      output_acceleration:=/localization/acceleration \
      input_twist_with_covariance:=/localization/twist_estimator/twist_with_covariance \
      2>&1 | tee -a $LAUNCH_LOG_FILE &
fi
sleep 2

if [ -n "$TOPIC_TYPE" ]; then
    echo "Starting rosbag recording (TOPIC_TYPE=$TOPIC_TYPE)..." | tee -a $LAUNCH_LOG_FILE
    ./record_rosbag_localization_replay.sh $OUTPUT_DIR $TOPIC_TYPE &
else
    echo "Rosbag recording disabled (TOPIC_TYPE not specified)" | tee -a $LAUNCH_LOG_FILE
fi

echo "Starting rosbag playback..." | tee -a $LAUNCH_LOG_FILE

if [ -n "$COMPARE_BAG" ]; then
    PLAY_MULTIPLE_SCRIPT="$HOME/scripts_for_autoware/py/play_multiple_rosbags.py"
    if [ ! -f "$PLAY_MULTIPLE_SCRIPT" ]; then
        echo "Error: play_multiple_rosbags.py not found at $PLAY_MULTIPLE_SCRIPT" | tee -a $LAUNCH_LOG_FILE
        echo "Falling back to single rosbag playback..." | tee -a $LAUNCH_LOG_FILE
        ros2 bag play ${ROSBAG} -r 1.0 $([ "$USE_SIM_TIME" = "true" ] && echo "--clock 200") "${PLAY_OFFSET_ARGS[@]}" 2>&1 | tee -a $LAUNCH_LOG_FILE &
        ROSBAG_PID=$!
    else
        echo "Using play_multiple_rosbags.py for simultaneous playback..." | tee -a $LAUNCH_LOG_FILE
        if [ ! -f "$COMPARE_BAG" ] && [ ! -d "$COMPARE_BAG" ]; then
            echo "Error: Compare bag not found: $COMPARE_BAG" | tee -a $LAUNCH_LOG_FILE
            ros2 bag play ${ROSBAG} -r 1.0 $([ "$USE_SIM_TIME" = "true" ] && echo "--clock 200") "${PLAY_OFFSET_ARGS[@]}" 2>&1 | tee -a $LAUNCH_LOG_FILE &
            ROSBAG_PID=$!
        else
            python3 -u "$PLAY_MULTIPLE_SCRIPT" \
                --source-bag "$ROSBAG" \
                --recorded-bag "$COMPARE_BAG" \
                --recorded-topics "${COMPARE_TOPICS[@]}" \
                --remap /localization/pose_twist_fusion_filter/biased_pose_with_covariance:=/localization/pose_twist_fusion_filter/biased_pose_with_covariance_recorded \
                -r 1.0 \
                $([ "$USE_SIM_TIME" = "true" ] && echo "--clock 200") \
                "${PLAY_OFFSET_ARGS[@]}" \
                2>&1 | tee -a $LAUNCH_LOG_FILE &
            ROSBAG_PID=$!
        fi
    fi
else
    ros2 bag play ${ROSBAG} -r 0.2 $([ "$USE_SIM_TIME" = "true" ] && echo "--clock 200") "${PLAY_OFFSET_ARGS[@]}" 2>&1 | tee -a $LAUNCH_LOG_FILE &
    ROSBAG_PID=$!
fi

SENSOR_TOPIC="/sensing/lidar/concatenated/pointcloud"
echo "Waiting for first message on $SENSOR_TOPIC (timeout 60s)..." | tee -a $LAUNCH_LOG_FILE
if timeout 60 ros2 topic echo "$SENSOR_TOPIC" --once > /dev/null 2>&1; then
    echo "Sensor data started flowing." | tee -a $LAUNCH_LOG_FILE
else
    echo "Warning: Timeout waiting for $SENSOR_TOPIC (proceeding anyway)." | tee -a $LAUNCH_LOG_FILE
fi

INITIAL_POSE_YAML="${ROSBAG_DIR}/initial_pose.yaml"
echo "DEBUG: Checking for initial_pose.yaml at: $INITIAL_POSE_YAML" | tee -a $LAUNCH_LOG_FILE

if [ -f "$INITIAL_POSE_YAML" ]; then
    echo "Setting initial pose from $INITIAL_POSE_YAML..." | tee -a $LAUNCH_LOG_FILE
    python3 $(dirname $0)/../py/set_initial_pose.py "$INITIAL_POSE_YAML" 2>&1 | tee -a $LAUNCH_LOG_FILE
    POSE_SET_RESULT=${PIPESTATUS[0]}
    if [ $POSE_SET_RESULT -eq 0 ]; then
        echo "Initial pose set successfully" | tee -a $LAUNCH_LOG_FILE
    else
        echo "Error: Failed to set initial pose (exit code: $POSE_SET_RESULT)" | tee -a $LAUNCH_LOG_FILE
    fi
    sleep 5
else
    echo "Info: initial_pose.yaml not found at $INITIAL_POSE_YAML" | tee -a $LAUNCH_LOG_FILE
    echo "Unified localization: setting initial pose from GNSS (/sensing/gnss/pose_with_covariance)..." | tee -a $LAUNCH_LOG_FILE
    GNSS_POSE_SCRIPT="$(dirname $0)/../py/gnss_to_initial_pose.py"
    if [ -f "$GNSS_POSE_SCRIPT" ]; then
        python3 "$GNSS_POSE_SCRIPT" --timeout 45 2>&1 | tee -a $LAUNCH_LOG_FILE
        GNSS_RESULT=${PIPESTATUS[0]}
        if [ $GNSS_RESULT -eq 0 ]; then
            echo "Initial pose set from GNSS successfully" | tee -a $LAUNCH_LOG_FILE
            sleep 3
        else
            echo "Warning: Could not get initial pose from GNSS. Set manually via RViz (2D Pose Estimate)." | tee -a $LAUNCH_LOG_FILE
        fi
    else
        echo "Warning: $GNSS_POSE_SCRIPT not found. Set initial pose manually via RViz (2D Pose Estimate)." | tee -a $LAUNCH_LOG_FILE
    fi
fi

echo "Waiting for rosbag playback to complete..." | tee -a $LAUNCH_LOG_FILE
wait $ROSBAG_PID
