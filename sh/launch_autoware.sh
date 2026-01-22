#!/bin/bash

# Usage: ./launch_autoware.sh <MAP_PATH> <ROSBAG_PATH>
# Example: ./launch_autoware.sh "$HOME/autoware_map/Komatsu/628-20250619100818540138" "$HOME/rosbag_replay/data1/final_merged/final_merged_0.db3"

CALL_DIR=$(pwd)

# Usage: ./launch_autoware.sh <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE]
# POSE_SOURCE_ID: 0=ndt (default), 1=ndt_lidar-marker, other numbers can be added in the future
# SAVE_LAUNCH_LOG: "true" to save ros2 launch log, anything else or omitted disables log saving
# TOPIC_TYPE: Topic type for record_rosbag.sh (default, lidar-marker_replay, full-sensing_replay, output, output_lidar-marker, convergence_evaluation, occlusion_adding)
#   If TOPIC_TYPE is omitted, rosbag recording will be disabled
if [ $# -lt 2 ] || [ $# -gt 5 ] || [ ! -f "$CALL_DIR/install/setup.bash" ]; then
    echo "Usage: $0 <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE]"
    echo "Must provide <MAP_PATH> <ROSBAG_PATH> and call from the directory where autoware is located and built(ex. $HOME/autoware)."
    echo "POSE_SOURCE_ID: 0=ndt (default), 1=ndt_lidar-marker"
    echo "SAVE_LAUNCH_LOG: 'true' to save ros2 launch log, anything else or omitted disables log saving"
    echo "TOPIC_TYPE: default, lidar-marker_replay, full-sensing_replay, output, output_lidar-marker, convergence_evaluation, occlusion_adding"
    echo "  If TOPIC_TYPE is omitted, rosbag recording will be disabled"
    echo "Example: $0 \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 1 true output"
    exit 1
fi

# $HOME/autoware_map/Shimizu_port/1605-20250121101014951069

MAP_PATH="$1"
ROSBAG="$2"
POSE_SOURCE_ID="${3:-0}"
SAVE_LAUNCH_LOG="${4:-false}"
TOPIC_TYPE="${5:-}"

LOG_DIR=$HOME/log

LAUNCH_DRIVER="true"  # true for rosbag replay with packets, false for pointcloud_raw
LAUNCH_SENSING="true"
LAUNCH_PERCEPTION="false"
LAUNCH_PLANNING="false"
LAUNCH_CONTROL="false"

# Function to convert pose_source_id to pose_source string
# 0: ndt (default)
# 1: ndt_lidar-marker
# Other numbers can be added in the future
get_pose_source() {
    case "$1" in
        0)
            echo "ndt"
            ;;
        1)
            echo "ndt_lidar-marker"
            ;;
        *)
            echo "Error: Invalid POSE_SOURCE_ID: $1. Valid values are: 0 (ndt), 1 (ndt_lidar-marker)" >&2
            exit 1
            ;;
    esac
}

POSE_SOURCE=$(get_pose_source "$POSE_SOURCE_ID")
USE_SIM_TIME="true"  # Always true for rosbag replay
RVIZ="true"
RVIZ_CONFIG="$CALL_DIR/install/autoware_launch/share/autoware_launch/rviz/autoware.rviz"

# For replaying rosbags from Data Search on autoware
# Set LAUNCH_LOG_FILE to empty string if logging is disabled (tee will just pass through)
if [ "$SAVE_LAUNCH_LOG" = "true" ]; then
    LAUNCH_LOG_FILE="$LOG_DIR/ros2_launch_$(date '+%Y%m%d_%H%M%S').log"
    mkdir -p "$LOG_DIR"
    touch "$LAUNCH_LOG_FILE"
    echo "ros2 launch log will be saved to $LAUNCH_LOG_FILE" | tee -a "$LAUNCH_LOG_FILE"
else
    LAUNCH_LOG_FILE=""
fi

# Check if paths exist
if [ ! -d "$MAP_PATH" ]; then
    echo "Error: MAP_PATH does not exist: $MAP_PATH"
    exit 1
fi

if [ ! -f "$ROSBAG" ]; then
    echo "Error: ROSBAG file does not exist: $ROSBAG"
    exit 1
fi

DATETIME=$(date '+%Y%m%d_%H%M%S')

echo "MAP_PATH: $MAP_PATH" | tee -a $LAUNCH_LOG_FILE
echo "ROSBAG: $ROSBAG" | tee -a $LAUNCH_LOG_FILE
echo "DATETIME: $DATETIME" | tee -a $LAUNCH_LOG_FILE
echo "POSE_SOURCE: $POSE_SOURCE" | tee -a $LAUNCH_LOG_FILE
if [ -n "$TOPIC_TYPE" ]; then
    echo "TOPIC_TYPE: $TOPIC_TYPE (rosbag recording enabled)" | tee -a $LAUNCH_LOG_FILE
else
    echo "TOPIC_TYPE: (not specified, rosbag recording disabled)" | tee -a $LAUNCH_LOG_FILE
fi

# Vehicle ID mapping table
# Format: "vehicle_id_fragment|vehicle_model|vehicle_id|sensor_model"
declare -a VEHICLE_CONFIGS=(
    # Add more vehicle mappings as needed
)


# Function to detect vehicle configuration from ROSBAG path
detect_vehicle_config() {
    local rosbag_path="$1"

    for config in "${VEHICLE_CONFIGS[@]}"; do
        IFS='|' read -r vehicle_fragment vehicle_model_val vehicle_id_val sensor_model_val <<< "$config"

        if [[ "$rosbag_path" == *"$vehicle_fragment"* ]]; then
            echo "$vehicle_model_val|$vehicle_id_val|$sensor_model_val"
            return 0
        fi
    done

    # No match found
    return 1
}

# Detect vehicle configuration from ROSBAG path
VEHICLE_CONFIG=$(detect_vehicle_config "$ROSBAG")
if [ $? -ne 0 ]; then
    echo "Error: Vehicle configuration not found for ROSBAG path: $ROSBAG"
    echo "Available vehicle ID fragments:"
    for config in "${VEHICLE_CONFIGS[@]}"; do
        IFS='|' read -r vehicle_fragment vehicle_model_val vehicle_id_val sensor_model_val <<< "$config"
        echo "  $vehicle_fragment -> $vehicle_model_val, $vehicle_id_val, $sensor_model_val"
    done
    echo "Please add the vehicle configuration to VEHICLE_CONFIGS array in this script."
    exit 1
fi

IFS='|' read -r VEHICLE_MODEL VEHICLE_ID SENSOR_MODEL <<< "$VEHICLE_CONFIG"

echo "Detected vehicle configuration:" | tee -a $LAUNCH_LOG_FILE
echo "  VEHICLE_MODEL: $VEHICLE_MODEL" | tee -a $LAUNCH_LOG_FILE
echo "  VEHICLE_ID: $VEHICLE_ID" | tee -a $LAUNCH_LOG_FILE
echo "  SENSOR_MODEL: $SENSOR_MODEL" | tee -a $LAUNCH_LOG_FILE

echo "Launch configuration:" | tee -a $LAUNCH_LOG_FILE
echo "  LAUNCH_DRIVER: $LAUNCH_DRIVER" | tee -a $LAUNCH_LOG_FILE
echo "  SENSING: $LAUNCH_SENSING" | tee -a $LAUNCH_LOG_FILE
echo "  PERCEPTION: $LAUNCH_PERCEPTION" | tee -a $LAUNCH_LOG_FILE
echo "  PLANNING: $LAUNCH_PLANNING" | tee -a $LAUNCH_LOG_FILE
echo "  CONTROL: $LAUNCH_CONTROL" | tee -a $LAUNCH_LOG_FILE
echo "  POSE_SOURCE: $POSE_SOURCE" | tee -a $LAUNCH_LOG_FILE
echo "  RVIZ: $RVIZ" | tee -a $LAUNCH_LOG_FILE
echo "  RVIZ_CONFIG: $RVIZ_CONFIG" | tee -a $LAUNCH_LOG_FILE
echo "  USE_SIM_TIME: $USE_SIM_TIME" | tee -a $LAUNCH_LOG_FILE

# Set traps to stop all background processes when the script exits
trap "$HOME/scripts_for_autoware/sh/kill_autoware.sh" EXIT INT TERM HUP

# Set OUTPUT_DIR to the ROSBAG directory with timestamp
ROSBAG_DIR=$(dirname "$ROSBAG")
OUTPUT_DIR=$ROSBAG_DIR/record_replay_$DATETIME
# OUTPUT_DIRはrecord_rosbag.shで作成されるため、ここでは作成しない（record_rosbag.shが既存ディレクトリを削除する）
# ただし、既に存在する場合は削除（前回の実行の残りなど）
if [ -d "$OUTPUT_DIR" ]; then
    echo "Warning: OUTPUT_DIR already exists, removing: $OUTPUT_DIR" | tee -a $LAUNCH_LOG_FILE
    rm -rf "$OUTPUT_DIR"
fi
echo "OUTPUT_DIR will be created by record script: $OUTPUT_DIR" | tee -a $LAUNCH_LOG_FILE

cd $(dirname $0)

# Filter out old workspace paths from AMENT_PREFIX_PATH to prevent plugin loading,
# but keep the current workspace ($CALL_DIR/install)
AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | awk -v keep="$CALL_DIR/install" '
  index($0, keep) == 1 || index($0, ENVIRON["HOME"]) == 0 { print }
' | tr '\n' ':' | sed 's/:$//')
export AMENT_PREFIX_PATH

# Source ROS 2 base environment first
source /opt/ros/humble/setup.bash
# Then source our workspace
source $CALL_DIR/install/setup.bash


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
    use_sim_time:=$USE_SIM_TIME \
    rviz:=$RVIZ \
    rviz_config:=$RVIZ_CONFIG \
    pose_source:=$POSE_SOURCE \
    | tee -a $LAUNCH_LOG_FILE &

# 立ち上がるまで待つ
# サービス呼び出しを立ち上がりの確認とする

echo "Calling trigger_node service..." | tee -a $LAUNCH_LOG_FILE
ros2 service call /localization/pose_twist_fusion_filter/trigger_node std_srvs/srv/SetBool "{data: false}" 2>&1 | tee -a $LAUNCH_LOG_FILE

# 安定性のため少し待つ
sleep 3

# 保存 - TOPIC_TYPEが指定されている場合のみrecord_rosbag.shを実行
if [ -n "$TOPIC_TYPE" ]; then
    echo "Starting rosbag recording (TOPIC_TYPE=$TOPIC_TYPE)..." | tee -a $LAUNCH_LOG_FILE
    ./record_rosbag.sh $OUTPUT_DIR $TOPIC_TYPE &
else
    echo "Rosbag recording disabled (TOPIC_TYPE not specified)" | tee -a $LAUNCH_LOG_FILE
fi

# 再生（バックグラウンドで開始）
echo "Starting rosbag playback..." | tee -a $LAUNCH_LOG_FILE
ros2 bag play ${ROSBAG} -r 1.0 --clock 200 2>&1 | tee -a $LAUNCH_LOG_FILE &
# ros2 bag play ${ROSBAG} -r 0.1 -s sqlite3 2>&1 | tee -a $LAUNCH_LOG_FILE &
ROSBAG_PID=$!

# rosbagからPointCloudデータが流れ始めるまで待つ
echo "Waiting for sensor data to start flowing (10 seconds)..." | tee -a $LAUNCH_LOG_FILE
sleep 10

# 初期位置を自動設定
# ROSBAGと同じディレクトリにあるinitial_pose.yamlを探す
INITIAL_POSE_YAML="${ROSBAG_DIR}/initial_pose.yaml"

echo "DEBUG: Checking for initial_pose.yaml at: $INITIAL_POSE_YAML" | tee -a $LAUNCH_LOG_FILE

if [ -f "$INITIAL_POSE_YAML" ]; then
    echo "Setting initial pose from $INITIAL_POSE_YAML..." | tee -a $LAUNCH_LOG_FILE
    python3 $(dirname $0)/set_initial_pose.py "$INITIAL_POSE_YAML" 2>&1 | tee -a $LAUNCH_LOG_FILE
    POSE_SET_RESULT=${PIPESTATUS[0]}
    if [ $POSE_SET_RESULT -eq 0 ]; then
        echo "Initial pose set successfully" | tee -a $LAUNCH_LOG_FILE
    else
        echo "Error: Failed to set initial pose (exit code: $POSE_SET_RESULT)" | tee -a $LAUNCH_LOG_FILE
    fi
    # 初期位置設定後、localizationが安定するまで待つ
    sleep 5
else
    echo "Info: initial_pose.yaml not found at $INITIAL_POSE_YAML" | tee -a $LAUNCH_LOG_FILE
    echo "Please set initial pose manually via RViz (2D Pose Estimate)" | tee -a $LAUNCH_LOG_FILE
fi

# rosbagプロセスが終了するまで待つ
echo "Waiting for rosbag playback to complete..." | tee -a $LAUNCH_LOG_FILE
wait $ROSBAG_PID
