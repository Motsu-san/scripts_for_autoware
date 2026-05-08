#!/bin/bash

# Usage: ./launch_autoware.sh <MAP_PATH> <ROSBAG_PATH>
# Example: ./launch_autoware.sh "$HOME/autoware_map/Komatsu/628-20250619100818540138" "$HOME/rosbag_replay/data1/final_merged/final_merged_0.db3"
#
# Requirements (必須・条件付きで参照するファイル。先頭で存在チェックする):
#   - 常時: $SCRIPT_DIR/vehicle_configs.sh
#   - 常時: $HOME/scripts_for_autoware/sh/kill_autoware.sh
#   - 常時: カレントディレクトリが autoware ビルド済みで install/setup.bash が存在すること（引数チェックで検証）
#   - TOPIC_TYPE 指定時: $SCRIPT_DIR/record_rosbag_localization_replay.sh
#   - --compare-bag 指定時: $HOME/scripts_for_autoware/py/play_multiple_rosbags.py
#   - 使用時に存在確認: scripts_for_autoware/py/set_initial_pose.py, gnss_to_initial_pose.py
#   - --record-rviz 指定時: $SCRIPT_DIR/capture_rviz_display.sh と xdotool（RViz ウィンドウ検出用）

CALL_DIR=$(pwd)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f "$SCRIPT_DIR/vehicle_configs.sh" ]; then
    echo "Error: Required file not found: $SCRIPT_DIR/vehicle_configs.sh" >&2
    exit 1
fi
source "$SCRIPT_DIR/vehicle_configs.sh"

# Usage: ./launch_autoware.sh <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE] [--compare-bag COMPARE_BAG] [--compare-topics TOPIC1 TOPIC2 ...] [-t START_TIME] [-T END_TIME]
# POSE_SOURCE_ID: 0=ndt (default), 1=ndt_lidar-marker, other numbers can be added in the future
# SAVE_LAUNCH_LOG: "true" to save ros2 launch log, anything else or omitted disables log saving
# TOPIC_TYPE: Topic type for record_rosbag.sh (default, lidar-marker_replay, full-sensing_replay, output, output_lidar-marker, convergence_evaluation, occlusion_adding)
#   If TOPIC_TYPE is omitted, rosbag recording will be disabled
# --compare-bag: Path to recorded rosbag for comparison (optional)
# --compare-topics: Topics to replay from recorded rosbag (optional, requires --compare-bag)
# --rate RATE: rosbag playback rate (default: 0.2). Passed to 'ros2 bag play -r RATE'.
# --record-rviz: Record RViz display (start after RViz window appears, stop when playback ends). Requires capture_rviz_display.sh and xdotool.
# -t TIME: Start playback from this time. UNIX time (e.g. 1772096549.105) or JST datetime (e.g. '2026-02-26 12:34:56').
# -T TIME, --end-time TIME: End playback at this absolute time. UNIX/JST 指定可。内部的には --duration に変換。
#
# For unified_localization (NDT+EKF in one node), use launch_unified_localization.sh instead.
#
# sample-rosbag 再生時は use_sim_time=false かつ --clock なしで再生し、RViz チラつきを防ぐ（自動）。
# それ以外で use_sim_time を off にする場合: USE_SIM_TIME=false ./launch_autoware.sh <MAP_PATH> <ROSBAG_PATH> ...
#
# 起動モジュールの絞り込み（デフォルトは従来どおり）。localization + RViz 中心にしたい場合の例:
#   LAUNCH_PERCEPTION=false LAUNCH_PLANNING=false LAUNCH_CONTROL=false はスクリプト内で既に false。
#   さらに例: LAUNCH_API=false（AD API 停止） LAUNCH_SENSING=false（bag に /sensing/lidar/... が含まれる場合など）
#   LAUNCH_SENSING_DRIVER: LiDAR の「実機(HW)ドライバ」を起動するか（未指定時は false）。
#     rosbag の PandarScan 再生: false（nebula launch_hw=false で pandar_packets を点群化）。true にすると実センサ接続となり bag は無視される。
#     点群が既に bag にある場合も false 推奨（不要な nebula 負荷を避ける）。
#   vehicle / system / map も false にできるが、map=false は NDT 向けに通常不可。system/vehicle off は診断や車両情報で不具合の可能性あり。
#   環境変数（未設定時は true）: LAUNCH_VEHICLE, LAUNCH_SYSTEM, LAUNCH_MAP, LAUNCH_SENSING, LAUNCH_SENSING_DRIVER, LAUNCH_API, LAUNCH_LOCALIZATION, LAUNCH_RVIZ
#   URDF（vls_description 等）未ビルドで xacro 失敗: colcon build --packages-up-to aip_xx1_description または LAUNCH_VEHICLE=false
#   system 用（duplicated_node_checker 等）未ビルド: 下の colcon 一括 または LAUNCH_SYSTEM=false
#   aip_xx1 + sensing で pe_ars408_ros 未ビルド: colcon build --packages-select pe_ars408_ros または LAUNCH_SENSING=false
#   localization で autoware_stop_filter 未ビルド: colcon build --packages-select autoware_stop_filter（無いと RViz も出ない）
#
# 位置引数の解析
POSITIONAL_ARGS=()
COMPARE_BAG=""
COMPARE_TOPICS=()
START_UNIX_TIME=""
END_UNIX_TIME=""
PLAYBACK_RATE=""
RECORD_RVIZ="false"

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
        --rate)
            PLAYBACK_RATE="$2"
            shift 2
            ;;
        --record-rviz)
            RECORD_RVIZ="true"
            shift
            ;;
        -t)
            START_UNIX_TIME="$2"
            shift 2
            ;;
        -T|--end-time)
            END_UNIX_TIME="$2"
            shift 2
            ;;
        *)
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done

# 位置引数を設定
set -- "${POSITIONAL_ARGS[@]}"

if [ $# -lt 2 ] || [ $# -gt 5 ] || [ ! -f "$CALL_DIR/install/setup.bash" ]; then
    echo "Usage: $0 <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE] [--compare-bag COMPARE_BAG] [--compare-topics TOPIC1 TOPIC2 ...] [-t START_TIME] [-T END_TIME]"
    echo "Must provide <MAP_PATH> <ROSBAG_PATH> and call from the directory where autoware is located and built(ex. $HOME/autoware)."
    echo "POSE_SOURCE_ID: 0=ndt (default), 1=ndt_lidar-marker"
    echo "SAVE_LAUNCH_LOG: 'true' to save ros2 launch log, anything else or omitted disables log saving"
    echo "TOPIC_TYPE: default, lidar-marker_replay, full-sensing_replay, output, output_lidar-marker, convergence_evaluation, occlusion_adding"
    echo "  If TOPIC_TYPE is omitted, rosbag recording will be disabled"
    echo "--compare-bag: Path to recorded rosbag for comparison (optional)"
    echo "--compare-topics: Topics to replay from recorded rosbag (optional, requires --compare-bag)"
    echo "--rate RATE: Playback rate for ros2 bag play (default: 0.2)"
    echo "--record-rviz: Record RViz display (starts when RViz window appears, stops when playback ends)"
    echo "-t TIME: Start playback from this time. UNIX time (e.g. 1772096549.105) or JST datetime (e.g. '2026-02-26 12:34:56')"
    echo "-T, --end-time TIME: End playback at this absolute time. UNIX/JST accepted"
    echo "Example: $0 \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 1 true output"
    echo "Example with -t (UNIX): $0 \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 0 false '' -t 1772096549.105"
    echo "Example with -t (JST):  $0 \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 0 false '' -t '2026-02-26 12:34:56'"
    echo "Example with comparison: $0 \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 1 true output --compare-bag \"$HOME/rosbag_replay/recorded.bag\" --compare-topics /localization/kinematic_state"
    exit 1
fi

MAP_PATH="$1"
ROSBAG="$2"
POSE_SOURCE_ID="${3:-0}"
SAVE_LAUNCH_LOG="${4:-false}"
TOPIC_TYPE="${5:-}"
PLAYBACK_RATE="${PLAYBACK_RATE:-0.2}"

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

# 比較用rosbagの検証
if [ -n "$COMPARE_BAG" ] && [ ${#COMPARE_TOPICS[@]} -eq 0 ]; then
    echo "Error: --compare-topics must be specified when using --compare-bag" >&2
    exit 1
fi

if [ ${#COMPARE_TOPICS[@]} -gt 0 ] && [ -z "$COMPARE_BAG" ]; then
    echo "Error: --compare-bag must be specified when using --compare-topics" >&2
    exit 1
fi

LOG_DIR=$HOME/log

# launch_sensing_driver → tier4_sensing → nebula の launch_hw。
# nebula: launch_hw=true は実 LiDAR（UDP）接続。bag の PandarScan は無視される。
#        launch_hw=false のとき pandar_packets を購読し PandarScan リプレイを点群化する（オフライン再生向け）。
# よろしく rosbag 再生では未指定＝ false（従来 logging_simulator 既定と同じ）。実車同セグメント上で生データを使う場合のみ true。
LAUNCH_SENSING="${LAUNCH_SENSING:-true}"
LAUNCH_DRIVER="${LAUNCH_DRIVER:-false}"
LAUNCH_LOCALIZATION="${LAUNCH_LOCALIZATION:-true}"
LAUNCH_API="${LAUNCH_API:-true}"
LAUNCH_DEFAULT_AD_API="${LAUNCH_DEFAULT_AD_API:-true}"
LAUNCH_RVIZ_ADAPTORS="${LAUNCH_RVIZ_ADAPTORS:-true}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
LAUNCH_PERCEPTION="false"
LAUNCH_PLANNING="false"
LAUNCH_CONTROL="false"

# Parse JST date/time string to UNIX time (seconds, decimal OK).
# Accepts: "2026-02-26 12:34:56", "2026-02-26 12:34:56.123", "2026/02/26 12:34:56"
# Output: UNIX seconds (decimal), or empty on failure.
parse_jst_to_unix_sec() {
    local s="$1"
    local frac=""
    if [[ "$s" =~ \.[0-9]+$ ]]; then
        frac="${s##*.}"
        s="${s%.*}"
    fi
    # Normalize separators: replace / with -
    s="${s//\//-}"
    local unix_sec
    unix_sec=$(TZ=Asia/Tokyo date -d "$s" +%s 2>/dev/null) || return 1
    [[ -z "$unix_sec" ]] && return 1
    if [[ -n "$frac" ]]; then
        echo "${unix_sec}.${frac}"
    else
        echo "${unix_sec}"
    fi
}

# Get bag start time in UNIX seconds (for -t option). Uses "ros2 bag info" (must be run after sourcing ROS).
# Output: start time in seconds (decimal), or empty on failure.
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

# Return 0 if bag contains /clock topic, non-zero otherwise.
# Must be called after sourcing ROS env (ros2 command is required).
bag_has_clock_topic() {
    local bag_path="$1"
    ros2 bag info "$bag_path" 2>/dev/null | awk '
        /Topic:[[:space:]]*\/clock[[:space:]]*\|/ { found=1 }
        END { exit(found ? 0 : 1) }
    '
}

# Return 0 if bag contains /tf_static topic, non-zero otherwise.
# Must be called after sourcing ROS env (ros2 command is required).
bag_has_tf_static_topic() {
    local bag_path="$1"
    ros2 bag info "$bag_path" 2>/dev/null | awk '
        /Topic:[[:space:]]*\/tf_static[[:space:]]*\|/ { found=1 }
        END { exit(found ? 0 : 1) }
    '
}

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
# use_sim_time: 未指定時は true。sample-rosbag 再生時はチラつき防止のため false とし、--clock なしで再生する。
# ただし USE_SIM_TIME を明示的に指定した場合はその値を優先する。
_USE_SIM_TIME_EXPLICITLY_SET="${USE_SIM_TIME+yes}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
if [[ "$ROSBAG" == *"sample-rosbag"* ]] && [ -z "$COMPARE_BAG" ] && [ -z "$_USE_SIM_TIME_EXPLICITLY_SET" ]; then
  USE_SIM_TIME="false"
fi
unset _USE_SIM_TIME_EXPLICITLY_SET
RVIZ="$LAUNCH_RVIZ"
# RViz: 未設定のときは localization_standalone の既定
# ($(find-pkg-share tier4_localization_launch)/rviz/autoware.rviz)。上書きする場合のみ
# export RVIZ_CONFIG=/path/to/config.rviz
RVIZ_CONFIG="${RVIZ_CONFIG-}"

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

if [ ! -f "$ROSBAG" ] && [ ! -d "$ROSBAG" ]; then
    echo "Error: ROSBAG path does not exist: $ROSBAG"
    exit 1
fi

# -t 指定時: 値の解釈のみここで行う。bag 先頭時刻の取得とオフセット計算は source 後に実施（ros2 bag info を使うため）
START_OFFSET_SEC=""
PLAY_OFFSET_ARGS=()
PLAY_DURATION_SEC=""
END_OFFSET_SEC=""
if [ -n "$START_UNIX_TIME" ]; then
    if [[ "$START_UNIX_TIME" =~ ^[0-9]+\.?[0-9]*$ ]]; then
        : # そのまま UNIX 時刻として使用
    else
        # JST 日時文字列として解釈して UNIX 時刻に変換
        START_UNIX_TIME=$(parse_jst_to_unix_sec "$START_UNIX_TIME")
        if [ -z "$START_UNIX_TIME" ]; then
            echo "Error: -t could not parse as JST datetime (e.g. '2026-02-26 12:34:56' or '2026-02-26 12:34:56.123')" >&2
            exit 1
        fi
        echo "Parsed -t as JST -> UNIX time: $START_UNIX_TIME"
    fi
fi
if [ -n "$END_UNIX_TIME" ]; then
    if [[ "$END_UNIX_TIME" =~ ^[0-9]+\.?[0-9]*$ ]]; then
        : # そのまま UNIX 時刻として使用
    else
        # JST 日時文字列として解釈して UNIX 時刻に変換
        END_UNIX_TIME=$(parse_jst_to_unix_sec "$END_UNIX_TIME")
        if [ -z "$END_UNIX_TIME" ]; then
            echo "Error: -T/--end-time could not parse as JST datetime (e.g. '2026-02-26 12:34:56' or '2026-02-26 12:34:56.123')" >&2
            exit 1
        fi
        echo "Parsed -T/--end-time as JST -> UNIX time: $END_UNIX_TIME"
    fi
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

if [ -n "$COMPARE_BAG" ]; then
    echo "COMPARE_BAG: $COMPARE_BAG" | tee -a $LAUNCH_LOG_FILE
    echo "COMPARE_TOPICS: ${COMPARE_TOPICS[*]}" | tee -a $LAUNCH_LOG_FILE
fi
if [ -n "$START_UNIX_TIME" ]; then
    echo "START_UNIX_TIME (-t): $START_UNIX_TIME" | tee -a $LAUNCH_LOG_FILE
    echo "START_OFFSET_SEC: $START_OFFSET_SEC" | tee -a $LAUNCH_LOG_FILE
fi
if [ -n "$END_UNIX_TIME" ]; then
    echo "END_UNIX_TIME (-T/--end-time): $END_UNIX_TIME" | tee -a $LAUNCH_LOG_FILE
fi
echo "PLAYBACK_RATE (--rate): $PLAYBACK_RATE" | tee -a $LAUNCH_LOG_FILE
echo "RECORD_RVIZ (--record-rviz): $RECORD_RVIZ" | tee -a $LAUNCH_LOG_FILE

# Detect vehicle configuration from ROSBAG path (VEHICLE_CONFIGS / detect_vehicle_config は vehicle_configs.sh で定義)
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
export VEHICLE_ID

echo "Detected vehicle configuration:" | tee -a $LAUNCH_LOG_FILE
echo "  VEHICLE_MODEL: $VEHICLE_MODEL" | tee -a $LAUNCH_LOG_FILE
echo "  VEHICLE_ID: $VEHICLE_ID" | tee -a $LAUNCH_LOG_FILE
echo "  SENSOR_MODEL: $SENSOR_MODEL" | tee -a $LAUNCH_LOG_FILE

echo "Launch configuration:" | tee -a $LAUNCH_LOG_FILE
echo "  SENSING: $LAUNCH_SENSING" | tee -a $LAUNCH_LOG_FILE
echo "  DRIVER: $LAUNCH_DRIVER" | tee -a $LAUNCH_LOG_FILE
echo "  LOCALIZATION: $LAUNCH_LOCALIZATION" | tee -a $LAUNCH_LOG_FILE
echo "  API: $LAUNCH_API" | tee -a $LAUNCH_LOG_FILE
echo "  DEFAULT_AD_API: $LAUNCH_DEFAULT_AD_API" | tee -a $LAUNCH_LOG_FILE
echo "  RVIZ_ADAPTORS: $LAUNCH_RVIZ_ADAPTORS" | tee -a $LAUNCH_LOG_FILE
echo "  PERCEPTION: $LAUNCH_PERCEPTION" | tee -a $LAUNCH_LOG_FILE
echo "  PLANNING: $LAUNCH_PLANNING" | tee -a $LAUNCH_LOG_FILE
echo "  CONTROL: $LAUNCH_CONTROL" | tee -a $LAUNCH_LOG_FILE
echo "  POSE_SOURCE: $POSE_SOURCE" | tee -a $LAUNCH_LOG_FILE
echo "  RVIZ: $RVIZ" | tee -a $LAUNCH_LOG_FILE
if [ -n "$RVIZ_CONFIG" ]; then
    echo "  RVIZ_CONFIG: $RVIZ_CONFIG" | tee -a $LAUNCH_LOG_FILE
else
    echo "  RVIZ_CONFIG: (launch default: tier4_localization_launch/rviz/autoware.rviz)" | tee -a $LAUNCH_LOG_FILE
fi
echo "  USE_SIM_TIME: $USE_SIM_TIME" | tee -a $LAUNCH_LOG_FILE

# Cleanup on exit (Ctrl+C etc.): stop RViz capture then kill autoware
RVIZ_CAPTURE_PID=""
TF_STATIC_PLAYER_PID=""
END_TIME_MONITOR_PID=""
cleanup_on_exit() {
    if [ -n "${END_TIME_MONITOR_PID:-}" ] && kill -0 "$END_TIME_MONITOR_PID" 2>/dev/null; then
        kill -INT "$END_TIME_MONITOR_PID" 2>/dev/null || true
        wait "$END_TIME_MONITOR_PID" 2>/dev/null || true
    fi
    if [ -n "${TF_STATIC_PLAYER_PID:-}" ] && kill -0 "$TF_STATIC_PLAYER_PID" 2>/dev/null; then
        kill -INT "$TF_STATIC_PLAYER_PID" 2>/dev/null || true
        wait "$TF_STATIC_PLAYER_PID" 2>/dev/null || true
    fi
    if [ -n "${RVIZ_CAPTURE_PID:-}" ] && kill -0 "$RVIZ_CAPTURE_PID" 2>/dev/null; then
        kill -INT "$RVIZ_CAPTURE_PID" 2>/dev/null || true
        wait "$RVIZ_CAPTURE_PID" 2>/dev/null || true
    fi
    "$HOME/scripts_for_autoware/sh/kill_autoware.sh"
}
trap cleanup_on_exit EXIT INT TERM HUP

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
# 同一パッケージが UNDERLAY（例: ~/autoware/install）にもあると、ament_index の探索順でそちらが先に当たり
# 「localization_standalone.launch.xml が無い」旧 share が選ばれる。呼び出し元 $CALL_DIR を必ず先頭に置く。
_CALL_INSTALL="$CALL_DIR/install"
if [ -d "$_CALL_INSTALL" ]; then
    export AMENT_PREFIX_PATH="$_CALL_INSTALL${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
fi
unset _CALL_INSTALL
# tier4_localization_launch は merge-install 直下だけでは別 prefix の UNDERLAY より後ろに並ぶことがある。
# include 内の $(find-pkg-share tier4_localization_launch) が autoware 版を掴むと必須引数が増えて即死するため、
# 当該パッケージ prefix は $CALL_DIR 側のみを先頭に残す。
_T4_LOC_PREFIX="$CALL_DIR/install/tier4_localization_launch"
if [ -d "$_T4_LOC_PREFIX/share/tier4_localization_launch" ]; then
    _t4_new=""
    _t4_ifs="$IFS"
    IFS=':'
    for _t4_p in $AMENT_PREFIX_PATH; do
        [ -z "$_t4_p" ] && continue
        case "$_t4_p" in
            */tier4_localization_launch) continue ;;
        esac
        _t4_new="${_t4_new:+${_t4_new}:}${_t4_p}"
    done
    IFS="$_t4_ifs"
    export AMENT_PREFIX_PATH="${_T4_LOC_PREFIX}${_t4_new:+:${_t4_new}}"
fi
unset _T4_LOC_PREFIX
# tier4_map_launch も UNDERLAY 版だと autoware_map_projection_loader 等が混ざり MapProjectorInfo の ABI 不一致で map が即死する
_T4_MAP_PREFIX="$CALL_DIR/install/tier4_map_launch"
if [ -d "$_T4_MAP_PREFIX/share/tier4_map_launch" ]; then
    _t4m_new=""
    _t4m_ifs="$IFS"
    IFS=':'
    for _t4m_p in $AMENT_PREFIX_PATH; do
        [ -z "$_t4m_p" ] && continue
        case "$_t4m_p" in
            */tier4_map_launch) continue ;;
        esac
        _t4m_new="${_t4m_new:+${_t4m_new}:}${_t4m_p}"
    done
    IFS="$_t4m_ifs"
    export AMENT_PREFIX_PATH="${_T4_MAP_PREFIX}${_t4m_new:+:${_t4m_new}}"
fi
unset _T4_MAP_PREFIX

# aip_xx1 + vehicle 起動時: tier4_vehicle の xacro が各 *_description を要求（無いと launch 即死）
if [ "${LAUNCH_VEHICLE:-true}" = "true" ] && [ "$SENSOR_MODEL" = "aip_xx1" ]; then
    _need=(velodyne_description vls_description livox_description camera_description imu_description aip_xx1_description)
    if [ "$VEHICLE_MODEL" = "jpntaxi" ]; then
        _need+=(jpntaxi_description)
    fi
    _missing=()
    for _p in "${_need[@]}"; do
        if ! ros2 pkg prefix "$_p" &>/dev/null; then
            _missing+=("$_p")
        fi
    done
    if [ ${#_missing[@]} -gt 0 ]; then
        echo "Error: vehicle URDF 用パッケージが未インストール: ${_missing[*]}" | tee -a $LAUNCH_LOG_FILE >&2
        echo "  対処1) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-up-to aip_xx1_description" | tee -a $LAUNCH_LOG_FILE >&2
        echo "  対処2) LAUNCH_VEHICLE=false $0 <MAP> <BAG> ...  （TF は bag 頼み）" | tee -a $LAUNCH_LOG_FILE >&2
        exit 1
    fi
fi

# system 起動時: tier4_system.system.launch.xml が多数のパッケージを find-pkg（未ビルドだと連鎖失敗）
# if [ "${LAUNCH_SYSTEM:-true}" = "true" ]; then
#     _sysneed=(diagnostic_graph_aggregator duplicated_node_checker autoware_processing_time_checker component_interface_tools component_state_monitor)
#     _miss=()
#     for _p in "${_sysneed[@]}"; do
#         if ! ros2 pkg prefix "$_p" &>/dev/null; then
#             _miss+=("$_p")
#         fi
#     done
#     if [ ${#_miss[@]} -gt 0 ]; then
#         echo "Error: system 用パッケージが未インストール: ${_miss[*]}" | tee -a $LAUNCH_LOG_FILE >&2
#         echo "  対処) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-select duplicated_node_checker autoware_processing_time_checker component_interface_tools diagnostic_graph_aggregator" | tee -a $LAUNCH_LOG_FILE >&2
#         echo "  緩和) LAUNCH_SYSTEM=false $0 ..." | tee -a $LAUNCH_LOG_FILE >&2
#         exit 1
#     fi
# fi

# localization 起動時: pose_twist_fusion_filter.launch.xml が下記を include（未ビルドだと launch 例外→ RViz も起動しない）
# pilot-auto.x2 等では autoware プレフィックス無しのパッケージ名でインストールされる。いずれかが解決すればよい。
if [ "${LAUNCH_LOCALIZATION:-true}" = "true" ]; then
    _loc_pkg_groups=(
        "autoware_ekf_localizer ekf_localizer"
        "autoware_stop_filter stop_filter"
        "autoware_twist2accel twist2accel"
        "autoware_pose_instability_detector pose_instability_detector"
        "autoware_localization_error_monitor localization_error_monitor"
    )
    _loc_miss=()
    for _g in "${_loc_pkg_groups[@]}"; do
        _loc_ok=0
        for _p in $_g; do
            if ros2 pkg prefix "$_p" &>/dev/null; then
                _loc_ok=1
                break
            fi
        done
        if [ "$_loc_ok" -eq 0 ]; then
            _loc_miss+=("($_g)")
        fi
    done
    unset _loc_ok
    if [ ${#_loc_miss[@]} -ne 0 ]; then
        echo "Error: ローカリゼーション用パッケージが未インストール（いずれかの名前で存在すること）: ${_loc_miss[*]}" | tee -a $LAUNCH_LOG_FILE >&2
        echo "  対処) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-up-to tier4_localization_launch" | tee -a $LAUNCH_LOG_FILE >&2
        echo "  または Autoware underlay を source した上で再実行: source /path/to/autoware/install/setup.bash" | tee -a $LAUNCH_LOG_FILE >&2
        exit 1
    fi
fi

# aip_xx1 + sensing: common_sensor_launch/ars408.launch.xml が pe_ars408_ros を無条件 include（未ビルドだと find-pkg 即死）
if [ "${LAUNCH_SENSING:-true}" = "true" ] && [ "$SENSOR_MODEL" = "aip_xx1" ]; then
    if ! ros2 pkg prefix pe_ars408_ros &>/dev/null; then
        echo "Error: pe_ars408_ros（Continental ARS408）が未インストール。aip の radar launch に必須。" | tee -a $LAUNCH_LOG_FILE >&2
        echo "  対処) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-select pe_ars408_ros" | tee -a $LAUNCH_LOG_FILE >&2
        echo "  緩和) LAUNCH_SENSING=false $0 ...  （bag に点群等があればローカライゼーションは進む場合あり）" | tee -a $LAUNCH_LOG_FILE >&2
        exit 1
    fi
fi

# API 起動時: default_ad_api を含む API スタックが必要（automatic_pose_initializer が /api/localization/* を使用）
if [ "${LAUNCH_API:-true}" = "true" ]; then
    _api_need=(default_ad_api)
    if [ "${LAUNCH_RVIZ_ADAPTORS:-true}" = "true" ]; then
        _api_need+=(ad_api_adaptors)
    fi
    _api_miss=()
    for _p in "${_api_need[@]}"; do
        if ! ros2 pkg prefix "$_p" &>/dev/null; then
            _api_miss+=("$_p")
        fi
    done
    if [ ${#_api_miss[@]} -gt 0 ]; then
        echo "Error: API 用パッケージが未インストール: ${_api_miss[*]}" | tee -a $LAUNCH_LOG_FILE >&2
        echo "  対処) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-up-to default_ad_api ad_api_adaptors" | tee -a $LAUNCH_LOG_FILE >&2
        exit 1
    fi
fi

# -t 指定時: ros2 bag info で bag 先頭時刻を取得し、オフセットを計算（source 後に実行）
if [ -n "$START_UNIX_TIME" ] || [ -n "$END_UNIX_TIME" ]; then
    BAG_START_SEC=$(get_bag_start_unix_sec "$ROSBAG")
    if [ -z "$BAG_START_SEC" ]; then
        echo "Error: Could not get bag start time for -t/-T option (run 'ros2 bag info $ROSBAG' to check)" >&2
        exit 1
    fi
fi

if [ -n "$START_UNIX_TIME" ]; then
    START_OFFSET_SEC=$(echo "$START_UNIX_TIME $BAG_START_SEC" | awk '{s=$1-$2; if(s<0)s=0; printf "%.3f", s}')
    PLAY_OFFSET_ARGS=(--start-offset "$START_OFFSET_SEC")
    echo "Start from UNIX time -t $START_UNIX_TIME (bag start: $BAG_START_SEC) -> --start-offset ${START_OFFSET_SEC}s"
fi

if [ -n "$END_UNIX_TIME" ]; then
    END_OFFSET_SEC=$(echo "$END_UNIX_TIME $BAG_START_SEC" | awk '{e=$1-$2; if(e<0)e=0; printf "%.3f", e}')
    if [ -n "$START_OFFSET_SEC" ]; then
        PLAY_DURATION_SEC=$(echo "$END_OFFSET_SEC $START_OFFSET_SEC" | awk '{d=$1-$2; printf "%.3f", d}')
    else
        PLAY_DURATION_SEC="$END_OFFSET_SEC"
    fi
    if ! awk "BEGIN{exit !($PLAY_DURATION_SEC > 0)}"; then
        echo "Error: -T/--end-time must be later than effective start time (computed duration=${PLAY_DURATION_SEC}s)." >&2
        exit 1
    fi
    echo "End at UNIX time -T $END_UNIX_TIME (bag start: $BAG_START_SEC) -> stop on /clock monitor"
fi

# ros2 launch autoware_launch logging_simulator.launch.xml \
    # map_path:=$MAP_PATH \
    # vehicle_model:=$VEHICLE_MODEL \
    # vehicle_id:=$VEHICLE_ID \
    # sensor_model:=$SENSOR_MODEL \
    # launch_driver:=$LAUNCH_DRIVER \
    # sensing:=$LAUNCH_SENSING \
    # localization:=$LAUNCH_LOCALIZATION \
    # perception:=$LAUNCH_PERCEPTION \
    # planning:=$LAUNCH_PLANNING \
    # control:=$LAUNCH_CONTROL \
    # use_sim_time:=$USE_SIM_TIME \
    # rviz:=$RVIZ \
    # rviz_config:=$RVIZ_CONFIG \
    # pose_source:=$POSE_SOURCE \
    # | tee -a $LAUNCH_LOG_FILE &
# tier4_localization_launch の localization_standalone（引数は同 launch の <arg> と一致させる）
# UNDERLAY に同名パッケージがあると ros2 が旧 share を選ぶため、$CALL_DIR 側にファイルがあれば絶対パスで起動する
_LOCALIZATION_STANDALONE_XML="$CALL_DIR/install/tier4_localization_launch/share/tier4_localization_launch/launch/localization_standalone.launch.xml"
if [ -f "$_LOCALIZATION_STANDALONE_XML" ]; then
    _LOC_LAUNCH=( "$_LOCALIZATION_STANDALONE_XML" )
    echo "Using localization standalone launch (absolute path): $_LOCALIZATION_STANDALONE_XML" | tee -a $LAUNCH_LOG_FILE
else
    _LOC_LAUNCH=( tier4_localization_launch localization_standalone.launch.xml )
    echo "Using localization standalone launch (package): tier4_localization_launch localization_standalone.launch.xml" | tee -a $LAUNCH_LOG_FILE
fi
# stderr も tee へ（パイプは stdout のみのため 2>&1 が必須。無いと launch 失敗がログに残らず trigger が永久待ちに見える）
_LOC_STANDALONE_ARGS=(
    preset:=logging
    map_path:=$MAP_PATH
    vehicle_model:=$VEHICLE_MODEL
    sensor_model:=$SENSOR_MODEL
    launch_sensing:=$LAUNCH_SENSING
    launch_api:=$LAUNCH_API
    launch_default_ad_api:=$LAUNCH_DEFAULT_AD_API
    launch_rviz_adaptors:=$LAUNCH_RVIZ_ADAPTORS
    launch_sensing_driver:=$LAUNCH_DRIVER
    use_sim_time:=$USE_SIM_TIME
    rviz:=$RVIZ
    pose_source:=$POSE_SOURCE
)
[ -n "$RVIZ_CONFIG" ] && _LOC_STANDALONE_ARGS+=( "rviz_config:=$RVIZ_CONFIG" )
ros2 launch "${_LOC_LAUNCH[@]}" "${_LOC_STANDALONE_ARGS[@]}" \
    2>&1 | tee -a $LAUNCH_LOG_FILE &
unset _LOC_STANDALONE_ARGS
unset _LOC_LAUNCH
unset _LOCALIZATION_STANDALONE_XML

# バックグラウンド起動直後に trigger するとノード未起動で待ち続けることがあるため短い猶予
sleep 3

# 立ち上がるまで待つ（trigger_node サービス呼び出しで起動確認）
echo "Calling trigger_node service..." | tee -a $LAUNCH_LOG_FILE
ros2 service call /localization/pose_twist_fusion_filter/trigger_node std_srvs/srv/SetBool "{data: false}" 2>&1 | tee -a $LAUNCH_LOG_FILE

# 安定性のため少し待つ
sleep 3

# RViz 録画: ウィンドウが開いてから録画開始、再生終了時に録画終了
RVIZ_CAPTURE_PID=""
if [ "$RECORD_RVIZ" = "true" ]; then
    RVIZ_CAPTURE_SCRIPT="${RVIZ_CAPTURE_SCRIPT:-$SCRIPT_DIR/capture_rviz_display.sh}"
    if [ ! -f "$RVIZ_CAPTURE_SCRIPT" ]; then
        echo "Warning: --record-rviz requested but $RVIZ_CAPTURE_SCRIPT not found, skipping RViz capture." | tee -a $LAUNCH_LOG_FILE
    elif [ -z "${DISPLAY:-}" ]; then
        echo "Warning: --record-rviz requested but DISPLAY is not set, skipping RViz capture." | tee -a $LAUNCH_LOG_FILE
    else
        echo "Waiting for RViz window to appear (timeout 90s)..." | tee -a $LAUNCH_LOG_FILE
        RVIZ_WAIT_SEC=0
        while [ $RVIZ_WAIT_SEC -lt 90 ]; do
            RVIZ_WID=$(xdotool search --name "RViz" 2>/dev/null | head -1)
            if [ -n "$RVIZ_WID" ]; then
                echo "RViz window found, starting RViz capture (output: $ROSBAG_DIR)..." | tee -a $LAUNCH_LOG_FILE
                "$RVIZ_CAPTURE_SCRIPT" "$ROSBAG_DIR" &
                RVIZ_CAPTURE_PID=$!
                sleep 1
                break
            fi
            sleep 1
            RVIZ_WAIT_SEC=$((RVIZ_WAIT_SEC + 1))
        done
        if [ -z "$RVIZ_CAPTURE_PID" ]; then
            echo "Warning: RViz window did not appear within 90s, skipping RViz capture." | tee -a $LAUNCH_LOG_FILE
        fi
    fi
fi

# 保存 - TOPIC_TYPEが指定されている場合のみrecord_rosbag_localization_replay.shを実行
if [ -n "$TOPIC_TYPE" ]; then
    echo "Starting rosbag recording (TOPIC_TYPE=$TOPIC_TYPE)..." | tee -a $LAUNCH_LOG_FILE
    ./record_rosbag_localization_replay.sh $OUTPUT_DIR $TOPIC_TYPE &
else
    echo "Rosbag recording disabled (TOPIC_TYPE not specified)" | tee -a $LAUNCH_LOG_FILE
fi

# 再生（バックグラウンドで開始）
echo "Starting rosbag playback..." | tee -a $LAUNCH_LOG_FILE

# /clock の扱い:
# - bag に /clock が含まれる場合: bag 側 clock を使う（--clock を付けない）
# - bag に /clock が含まれない場合: player で /clock を生成（--clock <hz>）
PLAY_CLOCK_HZ="${PLAY_CLOCK_HZ:-200}"
CLOCK_ARGS=()
if bag_has_clock_topic "$ROSBAG"; then
    echo "Detected /clock in rosbag: play without --clock (use bag clock)" | tee -a $LAUNCH_LOG_FILE
else
    CLOCK_ARGS=(--clock "$PLAY_CLOCK_HZ")
    echo "No /clock in rosbag: play with --clock $PLAY_CLOCK_HZ" | tee -a $LAUNCH_LOG_FILE
fi

# 再生コマンドをバックグラウンド実行し、必要なら tee でログ保存
# 注意: cmd | tee & だと $! は tee 側になり、-T 終了や wait が本体に効かない。
# > >(tee -a ...) 2>&1 & なら $! は再生本体（ros2 bag play 等）になる。
# バックグラウンド & は非対話だと子の stdin が /dev/null になりがちなので、スペース一時停止用に </dev/tty を明示。
run_with_playback_log() {
    if [ -n "$LAUNCH_LOG_FILE" ]; then
        if [ -r /dev/tty ]; then
            "$@" </dev/tty > >(tee -a "$LAUNCH_LOG_FILE") 2>&1 &
        else
            "$@" > >(tee -a "$LAUNCH_LOG_FILE") 2>&1 &
        fi
    else
        if [ -r /dev/tty ]; then
            "$@" </dev/tty &
        else
            "$@" &
        fi
    fi
    ROSBAG_PID=$!
}

run_with_playback_log_nostdin() {
    if [ -n "$LAUNCH_LOG_FILE" ]; then
        "$@" < /dev/null > >(tee -a "$LAUNCH_LOG_FILE") 2>&1 &
    else
        "$@" < /dev/null &
    fi
    ROSBAG_PID=$!
}

# 途中開始時の /tf_static 補助:
# - START_OFFSET_SEC > 0 のときのみ、/tf_static 専用プレイヤーを先行起動して loop させる
# - これにより、途中開始で先頭付近の /tf_static を飛ばしても static TF を継続供給できる
TF_STATIC_LOOP_ON_OFFSET="${TF_STATIC_LOOP_ON_OFFSET:-true}"
if [ "$TF_STATIC_LOOP_ON_OFFSET" = "true" ] && [ -n "$START_OFFSET_SEC" ] && awk "BEGIN{exit !($START_OFFSET_SEC > 0)}"; then
    if bag_has_tf_static_topic "$ROSBAG"; then
        echo "Start /tf_static helper player (loop) for offset replay: --topics /tf_static --loop" | tee -a $LAUNCH_LOG_FILE
        run_with_playback_log_nostdin ros2 bag play "${ROSBAG}" --topics /tf_static --loop
        TF_STATIC_PLAYER_PID=$ROSBAG_PID
        sleep 1
    else
        echo "No /tf_static in rosbag: skip /tf_static helper player" | tee -a $LAUNCH_LOG_FILE
    fi
fi

# /clock 監視で終端時刻に到達したら再生を停止する
start_end_time_monitor() {
    local target_unix="$1"
    local target_pid="$2"
    (
        ros2 topic echo /clock --field clock 2>/dev/null | awk -v end_t="$target_unix" '
            /^sec:/ { sec=$2 }
            /^nanosec:/ {
                nsec=$2
                now=sec + (nsec / 1000000000.0)
                if (now >= end_t) {
                    exit 0
                }
            }
        '
        if [ $? -eq 0 ] && kill -0 "$target_pid" 2>/dev/null; then
            kill -INT "$target_pid" 2>/dev/null || true
        fi
    ) &
    END_TIME_MONITOR_PID=$!
}

# 比較用rosbagが指定されている場合はplay_multiple_rosbags.pyを使用
if [ -n "$COMPARE_BAG" ]; then
    # play_multiple_rosbags.pyのパスを確認
    PLAY_MULTIPLE_SCRIPT="$HOME/scripts_for_autoware/py/play_multiple_rosbags.py"
    if [ ! -f "$PLAY_MULTIPLE_SCRIPT" ]; then
        echo "Error: play_multiple_rosbags.py not found at $PLAY_MULTIPLE_SCRIPT" | tee -a $LAUNCH_LOG_FILE
        echo "Falling back to single rosbag playback..." | tee -a $LAUNCH_LOG_FILE
        run_with_playback_log ros2 bag play "${ROSBAG}" -r "$PLAYBACK_RATE" "${PLAY_OFFSET_ARGS[@]}" "${CLOCK_ARGS[@]}"
    else
        echo "Using play_multiple_rosbags.py for simultaneous playback..." | tee -a $LAUNCH_LOG_FILE
        # 比較用rosbagの存在確認
        if [ ! -f "$COMPARE_BAG" ] && [ ! -d "$COMPARE_BAG" ]; then
            echo "Error: Compare bag not found: $COMPARE_BAG" | tee -a $LAUNCH_LOG_FILE
            echo "Falling back to single rosbag playback..." | tee -a $LAUNCH_LOG_FILE
            run_with_playback_log ros2 bag play "${ROSBAG}" -r "$PLAYBACK_RATE" "${PLAY_OFFSET_ARGS[@]}" "${CLOCK_ARGS[@]}"
        else
            # play_multiple_rosbags.pyを実行
            # 注意: --topics-onlyはros2 bag playに--exclude-topicsがないため機能しません
            # --remapオプションで記録したrosbagのトピック名を変更して競合を避けます
            # Pythonの出力バッファリングを無効化するため、-uオプションを使用
            echo "Executing: python3 -u $PLAY_MULTIPLE_SCRIPT ... -r $PLAYBACK_RATE ${PLAY_OFFSET_ARGS[*]}" | tee -a $LAUNCH_LOG_FILE
            run_with_playback_log python3 -u "$PLAY_MULTIPLE_SCRIPT" \
                --source-bag "$ROSBAG" \
                --recorded-bag "$COMPARE_BAG" \
                --recorded-topics "${COMPARE_TOPICS[@]}" \
                --remap /localization/pose_twist_fusion_filter/biased_pose_with_covariance:=/localization/pose_twist_fusion_filter/biased_pose_with_covariance_recorded \
                -r "$PLAYBACK_RATE" \
                "${PLAY_OFFSET_ARGS[@]}"
        fi
    fi
else
    # 通常の単一rosbag再生（/clock の有無は上記で判定）
    run_with_playback_log ros2 bag play "${ROSBAG}" -r "$PLAYBACK_RATE" "${PLAY_OFFSET_ARGS[@]}" "${CLOCK_ARGS[@]}"
fi

if [ -n "$END_UNIX_TIME" ]; then
    if [ "$USE_SIM_TIME" = "true" ]; then
        echo "Starting end-time monitor: stop playback when /clock >= $END_UNIX_TIME" | tee -a $LAUNCH_LOG_FILE
        start_end_time_monitor "$END_UNIX_TIME" "$ROSBAG_PID"
    else
        echo "Warning: -T/--end-time requires /clock monitoring; USE_SIM_TIME=false, so end-time monitor is disabled." | tee -a $LAUNCH_LOG_FILE
    fi
fi

# rosbagからPointCloudデータが流れ始めるまで待つ（1メッセージ受信 or 最大60秒）
# use_sim_time 時は echo も --use-sim-time。concatenated/pointcloud は Sensor 相当 QoS (best effort) なので明示する。
SENSOR_TOPIC="/sensing/lidar/concatenated/pointcloud"
_ECHO_Q=(ros2 topic echo "$SENSOR_TOPIC" --once --qos-reliability best_effort)
if [ "$USE_SIM_TIME" = "true" ]; then
    _ECHO_Q=(ros2 topic echo "$SENSOR_TOPIC" --once --use-sim-time --qos-reliability best_effort)
fi
echo "Waiting for first message on $SENSOR_TOPIC (timeout 60s)..." | tee -a $LAUNCH_LOG_FILE
if timeout 60 "${_ECHO_Q[@]}" > /dev/null 2>&1; then
    echo "Sensor data started flowing." | tee -a $LAUNCH_LOG_FILE
else
    echo "Warning: Timeout waiting for $SENSOR_TOPIC (proceeding anyway)." | tee -a $LAUNCH_LOG_FILE
fi
unset _ECHO_Q

# 初期位置を自動設定
# ROSBAGと同じディレクトリにあるinitial_pose.yamlを探す
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
    # 初期位置設定後、localizationが安定するまで待つ
    sleep 5
else
    echo "Info: initial_pose.yaml not found at $INITIAL_POSE_YAML" | tee -a $LAUNCH_LOG_FILE
    echo "Please set initial pose manually via RViz (2D Pose Estimate)" | tee -a $LAUNCH_LOG_FILE
fi

# rosbagプロセスが終了するまで待つ
echo "Waiting for rosbag playback to complete..." | tee -a $LAUNCH_LOG_FILE
wait $ROSBAG_PID

# end-time monitor が残っていれば停止
if [ -n "$END_TIME_MONITOR_PID" ]; then
    if kill -0 "$END_TIME_MONITOR_PID" 2>/dev/null; then
        kill -INT "$END_TIME_MONITOR_PID" 2>/dev/null || true
        wait "$END_TIME_MONITOR_PID" 2>/dev/null || true
    fi
fi

# /tf_static helper が動作中なら停止
if [ -n "$TF_STATIC_PLAYER_PID" ]; then
    if kill -0 "$TF_STATIC_PLAYER_PID" 2>/dev/null; then
        echo "Stopping /tf_static helper player (PID $TF_STATIC_PLAYER_PID)..." | tee -a $LAUNCH_LOG_FILE
        kill -INT "$TF_STATIC_PLAYER_PID" 2>/dev/null || true
        wait "$TF_STATIC_PLAYER_PID" 2>/dev/null || true
        echo "/tf_static helper player stopped." | tee -a $LAUNCH_LOG_FILE
    fi
fi

# 再生終了直後に RViz 録画を終了（SIGINT で ffmpeg に正常終了させてファイル確定）
if [ -n "$RVIZ_CAPTURE_PID" ]; then
    if kill -0 "$RVIZ_CAPTURE_PID" 2>/dev/null; then
        echo "Stopping RViz capture (PID $RVIZ_CAPTURE_PID)..." | tee -a $LAUNCH_LOG_FILE
        kill -INT "$RVIZ_CAPTURE_PID" 2>/dev/null || true
        wait "$RVIZ_CAPTURE_PID" 2>/dev/null || true
        echo "RViz capture stopped." | tee -a $LAUNCH_LOG_FILE
    fi
fi
