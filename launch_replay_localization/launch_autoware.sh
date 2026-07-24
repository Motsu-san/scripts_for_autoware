#!/bin/bash

# Usage: ./launch_autoware.sh <MAP_PATH> <ROSBAG_PATH>
# Example: ./launch_autoware.sh "$HOME/autoware_map/Komatsu/628-20250619100818540138" "$HOME/rosbag_replay/data1/final_merged/final_merged_0.db3"
#
# Requirements (必須・条件付きで参照するファイル。先頭で存在チェックする):
#   - 常時: $SCRIPT_DIR/vehicle_configs.sh
#   - 常時: $SCRIPT_DIR/kill_autoware.sh
#   - 常時: カレントディレクトリが autoware ビルド済みで install/setup.bash が存在すること(引数チェックで検証)
#   - TOPIC_TYPE 指定時: $SCRIPT_DIR/record_rosbag_localization_replay.sh
#   - --compare-bag 指定時: $SCRIPT_DIR/py/play_multiple_rosbags.py
#   - 使用時に存在確認: $SCRIPT_DIR/py/set_initial_pose.py, gnss_to_initial_pose.py
#   - --record-rviz 指定時: $SCRIPT_DIR/capture_rviz_display.sh と xdotool(RViz ウィンドウ検出用)
#
# POSE_SOURCE_ID: 0=ndt (default), 1=ndt_lidar-marker, 99=odometry only (pose_source:="")
# SAVE_LAUNCH_LOG: "true" to save ros2 launch log, anything else or omitted disables log saving
# TOPIC_TYPE: Topic type for record_rosbag.sh (default, lidar-marker_replay, full-sensing_replay, output, ...)
# --compare-bag, --compare-topics, --rate, --force-sample-vehicle, --gnss-receiver, --record-rviz, -t, -T
#
# -t TIME: Start playback from this time (UNIX or JST).
#   未指定時は dirname(ROSBAG)/initial_pose.yaml の時刻(mean_pose_header_stamp / header_stamp /
#   pose.header.stamp / header.stamp)を START_UNIX_TIME として使う。
#   initial_pose.yaml あり・開始時刻あり・単一 bag・use_sim_time 時は py/prime_bag_playback.py で
#   一時停止オフセット再生 → sim time 合わせ → 初期位置 → resume(PRIME_BAG_PLAYBACK=false で無効化可)。
#
# 起動: tier4_localization_launch/localization_standalone.launch.xml があればそれを使用。
# 無い環境(旧 Autoware 等)では autoware_launch/logging_simulator.launch.xml にフォールバック。
# For unified_localization (NDT+EKF in one node), use launch_unified_localization.sh instead.
#
# sample-rosbag 再生時は use_sim_time=false かつ --clock なしで再生し、RViz チラつきを防ぐ(自動)。
# 環境変数: LAUNCH_VEHICLE, LAUNCH_SYSTEM, LAUNCH_MAP, LAUNCH_SENSING, LAUNCH_SENSING_DRIVER,
#   LAUNCH_API, LAUNCH_LOCALIZATION, LAUNCH_RVIZ, GNSS_RECEIVER, USE_SIM_TIME, RVIZ_CONFIG, ...

CALL_DIR=$(pwd)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="$SCRIPT_DIR/lib"
LAUNCH_SCRIPT_NAME="$0"

if [ ! -f "$SCRIPT_DIR/vehicle_configs.sh" ]; then
    echo "Error: Required file not found: $SCRIPT_DIR/vehicle_configs.sh" >&2
    exit 1
fi
source "$SCRIPT_DIR/vehicle_configs.sh"

source "$LIB_DIR/common.sh"
source "$LIB_DIR/parse_args.sh"
parse_launch_args "$@"

source "$LIB_DIR/validate_inputs.sh"
validate_inputs

source "$LIB_DIR/pose_utils.sh"
source "$LIB_DIR/time_utils.sh"
source "$LIB_DIR/launch_defaults.sh"
apply_launch_defaults

source "$LIB_DIR/vehicle_detect.sh"
detect_and_log_vehicle

source "$LIB_DIR/display.sh"
if [ "$LAUNCH_RVIZ" = "true" ]; then
    setup_rviz_display || exit 1
fi
log_launch_configuration

source "$LIB_DIR/cleanup.sh"
register_cleanup_trap
prepare_output_paths

source "$LIB_DIR/env_setup.sh"
setup_ros_env

source "$LIB_DIR/bag_utils.sh"
source "$LIB_DIR/package_checks.sh"
check_required_packages

compute_playback_offsets

source "$LIB_DIR/launch_nodes.sh"
launch_autoware_nodes

source "$LIB_DIR/record_sidecars.sh"
start_record_sidecars

source "$LIB_DIR/playback.sh"
start_playback

source "$LIB_DIR/post_playback.sh"
wait_and_finalize
