#!/bin/bash
# Shared paths and runtime state for launch_replay_localization.
# Set by launch_autoware.sh before sourcing other lib modules:
#   CALL_DIR, SCRIPT_DIR, LIB_DIR, LAUNCH_SCRIPT_NAME

# Positional / CLI (parse_args.sh)
# MAP_PATH, ROSBAG, POSE_SOURCE_ID, SAVE_LAUNCH_LOG, TOPIC_TYPE
# COMPARE_BAG, COMPARE_TOPICS, START_UNIX_TIME, END_UNIX_TIME
# PLAYBACK_RATE, RECORD_RVIZ, FORCE_SAMPLE_VEHICLE, GNSS_RECEIVER

# Launch configuration (launch_defaults.sh)
# LAUNCH_*, POSE_SOURCE, USE_SIM_TIME, RVIZ, RVIZ_CONFIG, LAUNCH_LOG_FILE, LOG_DIR

# Vehicle (vehicle_detect.sh)
# VEHICLE_MODEL, VEHICLE_ID, SENSOR_MODEL, VEHICLE_CONFIG

# Playback offsets (time_utils.sh)
# START_OFFSET_SEC, PLAY_OFFSET_ARGS, PLAY_DURATION_SEC, END_OFFSET_SEC, BAG_START_SEC

# Paths / output (launch_defaults.sh)
# DATETIME, ROSBAG_DIR, INITIAL_POSE_YAML, OUTPUT_DIR

# Background PIDs (cleanup.sh / sidecars / playback)
# RVIZ_CAPTURE_PID, TF_STATIC_PLAYER_PID, END_TIME_MONITOR_PID
# RECORD_ROSBAG_PID, ROSBAG_PID

# Playback state (playback.sh / post_playback.sh)
# PRIME_BAG_PLAYBACK, PRIME_BAG_PLAYBACK_USED, CLOCK_ARGS, PLAY_CLOCK_HZ

# Launch args (launch_defaults.sh)
# _POSE_SOURCE_LAUNCH_ARG

tee_log() {
    if [ -n "${LAUNCH_LOG_FILE:-}" ]; then
        tee -a "$LAUNCH_LOG_FILE"
    else
        cat
    fi
}
