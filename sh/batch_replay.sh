#!/bin/bash

# Usage: ./batch_replay.sh
# Batch execution script for multiple rosbag replays.
# Config format in replay_configs.sh: MAP_PATH|ROSBAG_PATH|POSE_SOURCE_ID|TOPIC_TYPE|RECORD_RVIZ|PLAYBACK_RATE|UNIX_TIME

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f "$SCRIPT_DIR/replay_configs.sh" ]; then
    echo "Error: Required file not found: $SCRIPT_DIR/replay_configs.sh" >&2
    exit 1
fi
source "$SCRIPT_DIR/replay_configs.sh"

# Setup logging
LOG_FILE="$SCRIPT_DIR/batch_replay.log"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
SAVE_LAUNCH_LOG="true"

# Function to log messages to both console and file
log_message() {
    local message="$1"
    echo "$message"
    echo "[$TIMESTAMP] $message" >> "$LOG_FILE"
}

# Build list of valid configs (skip empty and comment lines)
VALID_CONFIGS=()
for c in "${REPLAY_CONFIGS[@]}"; do
    [[ -z "$c" || "$c" =~ ^[[:space:]]*# ]] && continue
    VALID_CONFIGS+=("$c")
done

log_message "Starting batch replay execution..."
log_message "Total configurations: ${#VALID_CONFIGS[@]}"
log_message ""

for i in "${!VALID_CONFIGS[@]}"; do
    config="${VALID_CONFIGS[$i]}"

    # Update timestamp for each iteration
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

    # Split: MAP_PATH|ROSBAG_PATH|POSE_SOURCE_ID|TOPIC_TYPE|RECORD_RVIZ|PLAYBACK_RATE|UNIX_TIME
    IFS='|' read -r map_path rosbag_path pose_source_id topic_type record_rviz playback_rate unix_time <<< "$config"
    pose_source_id="${pose_source_id:-0}"
    topic_type="${topic_type:-}"
    record_rviz="${record_rviz:-false}"
    playback_rate="${playback_rate:-}"
    unix_time="${unix_time:-}"

    log_message "=== Configuration $((i+1))/${#VALID_CONFIGS[@]} ==="
    log_message "MAP_PATH: $map_path"
    log_message "ROSBAG: $rosbag_path"
    log_message "POSE_SOURCE_ID: $pose_source_id, TOPIC_TYPE: $topic_type, RECORD_RVIZ: $record_rviz, PLAYBACK_RATE: $playback_rate, UNIX_TIME: $unix_time"
    log_message ""

    # Check if paths exist
    if [ ! -d "$map_path" ]; then
        log_message "Warning: MAP_PATH does not exist, skipping: $map_path"
        continue
    fi


    if [ ! -f "$rosbag_path" ]; then
        log_message "Warning: ROSBAG file does not exist, skipping: $rosbag_path"
        continue
    fi

    # Build launch_autoware.sh args: MAP_PATH ROSBAG_PATH [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE] [--record-rviz] [--rate RATE] [-t UNIX_TIME]
    LAUNCH_ARGS=("$map_path" "$rosbag_path" "$pose_source_id" "$SAVE_LAUNCH_LOG" "$topic_type")
    [ "$record_rviz" = "true" ] && LAUNCH_ARGS+=(--record-rviz)
    [ -n "$playback_rate" ] && LAUNCH_ARGS+=(--rate "$playback_rate")
    [ -n "$unix_time" ] && LAUNCH_ARGS+=(-t "$unix_time")

    log_message "Executing: $SCRIPT_DIR/launch_autoware.sh ${LAUNCH_ARGS[*]}"
    "$SCRIPT_DIR/launch_autoware.sh" "${LAUNCH_ARGS[@]}"

    # Check exit status and run analysis if successful
    # if [ $? -eq 0 ]; then
    #     log_message "Configuration $((i+1))/${#VALID_CONFIGS[@]} rosbag replay completed successfully"
    #     # Find the generated log file for analysis
    #     rosbag_dir=$(dirname "$rosbag_path")
    #     latest_log_dir=$(find "$rosbag_dir" -maxdepth 1 -type d -name "record_replay_*" | sort | tail -1)

    #     if [ -n "$latest_log_dir" ]; then
    #         # Find the latest datetime directory in the log directory
    #         # datetime_dir=$(find "$latest_log_dir" -maxdepth 1 -type d -name "????????_??????" | sort | tail -1)

    #         # if [ -n "$datetime_dir" ]; then
    #             log_file=$(find "$latest_log_dir" -name "*.db3" | head -1)

    #             if [ -n "$log_file" ]; then
    #                 log_message "Found log file: $log_file"
    #                 log_message "Starting NDT convergence analysis..."

    #                 # Execute ndt_conv_eval.sh
    #                 "$SCRIPT_DIR/ndt_conv_eval.sh" "$log_file" "$map_path"

    #                 if [ $? -eq 0 ]; then
    #                     log_message "NDT convergence analysis completed successfully"
    #                 else
    #                     log_message "NDT convergence analysis failed"
    #                 fi
    #             else
    #                 log_message "Warning: No .db3 log file found in $datetime_dir"
    #             fi
    #         # else
    #         #     log_message "Warning: No datetime directory found in $latest_log_dir"
    #         # fi
    #     else
    #         log_message "Warning: No replay log directory found in $rosbag_dir"
    #     fi
    # else
    #     log_message "Configuration $((i+1)) failed with error"
    # fi

    log_message ""
    log_message "Waiting 5 seconds before next execution..."
    sleep 5
done

log_message "All batch replay executions completed."
