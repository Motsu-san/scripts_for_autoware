#!/bin/bash

# Usage: ./batch_replay.sh
# Batch execution script for multiple rosbag replays

# Configuration: MAP_PATH and ROSBAG pairs
declare -a REPLAY_CONFIGS=(
    # Format: "MAP_PATH|ROSBAG_PATH"
    # "$HOME/autoware_map|$HOME/rosbag_replay/rosbag_0.db3"
    # Add more MAP_PATH|ROSBAG_PATH pairs as needed
)

# Get script directory
SCRIPT_DIR=$(dirname "$0")

# Setup logging
LOG_FILE="$SCRIPT_DIR/batch_replay.log"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
SAVE_LAUNCH_LOG="true"
RECORD_MODE="none"

# Function to log messages to both console and file
log_message() {
    local message="$1"
    echo "$message"
    echo "[$TIMESTAMP] $message" >> "$LOG_FILE"
}

log_message "Starting batch replay execution..."
log_message "Total configurations: ${#REPLAY_CONFIGS[@]}"
log_message ""

for i in "${!REPLAY_CONFIGS[@]}"; do
    config="${REPLAY_CONFIGS[$i]}"

    # Update timestamp for each iteration
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

    # Split MAP_PATH and ROSBAG_PATH
    IFS='|' read -r map_path rosbag_path <<< "$config"

    log_message "=== Configuration $((i+1))/${#REPLAY_CONFIGS[@]} ==="
    log_message "MAP_PATH: $map_path"
    log_message "ROSBAG: $rosbag_path"
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

    # Execute launch_autoware.sh
    log_message "Executing: $SCRIPT_DIR/launch_autoware.sh \"$map_path\" \"$rosbag_path\" \"$SAVE_LAUNCH_LOG\" \"$RECORD_MODE\""
    "$SCRIPT_DIR/launch_autoware.sh" "$map_path" "$rosbag_path" "$SAVE_LAUNCH_LOG" "$RECORD_MODE"

    # Check exit status and run analysis if successful
    if [ $? -eq 0 ]; then
        log_message "Configuration $((i+1)) rosbag replay completed successfully"        # Find the generated log file for analysis
        rosbag_dir=$(dirname "$rosbag_path")
        latest_log_dir=$(find "$rosbag_dir" -maxdepth 1 -type d -name "record_replay_*" | sort | tail -1)

        if [ -n "$latest_log_dir" ]; then
            # Find the latest datetime directory in the log directory
            # datetime_dir=$(find "$latest_log_dir" -maxdepth 1 -type d -name "????????_??????" | sort | tail -1)

            # if [ -n "$datetime_dir" ]; then
                log_file=$(find "$latest_log_dir" -name "*.db3" | head -1)

                if [ -n "$log_file" ]; then
                    log_message "Found log file: $log_file"
                    log_message "Starting NDT convergence analysis..."

                    # Execute ndt_conv_eval.sh
                    "$SCRIPT_DIR/ndt_conv_eval.sh" "$log_file" "$map_path"

                    if [ $? -eq 0 ]; then
                        log_message "NDT convergence analysis completed successfully"
                    else
                        log_message "NDT convergence analysis failed"
                    fi
                else
                    log_message "Warning: No .db3 log file found in $datetime_dir"
                fi
            # else
            #     log_message "Warning: No datetime directory found in $latest_log_dir"
            # fi
        else
            log_message "Warning: No replay log directory found in $rosbag_dir"
        fi
    else
        log_message "Configuration $((i+1)) failed with error"
    fi

    log_message ""
    log_message "Waiting 5 seconds before next execution..."
    sleep 5
done

log_message "All batch replay executions completed."
