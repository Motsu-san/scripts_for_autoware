#!/bin/bash
# List of MAP_PATH|ROSBAG_PATH pairs for batch_replay.sh.
# Sourced by batch_replay.sh.
# Format: "MAP_PATH|ROSBAG_PATH"
# Lines starting with # and empty lines are ignored.

declare -a REPLAY_CONFIGS=(
    # Format: "MAP_PATH|ROSBAG_PATH"
    # "$HOME/autoware_map|$HOME/rosbag_replay/rosbag_0.db3"
    # Add more MAP_PATH|ROSBAG_PATH pairs as needed
)
