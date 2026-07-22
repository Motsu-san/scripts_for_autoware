#!/bin/bash

validate_inputs() {
    local MISSING=()
    [ ! -f "$SCRIPT_DIR/kill_autoware.sh" ] && MISSING+=("$SCRIPT_DIR/kill_autoware.sh")
    [ -n "$TOPIC_TYPE" ] && [ ! -f "$SCRIPT_DIR/record_rosbag_localization_replay.sh" ] && MISSING+=("$SCRIPT_DIR/record_rosbag_localization_replay.sh")
    [ -n "$COMPARE_BAG" ] && [ ! -f "$SCRIPT_DIR/py/play_multiple_rosbags.py" ] && MISSING+=("$SCRIPT_DIR/py/play_multiple_rosbags.py")
    if [ ${#MISSING[@]} -gt 0 ]; then
        echo "Error: Required file(s) not found:" >&2
        printf '  %s\n' "${MISSING[@]}" >&2
        exit 1
    fi

    if [ -n "$COMPARE_BAG" ] && [ ${#COMPARE_TOPICS[@]} -eq 0 ]; then
        echo "Error: --compare-topics must be specified when using --compare-bag" >&2
        exit 1
    fi

    if [ ${#COMPARE_TOPICS[@]} -gt 0 ] && [ -z "$COMPARE_BAG" ]; then
        echo "Error: --compare-bag must be specified when using --compare-topics" >&2
        exit 1
    fi

    if [ ! -d "$MAP_PATH" ]; then
        echo "Error: MAP_PATH does not exist: $MAP_PATH"
        exit 1
    fi

    if [ ! -f "$ROSBAG" ] && [ ! -d "$ROSBAG" ]; then
        echo "Error: ROSBAG path does not exist: $ROSBAG"
        exit 1
    fi
}
