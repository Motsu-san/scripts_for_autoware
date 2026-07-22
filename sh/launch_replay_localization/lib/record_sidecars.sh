#!/bin/bash

start_record_sidecars() {
    RVIZ_CAPTURE_PID=""
    if [ "$RECORD_RVIZ" = "true" ]; then
        local RVIZ_CAPTURE_SCRIPT="${RVIZ_CAPTURE_SCRIPT:-$SCRIPT_DIR/capture_rviz_display.sh}"
        if [ ! -f "$RVIZ_CAPTURE_SCRIPT" ]; then
            echo "Warning: --record-rviz requested but $RVIZ_CAPTURE_SCRIPT not found, skipping RViz capture." | tee -a $LAUNCH_LOG_FILE
        elif [ -z "${DISPLAY:-}" ]; then
            echo "Warning: --record-rviz requested but DISPLAY is not set, skipping RViz capture." | tee -a $LAUNCH_LOG_FILE
        else
            echo "Waiting for RViz window to appear (timeout 90s)..." | tee -a $LAUNCH_LOG_FILE
            local RVIZ_WAIT_SEC=0
            while [ $RVIZ_WAIT_SEC -lt 90 ]; do
                local RVIZ_WID
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

    if [ -n "$TOPIC_TYPE" ]; then
        echo "Starting rosbag recording (TOPIC_TYPE=$TOPIC_TYPE)..." | tee -a $LAUNCH_LOG_FILE
        "$SCRIPT_DIR/record_rosbag_localization_replay.sh" "$OUTPUT_DIR" "$TOPIC_TYPE" &
        RECORD_ROSBAG_PID=$!
    else
        echo "Rosbag recording disabled (TOPIC_TYPE not specified)" | tee -a $LAUNCH_LOG_FILE
    fi
}
