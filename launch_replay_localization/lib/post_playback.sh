#!/bin/bash

wait_and_finalize() {
    if [ "$PRIME_BAG_PLAYBACK_USED" != "true" ]; then
        local SENSOR_TOPIC="/sensing/lidar/concatenated/pointcloud"
        local _ECHO_Q=(ros2 topic echo "$SENSOR_TOPIC" --once --qos-reliability best_effort)
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

        echo "DEBUG: Checking for initial_pose.yaml at: $INITIAL_POSE_YAML" | tee -a $LAUNCH_LOG_FILE

        if [ -f "$INITIAL_POSE_YAML" ]; then
            echo "Setting initial pose from $INITIAL_POSE_YAML..." | tee -a $LAUNCH_LOG_FILE
            python3 "$SCRIPT_DIR/py/set_initial_pose.py" "$INITIAL_POSE_YAML" 2>&1 | tee -a $LAUNCH_LOG_FILE
            local POSE_SET_RESULT=${PIPESTATUS[0]}
            if [ $POSE_SET_RESULT -eq 0 ]; then
                echo "Initial pose set successfully" | tee -a $LAUNCH_LOG_FILE
            else
                echo "Error: Failed to set initial pose (exit code: $POSE_SET_RESULT)" | tee -a $LAUNCH_LOG_FILE
            fi
            sleep 5
        else
            echo "Info: initial_pose.yaml not found at $INITIAL_POSE_YAML" | tee -a $LAUNCH_LOG_FILE
            echo "Please set initial pose manually via RViz (2D Pose Estimate)" | tee -a $LAUNCH_LOG_FILE
        fi
    else
        echo "Info: PRIME_BAG_PLAYBACK_USED — skipping pointcloud wait and set_initial_pose (handled by prime_bag_playback.py)" | tee -a $LAUNCH_LOG_FILE
    fi

    echo "Waiting for rosbag playback to complete..." | tee -a $LAUNCH_LOG_FILE
    wait $ROSBAG_PID

    if [ -n "${RECORD_ROSBAG_PID:-}" ]; then
        local _rp_done="$RECORD_ROSBAG_PID"
        unset RECORD_ROSBAG_PID
        stop_rosbag_record_wrapper "$_rp_done"
    fi
    unset _rp_done 2>/dev/null || true

    if [ -n "$END_TIME_MONITOR_PID" ]; then
        if kill -0 "$END_TIME_MONITOR_PID" 2>/dev/null; then
            kill -INT "$END_TIME_MONITOR_PID" 2>/dev/null || true
            wait "$END_TIME_MONITOR_PID" 2>/dev/null || true
        fi
    fi

    if [ -n "$TF_STATIC_PLAYER_PID" ]; then
        if kill -0 "$TF_STATIC_PLAYER_PID" 2>/dev/null; then
            echo "Stopping /tf_static helper player (PID $TF_STATIC_PLAYER_PID)..." | tee -a $LAUNCH_LOG_FILE
            kill -INT "$TF_STATIC_PLAYER_PID" 2>/dev/null || true
            wait "$TF_STATIC_PLAYER_PID" 2>/dev/null || true
            echo "/tf_static helper player stopped." | tee -a $LAUNCH_LOG_FILE
        fi
    fi

    if [ -n "$RVIZ_CAPTURE_PID" ]; then
        if kill -0 "$RVIZ_CAPTURE_PID" 2>/dev/null; then
            echo "Stopping RViz capture (PID $RVIZ_CAPTURE_PID)..." | tee -a $LAUNCH_LOG_FILE
            kill -INT "$RVIZ_CAPTURE_PID" 2>/dev/null || true
            wait "$RVIZ_CAPTURE_PID" 2>/dev/null || true
            echo "RViz capture stopped." | tee -a $LAUNCH_LOG_FILE
        fi
    fi
}
