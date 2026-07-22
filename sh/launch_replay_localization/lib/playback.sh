#!/bin/bash

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

start_playback() {
    echo "Starting rosbag playback..." | tee -a $LAUNCH_LOG_FILE

    PRIME_BAG_PLAYBACK="${PRIME_BAG_PLAYBACK:-}"
    if [ -z "$PRIME_BAG_PLAYBACK" ]; then
        if [ -n "$START_UNIX_TIME" ] && [ -f "$INITIAL_POSE_YAML" ] && [ -z "$COMPARE_BAG" ] && [ "$USE_SIM_TIME" = "true" ]; then
            PRIME_BAG_PLAYBACK=true
        else
            PRIME_BAG_PLAYBACK=false
        fi
    fi
    if [ "$PRIME_BAG_PLAYBACK" = "true" ]; then
        echo "PRIME_BAG_PLAYBACK: enabled (start time + initial_pose.yaml, sim time aligned before resume)" | tee -a $LAUNCH_LOG_FILE
    else
        echo "PRIME_BAG_PLAYBACK: disabled (set PRIME_BAG_PLAYBACK=false to force off)" | tee -a $LAUNCH_LOG_FILE
    fi
    local PRIME_BAG_SCRIPT="$SCRIPT_DIR/py/prime_bag_playback.py"
    PRIME_BAG_PLAYBACK_USED=false

    PLAY_CLOCK_HZ="${PLAY_CLOCK_HZ:-200}"
    CLOCK_ARGS=()
    if bag_has_clock_topic "$ROSBAG"; then
        echo "Detected /clock in rosbag: play without --clock (use bag clock)" | tee -a $LAUNCH_LOG_FILE
    else
        CLOCK_ARGS=(--clock "$PLAY_CLOCK_HZ")
        echo "No /clock in rosbag: play with --clock $PLAY_CLOCK_HZ" | tee -a $LAUNCH_LOG_FILE
    fi

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

    if [ -n "$COMPARE_BAG" ]; then
        local PLAY_MULTIPLE_SCRIPT="$SCRIPT_DIR/py/play_multiple_rosbags.py"
        if [ ! -f "$PLAY_MULTIPLE_SCRIPT" ]; then
            echo "Error: play_multiple_rosbags.py not found at $PLAY_MULTIPLE_SCRIPT" | tee -a $LAUNCH_LOG_FILE
            echo "Falling back to single rosbag playback..." | tee -a $LAUNCH_LOG_FILE
            run_with_playback_log ros2 bag play "${ROSBAG}" -r "$PLAYBACK_RATE" "${PLAY_OFFSET_ARGS[@]}" "${CLOCK_ARGS[@]}"
        else
            echo "Using play_multiple_rosbags.py for simultaneous playback..." | tee -a $LAUNCH_LOG_FILE
            if [ ! -f "$COMPARE_BAG" ] && [ ! -d "$COMPARE_BAG" ]; then
                echo "Error: Compare bag not found: $COMPARE_BAG" | tee -a $LAUNCH_LOG_FILE
                echo "Falling back to single rosbag playback..." | tee -a $LAUNCH_LOG_FILE
                run_with_playback_log ros2 bag play "${ROSBAG}" -r "$PLAYBACK_RATE" "${PLAY_OFFSET_ARGS[@]}" "${CLOCK_ARGS[@]}"
            else
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
        if [ "$PRIME_BAG_PLAYBACK" = "true" ]; then
            if [ ! -f "$PRIME_BAG_SCRIPT" ]; then
                echo "Error: $PRIME_BAG_SCRIPT not found; falling back to direct ros2 bag play" | tee -a $LAUNCH_LOG_FILE
                run_with_playback_log ros2 bag play "${ROSBAG}" -r "$PLAYBACK_RATE" "${PLAY_OFFSET_ARGS[@]}" "${CLOCK_ARGS[@]}"
            else
                local _PRIME_ARGS=(
                    python3 -u "$PRIME_BAG_SCRIPT"
                    --bag "$ROSBAG"
                    --start-offset "${START_OFFSET_SEC:-0}"
                    -r "$PLAYBACK_RATE"
                    --initial-pose-yaml "$INITIAL_POSE_YAML"
                    --stamp-unix-sec "$START_UNIX_TIME"
                    --settle-sec "${INITIAL_POSE_SETTLE_SEC:-5}"
                )
                if bag_has_clock_topic "$ROSBAG"; then
                    _PRIME_ARGS+=(--use-bag-clock)
                else
                    _PRIME_ARGS+=(--clock-hz "$PLAY_CLOCK_HZ")
                fi
                echo "Executing: ${_PRIME_ARGS[*]}" | tee -a $LAUNCH_LOG_FILE
                run_with_playback_log "${_PRIME_ARGS[@]}"
                PRIME_BAG_PLAYBACK_USED=true
                unset _PRIME_ARGS
            fi
        else
            run_with_playback_log ros2 bag play "${ROSBAG}" -r "$PLAYBACK_RATE" "${PLAY_OFFSET_ARGS[@]}" "${CLOCK_ARGS[@]}"
        fi
    fi

    if [ -n "$END_UNIX_TIME" ]; then
        if [ "$USE_SIM_TIME" = "true" ]; then
            echo "Starting end-time monitor: stop playback when /clock >= $END_UNIX_TIME" | tee -a $LAUNCH_LOG_FILE
            start_end_time_monitor "$END_UNIX_TIME" "$ROSBAG_PID"
        else
            echo "Warning: -T/--end-time requires /clock monitoring; USE_SIM_TIME=false, so end-time monitor is disabled." | tee -a $LAUNCH_LOG_FILE
        fi
    fi
}
