#!/bin/bash

stop_rosbag_record_wrapper() {
    local pid="$1"
    [ -z "$pid" ] && return 0
    kill -0 "$pid" 2>/dev/null || return 0
    if [ -n "${LAUNCH_LOG_FILE:-}" ]; then
        echo "Stopping rosbag record (PID $pid)..." | tee -a "$LAUNCH_LOG_FILE"
    else
        echo "Stopping rosbag record (PID $pid)..."
    fi
    kill -INT "$pid" 2>/dev/null || true
    local max="${RECORD_STOP_WAIT_INT_SEC:-25}"
    local i=0
    while kill -0 "$pid" 2>/dev/null && [ "$i" -lt "$max" ]; do
        sleep 1
        i=$((i + 1))
    done
    if kill -0 "$pid" 2>/dev/null; then
        pkill -P "$pid" 2>/dev/null || true
        sleep 1
    fi
    if kill -0 "$pid" 2>/dev/null; then
        if [ -n "${LAUNCH_LOG_FILE:-}" ]; then
            echo "Record still running after SIGINT ~${max}s; sending SIGTERM..." | tee -a "$LAUNCH_LOG_FILE"
        else
            echo "Record still running after SIGINT ~${max}s; sending SIGTERM..."
        fi
        kill -TERM "$pid" 2>/dev/null || true
        sleep 2
    fi
    if kill -0 "$pid" 2>/dev/null; then
        if [ -n "${LAUNCH_LOG_FILE:-}" ]; then
            echo "Record still running; sending SIGKILL..." | tee -a "$LAUNCH_LOG_FILE"
        else
            echo "Record still running; sending SIGKILL..."
        fi
        kill -KILL "$pid" 2>/dev/null || true
        sleep 1
    fi
    wait "$pid" 2>/dev/null || true
    if [ -n "${LAUNCH_LOG_FILE:-}" ]; then
        echo "Rosbag record finished." | tee -a "$LAUNCH_LOG_FILE"
    else
        echo "Rosbag record finished."
    fi
}

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
    if [ -n "${RECORD_ROSBAG_PID:-}" ] && kill -0 "$RECORD_ROSBAG_PID" 2>/dev/null; then
        local _rp="$RECORD_ROSBAG_PID"
        unset RECORD_ROSBAG_PID
        stop_rosbag_record_wrapper "$_rp"
    fi
    "$SCRIPT_DIR/kill_autoware.sh"
}

register_cleanup_trap() {
    RVIZ_CAPTURE_PID=""
    TF_STATIC_PLAYER_PID=""
    END_TIME_MONITOR_PID=""
    RECORD_ROSBAG_PID=""
    trap cleanup_on_exit EXIT INT TERM HUP
}
