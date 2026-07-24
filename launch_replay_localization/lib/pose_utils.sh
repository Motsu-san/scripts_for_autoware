#!/bin/bash

get_pose_source() {
    case "$1" in
        0)
            echo "ndt"
            ;;
        1)
            echo "ndt_lidar-marker"
            ;;
        99)
            echo ""
            ;;
        *)
            echo "Error: Invalid POSE_SOURCE_ID: $1. Valid values are: 0 (ndt), 1 (ndt_lidar-marker), 99 (odometry only)" >&2
            exit 1
            ;;
    esac
}

initial_pose_skip_localization() {
    local yaml_file="$1"
    [ -f "$yaml_file" ] || return 1
    python3 -c "
import sys, yaml
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f) or {}
sys.exit(0 if d.get('skip_initial_localization') else 1)
" "$yaml_file" 2>/dev/null
}

# initial_pose.yaml から開始 UNIX 時刻を読む(calibrate_odom_from_bag.load_initial_pose_yaml と同じ優先順位)
read_initial_pose_start_unix_sec() {
    local yaml_file="$1"
    [ -f "$yaml_file" ] || return 1
    python3 - "$yaml_file" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}

start_sec = None
for key in ("mean_pose_header_stamp", "header_stamp"):
    block = data.get(key)
    if isinstance(block, dict) and "sec" in block:
        start_sec = float(block["sec"]) + float(block.get("nanosec", 0)) * 1e-9
        break

pose_block = data.get("pose")
if start_sec is None and isinstance(pose_block, dict):
    hdr = pose_block.get("header")
    if isinstance(hdr, dict) and isinstance(hdr.get("stamp"), dict) and "sec" in hdr["stamp"]:
        stamp = hdr["stamp"]
        start_sec = float(stamp["sec"]) + float(stamp.get("nanosec", 0)) * 1e-9

if start_sec is None:
    root_hdr = data.get("header")
    if isinstance(root_hdr, dict) and isinstance(root_hdr.get("stamp"), dict) and "sec" in root_hdr["stamp"]:
        stamp = root_hdr["stamp"]
        start_sec = float(stamp["sec"]) + float(stamp.get("nanosec", 0)) * 1e-9

if start_sec is None:
    sys.exit(1)
print(start_sec)
PY
}

stop_pose_initializer_nodes() {
    local log_target="${LAUNCH_LOG_FILE:-/dev/stderr}"
    echo "Stopping pose_initializer / automatic_pose_initializer nodes..." | tee -a "$log_target"
    local pids
    pids=$(pgrep -f 'pose_initializer_node' 2>/dev/null || true)
    if [ -z "$pids" ]; then
        echo "Info: pose_initializer_node not found (may not be started yet)" | tee -a "$log_target"
        return 0
    fi
    for pid in $pids; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGTERM "$pid" 2>/dev/null || true
            echo "Stopped pose_initializer_node (PID: $pid)" | tee -a "$log_target"
        fi
    done
    sleep 1
}
