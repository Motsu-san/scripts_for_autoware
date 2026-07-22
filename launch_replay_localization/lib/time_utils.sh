#!/bin/bash

parse_jst_to_unix_sec() {
    local s="$1"
    local frac=""
    if [[ "$s" =~ \.[0-9]+$ ]]; then
        frac="${s##*.}"
        s="${s%.*}"
    fi
    s="${s//\//-}"
    local unix_sec
    unix_sec=$(TZ=Asia/Tokyo date -d "$s" +%s 2>/dev/null) || return 1
    [[ -z "$unix_sec" ]] && return 1
    if [[ -n "$frac" ]]; then
        echo "${unix_sec}.${frac}"
    else
        echo "${unix_sec}"
    fi
}

get_bag_start_unix_sec() {
    local bag_path="$1"
    local start_line
    start_line=$(ros2 bag info "$bag_path" 2>/dev/null | grep '^Start:')
    [[ -z "$start_line" ]] && return 1
    local unix_sec
    unix_sec=$(echo "$start_line" | sed -n 's/.*(\([0-9][0-9]*\.\?[0-9]*\)).*/\1/p')
    [[ -z "$unix_sec" ]] && return 1
    echo "$unix_sec"
}

parse_time_args_before_ros() {
    START_OFFSET_SEC=""
    PLAY_OFFSET_ARGS=()
    PLAY_DURATION_SEC=""
    END_OFFSET_SEC=""

    if [ -n "$START_UNIX_TIME" ]; then
        if [[ "$START_UNIX_TIME" =~ ^[0-9]+\.?[0-9]*$ ]]; then
            :
        else
            START_UNIX_TIME=$(parse_jst_to_unix_sec "$START_UNIX_TIME")
            if [ -z "$START_UNIX_TIME" ]; then
                echo "Error: -t could not parse as JST datetime (e.g. '2026-02-26 12:34:56' or '2026-02-26 12:34:56.123')" >&2
                exit 1
            fi
            echo "Parsed -t as JST -> UNIX time: $START_UNIX_TIME"
        fi
    fi
    # -t 未指定時: initial_pose.yaml の時刻を START_UNIX_TIME に使う（PRIME_BAG_PLAYBACK 等）
    if [ -z "$START_UNIX_TIME" ] && [ -n "${INITIAL_POSE_YAML:-}" ] && [ -f "$INITIAL_POSE_YAML" ]; then
        local _yaml_start_unix
        _yaml_start_unix=$(read_initial_pose_start_unix_sec "$INITIAL_POSE_YAML" 2>/dev/null) || _yaml_start_unix=""
        if [ -n "$_yaml_start_unix" ]; then
            START_UNIX_TIME="$_yaml_start_unix"
            echo "START_UNIX_TIME from initial_pose.yaml: $START_UNIX_TIME" | tee -a "${LAUNCH_LOG_FILE:-/dev/null}"
        fi
        unset _yaml_start_unix
    fi
    if [ -n "$END_UNIX_TIME" ]; then
        if [[ "$END_UNIX_TIME" =~ ^[0-9]+\.?[0-9]*$ ]]; then
            :
        else
            END_UNIX_TIME=$(parse_jst_to_unix_sec "$END_UNIX_TIME")
            if [ -z "$END_UNIX_TIME" ]; then
                echo "Error: -T/--end-time could not parse as JST datetime (e.g. '2026-02-26 12:34:56' or '2026-02-26 12:34:56.123')" >&2
                exit 1
            fi
            echo "Parsed -T/--end-time as JST -> UNIX time: $END_UNIX_TIME"
        fi
    fi
}

compute_playback_offsets() {
    if [ -n "$START_UNIX_TIME" ] || [ -n "$END_UNIX_TIME" ]; then
        BAG_START_SEC=$(get_bag_start_unix_sec "$ROSBAG")
        if [ -z "$BAG_START_SEC" ]; then
            echo "Error: Could not get bag start time for -t/-T option (run 'ros2 bag info $ROSBAG' to check)" >&2
            exit 1
        fi
    fi

    if [ -n "$START_UNIX_TIME" ]; then
        START_OFFSET_SEC=$(echo "$START_UNIX_TIME $BAG_START_SEC" | awk '{s=$1-$2; if(s<0)s=0; printf "%.3f", s}')
        PLAY_OFFSET_ARGS=(--start-offset "$START_OFFSET_SEC")
        echo "Start from UNIX time -t $START_UNIX_TIME (bag start: $BAG_START_SEC) -> --start-offset ${START_OFFSET_SEC}s"
    fi

    if [ -n "$END_UNIX_TIME" ]; then
        END_OFFSET_SEC=$(echo "$END_UNIX_TIME $BAG_START_SEC" | awk '{e=$1-$2; if(e<0)e=0; printf "%.3f", e}')
        if [ -n "$START_OFFSET_SEC" ]; then
            PLAY_DURATION_SEC=$(echo "$END_OFFSET_SEC $START_OFFSET_SEC" | awk '{d=$1-$2; printf "%.3f", d}')
        else
            PLAY_DURATION_SEC="$END_OFFSET_SEC"
        fi
        if ! awk "BEGIN{exit !($PLAY_DURATION_SEC > 0)}"; then
            echo "Error: -T/--end-time must be later than effective start time (computed duration=${PLAY_DURATION_SEC}s)." >&2
            exit 1
        fi
        echo "End at UNIX time -T $END_UNIX_TIME (bag start: $BAG_START_SEC) -> stop on /clock monitor"
    fi
}
