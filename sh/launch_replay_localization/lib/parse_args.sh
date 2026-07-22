#!/bin/bash

print_launch_usage() {
    echo "Usage: $LAUNCH_SCRIPT_NAME <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE] [--compare-bag COMPARE_BAG] [--compare-topics TOPIC1 TOPIC2 ...] [-t START_TIME] [-T END_TIME]"
    echo "Must provide <MAP_PATH> <ROSBAG_PATH> and call from the directory where autoware is located and built(ex. $HOME/autoware)."
    echo "POSE_SOURCE_ID: 0=ndt (default), 1=ndt_lidar-marker, 99=odometry only (pose_source:=\"\")"
    echo "SAVE_LAUNCH_LOG: 'true' to save ros2 launch log, anything else or omitted disables log saving"
    echo "TOPIC_TYPE: default, lidar-marker_replay, full-sensing_replay, output, output_pose_mean, output_lidar-marker, convergence_evaluation, occlusion_adding, ..."
    echo "  If TOPIC_TYPE is omitted, rosbag recording will be disabled"
    echo "--compare-bag: Path to recorded rosbag for comparison (optional)"
    echo "--compare-topics: Topics to replay from recorded rosbag (optional, requires --compare-bag)"
    echo "--rate RATE: Playback rate for ros2 bag play (default: 0.2)"
    echo "--force-sample-vehicle: Use Autoware sample_vehicle/default/sample_sensor_kit from vehicle_configs (ignore bag path detection)"
    echo "--gnss-receiver NAME: GNSS preset for localization_standalone sensing (sample kit: ublox | septentrio). Default: env GNSS_RECEIVER or septentrio"
    echo "--record-rviz: Record RViz display (starts when RViz window appears, stops when playback ends)"
    echo "-t TIME: Start playback from this time. UNIX time (e.g. 1772096549.105) or JST datetime (e.g. '2026-02-26 12:34:56')"
    echo "-T, --end-time TIME: End playback at this absolute time. UNIX/JST accepted"
    echo "Example: $LAUNCH_SCRIPT_NAME \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 1 true output"
    echo "Example with -t (UNIX): $LAUNCH_SCRIPT_NAME \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 0 false '' -t 1772096549.105"
    echo "Example with -t (JST):  $LAUNCH_SCRIPT_NAME \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 0 false '' -t '2026-02-26 12:34:56'"
    echo "Example with comparison: $LAUNCH_SCRIPT_NAME \"$HOME/autoware_map\" \"$HOME/rosbag_replay/rosbag_0.db3\" 1 true output --compare-bag \"$HOME/rosbag_replay/recorded.bag\" --compare-topics /localization/kinematic_state"
}

parse_launch_args() {
    local POSITIONAL_ARGS=()
    COMPARE_BAG=""
    COMPARE_TOPICS=()
    START_UNIX_TIME=""
    END_UNIX_TIME=""
    PLAYBACK_RATE=""
    RECORD_RVIZ="false"
    FORCE_SAMPLE_VEHICLE="false"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --compare-bag)
                COMPARE_BAG="$2"
                shift 2
                ;;
            --compare-topics)
                shift
                while [[ $# -gt 0 ]] && [[ ! "$1" =~ ^-- ]]; do
                    COMPARE_TOPICS+=("$1")
                    shift
                done
                ;;
            --rate)
                PLAYBACK_RATE="$2"
                shift 2
                ;;
            --record-rviz)
                RECORD_RVIZ="true"
                shift
                ;;
            --force-sample-vehicle)
                FORCE_SAMPLE_VEHICLE="true"
                shift
                ;;
            --gnss-receiver)
                GNSS_RECEIVER="$2"
                shift 2
                ;;
            -t)
                START_UNIX_TIME="$2"
                shift 2
                ;;
            -T|--end-time)
                END_UNIX_TIME="$2"
                shift 2
                ;;
            *)
                POSITIONAL_ARGS+=("$1")
                shift
                ;;
        esac
    done

    set -- "${POSITIONAL_ARGS[@]}"

    if [ $# -lt 2 ] || [ $# -gt 5 ] || [ ! -f "$CALL_DIR/install/setup.bash" ]; then
        print_launch_usage
        exit 1
    fi

    MAP_PATH="$1"
    ROSBAG="$2"
    POSE_SOURCE_ID="${3:-0}"
    SAVE_LAUNCH_LOG="${4:-false}"
    TOPIC_TYPE="${5:-}"
    PLAYBACK_RATE="${PLAYBACK_RATE:-0.2}"
}
