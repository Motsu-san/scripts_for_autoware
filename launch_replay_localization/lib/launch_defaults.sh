#!/bin/bash

apply_launch_defaults() {
    LOG_DIR=$HOME/log

    LAUNCH_SENSING="${LAUNCH_SENSING:-true}"
    LAUNCH_DRIVER="${LAUNCH_DRIVER:-false}"
    LAUNCH_LOCALIZATION="${LAUNCH_LOCALIZATION:-true}"
    LAUNCH_API="${LAUNCH_API:-true}"
    LAUNCH_DEFAULT_AD_API="${LAUNCH_DEFAULT_AD_API:-true}"
    LAUNCH_RVIZ_ADAPTORS="${LAUNCH_RVIZ_ADAPTORS:-true}"
    LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
    LAUNCH_PERCEPTION="false"
    GNSS_RECEIVER="${GNSS_RECEIVER:-septentrio}"
    case "$GNSS_RECEIVER" in
        ublox|septentrio) ;;
        *)
            echo "Error: GNSS_RECEIVER / --gnss-receiver は ublox または septentrio のみ（現在: $GNSS_RECEIVER）" >&2
            exit 1
            ;;
    esac
    LAUNCH_PLANNING="false"
    LAUNCH_CONTROL="false"

    POSE_SOURCE=$(get_pose_source "$POSE_SOURCE_ID")
    if [ -n "$POSE_SOURCE" ]; then
        _POSE_SOURCE_LAUNCH_ARG=( "pose_source:=$POSE_SOURCE" )
    else
        _POSE_SOURCE_LAUNCH_ARG=( 'pose_source:=""' )
    fi

    _USE_SIM_TIME_EXPLICITLY_SET="${USE_SIM_TIME+yes}"
    USE_SIM_TIME="${USE_SIM_TIME:-true}"
    if [[ "$ROSBAG" == *"sample-rosbag"* ]] && [ -z "$COMPARE_BAG" ] && [ -z "$_USE_SIM_TIME_EXPLICITLY_SET" ]; then
        USE_SIM_TIME="false"
    fi
    unset _USE_SIM_TIME_EXPLICITLY_SET
    RVIZ="$LAUNCH_RVIZ"
    RVIZ_CONFIG="${RVIZ_CONFIG-}"

    if [ "$SAVE_LAUNCH_LOG" = "true" ]; then
        LAUNCH_LOG_FILE="$LOG_DIR/ros2_launch_$(date '+%Y%m%d_%H%M%S').log"
        mkdir -p "$LOG_DIR"
        touch "$LAUNCH_LOG_FILE"
        echo "ros2 launch log will be saved to $LAUNCH_LOG_FILE" | tee -a "$LAUNCH_LOG_FILE"
    else
        LAUNCH_LOG_FILE=""
    fi

    # 時刻解釈の前に yaml パスを確定（-t 未指定時の initial_pose.yaml フォールバック用）
    ROSBAG_DIR=$(dirname "$ROSBAG")
    INITIAL_POSE_YAML="${ROSBAG_DIR}/initial_pose.yaml"

    parse_time_args_before_ros

    DATETIME=$(date '+%Y%m%d_%H%M%S')

    echo "MAP_PATH: $MAP_PATH" | tee -a $LAUNCH_LOG_FILE
    echo "ROSBAG: $ROSBAG" | tee -a $LAUNCH_LOG_FILE
    echo "DATETIME: $DATETIME" | tee -a $LAUNCH_LOG_FILE
    echo "POSE_SOURCE: $POSE_SOURCE" | tee -a $LAUNCH_LOG_FILE
    if [ -n "$TOPIC_TYPE" ]; then
        echo "TOPIC_TYPE: $TOPIC_TYPE (rosbag recording enabled)" | tee -a $LAUNCH_LOG_FILE
    else
        echo "TOPIC_TYPE: (not specified, rosbag recording disabled)" | tee -a $LAUNCH_LOG_FILE
    fi

    if [ -n "$COMPARE_BAG" ]; then
        echo "COMPARE_BAG: $COMPARE_BAG" | tee -a $LAUNCH_LOG_FILE
        echo "COMPARE_TOPICS: ${COMPARE_TOPICS[*]}" | tee -a $LAUNCH_LOG_FILE
    fi
    if [ -n "$START_UNIX_TIME" ]; then
        echo "START_UNIX_TIME (-t): $START_UNIX_TIME" | tee -a $LAUNCH_LOG_FILE
        echo "START_OFFSET_SEC: $START_OFFSET_SEC" | tee -a $LAUNCH_LOG_FILE
    fi
    if [ -n "$END_UNIX_TIME" ]; then
        echo "END_UNIX_TIME (-T/--end-time): $END_UNIX_TIME" | tee -a $LAUNCH_LOG_FILE
    fi
    echo "PLAYBACK_RATE (--rate): $PLAYBACK_RATE" | tee -a $LAUNCH_LOG_FILE
    echo "RECORD_RVIZ (--record-rviz): $RECORD_RVIZ" | tee -a $LAUNCH_LOG_FILE
    echo "FORCE_SAMPLE_VEHICLE (--force-sample-vehicle): $FORCE_SAMPLE_VEHICLE" | tee -a $LAUNCH_LOG_FILE
}

prepare_output_paths() {
    OUTPUT_DIR=$ROSBAG_DIR/record_replay_$DATETIME
    if [ -d "$OUTPUT_DIR" ]; then
        echo "Warning: OUTPUT_DIR already exists, removing: $OUTPUT_DIR" | tee -a $LAUNCH_LOG_FILE
        rm -rf "$OUTPUT_DIR"
    fi
    echo "OUTPUT_DIR will be created by record script: $OUTPUT_DIR" | tee -a $LAUNCH_LOG_FILE
}

log_launch_configuration() {
    echo "Launch configuration:" | tee -a $LAUNCH_LOG_FILE
    echo "  SENSING: $LAUNCH_SENSING" | tee -a $LAUNCH_LOG_FILE
    echo "  DRIVER: $LAUNCH_DRIVER" | tee -a $LAUNCH_LOG_FILE
    echo "  LOCALIZATION: $LAUNCH_LOCALIZATION" | tee -a $LAUNCH_LOG_FILE
    echo "  API: $LAUNCH_API" | tee -a $LAUNCH_LOG_FILE
    echo "  DEFAULT_AD_API: $LAUNCH_DEFAULT_AD_API" | tee -a $LAUNCH_LOG_FILE
    echo "  RVIZ_ADAPTORS: $LAUNCH_RVIZ_ADAPTORS" | tee -a $LAUNCH_LOG_FILE
    echo "  PERCEPTION: $LAUNCH_PERCEPTION" | tee -a $LAUNCH_LOG_FILE
    echo "  PLANNING: $LAUNCH_PLANNING" | tee -a $LAUNCH_LOG_FILE
    echo "  CONTROL: $LAUNCH_CONTROL" | tee -a $LAUNCH_LOG_FILE
    echo "  POSE_SOURCE: $POSE_SOURCE" | tee -a $LAUNCH_LOG_FILE
    echo "  RVIZ: $RVIZ" | tee -a $LAUNCH_LOG_FILE
    if [ -n "$RVIZ_CONFIG" ]; then
        echo "  RVIZ_CONFIG: $RVIZ_CONFIG" | tee -a $LAUNCH_LOG_FILE
    else
        echo "  RVIZ_CONFIG: (launch default: tier4_localization_launch/rviz/autoware.rviz)" | tee -a $LAUNCH_LOG_FILE
    fi
    echo "  USE_SIM_TIME: $USE_SIM_TIME" | tee -a $LAUNCH_LOG_FILE
    echo "  GNSS_RECEIVER (localization_standalone gnss_receiver): $GNSS_RECEIVER" | tee -a $LAUNCH_LOG_FILE
    if [ "$LAUNCH_RVIZ" = "true" ]; then
        echo "  DISPLAY (for RViz): ${DISPLAY:-<unset>}" | tee -a $LAUNCH_LOG_FILE
        echo "  XAUTHORITY (for RViz): ${XAUTHORITY:-<unset>}" | tee -a $LAUNCH_LOG_FILE
    fi
}
