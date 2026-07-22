#!/bin/bash

bag_has_clock_topic() {
    local bag_path="$1"
    ros2 bag info "$bag_path" 2>/dev/null | awk '
        /Topic:[[:space:]]*\/clock[[:space:]]*\|/ { found=1 }
        END { exit(found ? 0 : 1) }
    '
}

bag_has_tf_static_topic() {
    local bag_path="$1"
    ros2 bag info "$bag_path" 2>/dev/null | awk '
        /Topic:[[:space:]]*\/tf_static[[:space:]]*\|/ { found=1 }
        END { exit(found ? 0 : 1) }
    '
}
