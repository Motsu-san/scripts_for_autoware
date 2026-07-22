#!/bin/bash
# Mapping of vehicle ID fragment to vehicle_model / vehicle_id / sensor_model.
# Sourced by launch_autoware.sh and launch_unified_localization.sh.
# Format: "vehicle_id_fragment|vehicle_model|vehicle_id|sensor_model"
# Lines starting with # and empty lines are ignored.

declare -a VEHICLE_CONFIGS=(
    # example: "vehicle_fragment|vehicle_model|vehicle_id|sensor_model"
    # vehicle_fragment: Part of vehicle ID hash (expected in rosbag path or folder name)
    # vehicle_model: Vehicle model (must match vehicle_fragment)
    # vehicle_id: Vehicle ID (must match vehicle_fragment)
    # sensor_model: Sensor model (must match vehicle_fragment)
    "sample|sample_vehicle|default|sample_sensor_kit" # Autoware sample vehicle
    "0803cd66|lexus|default|aip_xx1"
    "162bd623|sample_vehicle|default|sample_sensor_kit"
    "2721c269|j6_gen1|j6_gen1_dev|aip_x2"
    "0c50398a|j6_gen1|j6_gen1_dev|aip_x2"
    "3905ebe7|cargo_transport|default|sample_sensor_kit"
    "5c684cbc|robobus|robobus_01|robobus_sensor_kit"
    "694a217d|cargo_transport|default|aip_x1"
    "948d4e44|medium_bus|default|aip_xx1"
    "b3902d62|j6_gen2|j6_gen2_12|aip_x2_gen2"
    "0388f3fc|j6_gen1|j6_gen1_02|aip_x2"
    "cfa23601|j6_gen2|j6_gen2_03|aip_x2_gen2"
    "aca0a78e|j6_gen2|j6_gen2_32|aip_x2_gen2"
    "01d060d9|jpntaxi|7|aip_xx1" # old jpntaxi 7 "default" may be appropriate
    "1d8fb5b8|jpntaxi|4|aip_xx1" # old jpntaxi 4 "default" may be appropriate
    "0ed0b1e4|jpntaxi|1|aip_xx1"
    "fc54b2c0|jpntaxi|4|aip_xx1"
    "bd44d34d|jpntaxi|7|aip_xx1"
)

detect_vehicle_config() {
    local rosbag_path="$1"
    for config in "${VEHICLE_CONFIGS[@]}"; do
        [[ -z "$config" || "$config" =~ ^[[:space:]]*# ]] && continue
        IFS='|' read -r vehicle_fragment vehicle_model_val vehicle_id_val sensor_model_val <<< "$config"
        if [[ "$rosbag_path" == *"$vehicle_fragment"* ]]; then
            echo "$vehicle_model_val|$vehicle_id_val|$sensor_model_val"
            return 0
        fi
    done
    return 1
}

# Exact match on vehicle_fragment (first column). Used by launch_autoware.sh --force-sample-vehicle, etc.
lookup_vehicle_config_by_fragment() {
    local fragment="$1"
    for config in "${VEHICLE_CONFIGS[@]}"; do
        [[ -z "$config" || "$config" =~ ^[[:space:]]*# ]] && continue
        IFS='|' read -r vehicle_fragment vehicle_model_val vehicle_id_val sensor_model_val <<< "$config"
        if [[ "$vehicle_fragment" == "$fragment" ]]; then
            echo "$vehicle_model_val|$vehicle_id_val|$sensor_model_val"
            return 0
        fi
    done
    return 1
}
