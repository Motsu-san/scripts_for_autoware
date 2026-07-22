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
