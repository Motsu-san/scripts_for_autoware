#!/bin/bash
# Mapping of vehicle ID fragment to vehicle_model / vehicle_id / sensor_model.
# Sourced by launch_autoware.sh and launch_unified_localization.sh.
#
# The data is managed in vehicle_hash_map.yaml (edit that file to add vehicles).
# This script loads it into VEHICLE_CONFIGS as "fragment|model|id|sensor" lines
# via py/parse_vehicle_configs.py.

VEHICLE_CONFIGS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VEHICLE_HASH_MAP_YAML="$VEHICLE_CONFIGS_DIR/vehicle_hash_map.yaml"
VEHICLE_CONFIGS_PARSER="$VEHICLE_CONFIGS_DIR/py/parse_vehicle_configs.py"

declare -a VEHICLE_CONFIGS=()
_vc_output="$(python3 "$VEHICLE_CONFIGS_PARSER" "$VEHICLE_HASH_MAP_YAML")"
if [ $? -ne 0 ]; then
    echo "Error: failed to load vehicle configs from $VEHICLE_HASH_MAP_YAML" >&2
    return 1 2>/dev/null || exit 1
fi
mapfile -t VEHICLE_CONFIGS <<< "$_vc_output"
unset _vc_output

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
