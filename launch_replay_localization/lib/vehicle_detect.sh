#!/bin/bash

detect_and_log_vehicle() {
    if [ "$FORCE_SAMPLE_VEHICLE" = "true" ]; then
        VEHICLE_CONFIG=$(lookup_vehicle_config_by_fragment "sample")
        if [ $? -ne 0 ] || [ -z "$VEHICLE_CONFIG" ]; then
            echo "Error: --force-sample-vehicle requires a 'sample' fragment entry in $SCRIPT_DIR/vehicle_hash_map.yaml" >&2
            exit 1
        fi
        echo "Vehicle configuration: forced Autoware sample kit (vehicle_configs fragment: sample)" | tee -a $LAUNCH_LOG_FILE
    else
        VEHICLE_CONFIG=$(detect_vehicle_config "$ROSBAG")
        if [ $? -ne 0 ]; then
            echo "Error: Vehicle configuration not found for ROSBAG path: $ROSBAG"
            echo "Available vehicle ID fragments (edit $SCRIPT_DIR/vehicle_hash_map.yaml to add):"
            for config in "${VEHICLE_CONFIGS[@]}"; do
                [[ -z "$config" || "$config" =~ ^[[:space:]]*# ]] && continue
                IFS='|' read -r vehicle_fragment vehicle_model_val vehicle_id_val sensor_model_val <<< "$config"
                echo "  $vehicle_fragment -> $vehicle_model_val, $vehicle_id_val, $sensor_model_val"
            done
            exit 1
        fi
        echo "Detected vehicle configuration (from ROSBAG path):" | tee -a $LAUNCH_LOG_FILE
    fi

    IFS='|' read -r VEHICLE_MODEL VEHICLE_ID SENSOR_MODEL <<< "$VEHICLE_CONFIG"
    export VEHICLE_ID

    echo "  VEHICLE_MODEL: $VEHICLE_MODEL" | tee -a $LAUNCH_LOG_FILE
    echo "  VEHICLE_ID: $VEHICLE_ID" | tee -a $LAUNCH_LOG_FILE
    echo "  SENSOR_MODEL: $SENSOR_MODEL" | tee -a $LAUNCH_LOG_FILE
}
