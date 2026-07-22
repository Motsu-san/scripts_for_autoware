#!/bin/bash

launch_autoware_nodes() {
    local _LOCALIZATION_STANDALONE_XML=""
    local _CALL_DIR_LS_XML="$CALL_DIR/install/tier4_localization_launch/share/tier4_localization_launch/launch/localization_standalone.launch.xml"
    if [ -f "$_CALL_DIR_LS_XML" ]; then
        _LOCALIZATION_STANDALONE_XML="$_CALL_DIR_LS_XML"
    else
        local _loc_pkg_prefix
        _loc_pkg_prefix=$(ros2 pkg prefix tier4_localization_launch 2>/dev/null || true)
        if [ -n "$_loc_pkg_prefix" ]; then
            local _pkg_ls_xml="$_loc_pkg_prefix/share/tier4_localization_launch/launch/localization_standalone.launch.xml"
            if [ -f "$_pkg_ls_xml" ]; then
                _LOCALIZATION_STANDALONE_XML="$_pkg_ls_xml"
            fi
        fi
    fi

    if [ -n "$_LOCALIZATION_STANDALONE_XML" ]; then
        echo "Using localization standalone launch (absolute path): $_LOCALIZATION_STANDALONE_XML" | tee -a $LAUNCH_LOG_FILE
        local _LOC_STANDALONE_ARGS=(
            preset:=logging
            map_path:=$MAP_PATH
            vehicle_model:=$VEHICLE_MODEL
            sensor_model:=$SENSOR_MODEL
            gnss_receiver:=$GNSS_RECEIVER
            launch_sensing:=$LAUNCH_SENSING
            launch_api:=$LAUNCH_API
            launch_default_ad_api:=$LAUNCH_DEFAULT_AD_API
            launch_rviz_adaptors:=$LAUNCH_RVIZ_ADAPTORS
            launch_sensing_driver:=$LAUNCH_DRIVER
            use_sim_time:=$USE_SIM_TIME
            rviz:=$RVIZ
            "${_POSE_SOURCE_LAUNCH_ARG[@]}"
        )
        [ -n "$RVIZ_CONFIG" ] && _LOC_STANDALONE_ARGS+=( "rviz_config:=$RVIZ_CONFIG" )
        ros2 launch "$_LOCALIZATION_STANDALONE_XML" "${_LOC_STANDALONE_ARGS[@]}" \
            2>&1 | tee -a $LAUNCH_LOG_FILE &
        unset _LOC_STANDALONE_ARGS
    else
        echo "Info: localization_standalone.launch.xml not found; falling back to autoware_launch logging_simulator.launch.xml" | tee -a $LAUNCH_LOG_FILE
        local _LOGGING_SIM_ARGS=(
            map_path:=$MAP_PATH
            vehicle_model:=$VEHICLE_MODEL
            vehicle_id:=$VEHICLE_ID
            sensor_model:=$SENSOR_MODEL
            launch_driver:=$LAUNCH_DRIVER
            sensing:=$LAUNCH_SENSING
            localization:=$LAUNCH_LOCALIZATION
            perception:=$LAUNCH_PERCEPTION
            planning:=$LAUNCH_PLANNING
            control:=$LAUNCH_CONTROL
            use_sim_time:=$USE_SIM_TIME
            rviz:=$RVIZ
            "${_POSE_SOURCE_LAUNCH_ARG[@]}"
        )
        [ -n "$RVIZ_CONFIG" ] && _LOGGING_SIM_ARGS+=( "rviz_config:=$RVIZ_CONFIG" )
        ros2 launch autoware_launch logging_simulator.launch.xml "${_LOGGING_SIM_ARGS[@]}" \
            2>&1 | tee -a $LAUNCH_LOG_FILE &
        unset _LOGGING_SIM_ARGS
    fi
    unset _LOCALIZATION_STANDALONE_XML _CALL_DIR_LS_XML _loc_pkg_prefix _pkg_ls_xml _POSE_SOURCE_LAUNCH_ARG

    sleep 3

    echo "Calling trigger_node service..." | tee -a $LAUNCH_LOG_FILE
    ros2 service call /localization/pose_twist_fusion_filter/trigger_node std_srvs/srv/SetBool "{data: false}" 2>&1 | tee -a $LAUNCH_LOG_FILE

    sleep 3

    if [ -f "$INITIAL_POSE_YAML" ] && initial_pose_skip_localization "$INITIAL_POSE_YAML"; then
        echo "Info: skip_initial_localization=true in $INITIAL_POSE_YAML -> stopping pose_initializer nodes before rosbag play" | tee -a $LAUNCH_LOG_FILE
        stop_pose_initializer_nodes
    fi
}
