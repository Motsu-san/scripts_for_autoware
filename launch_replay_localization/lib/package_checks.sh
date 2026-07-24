#!/bin/bash

check_required_packages() {
    if [ "${LAUNCH_VEHICLE:-true}" = "true" ] && [ "$SENSOR_MODEL" = "aip_xx1" ]; then
        local _need=(velodyne_description vls_description livox_description camera_description imu_description aip_xx1_description)
        if [ "$VEHICLE_MODEL" = "jpntaxi" ]; then
            _need+=(jpntaxi_description)
        fi
        local _missing=()
        local _p
        for _p in "${_need[@]}"; do
            if ! ros2 pkg prefix "$_p" &>/dev/null; then
                _missing+=("$_p")
            fi
        done
        if [ ${#_missing[@]} -gt 0 ]; then
            echo "Error: vehicle URDF 用パッケージが未インストール: ${_missing[*]}" | tee -a $LAUNCH_LOG_FILE >&2
            echo "  対処1) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-up-to aip_xx1_description" | tee -a $LAUNCH_LOG_FILE >&2
            echo "  対処2) LAUNCH_VEHICLE=false $LAUNCH_SCRIPT_NAME <MAP> <BAG> ...  (TF は bag 頼み)" | tee -a $LAUNCH_LOG_FILE >&2
            exit 1
        fi
    fi

    if [ "${LAUNCH_LOCALIZATION:-true}" = "true" ]; then
        local _loc_pkg_groups=(
            "autoware_ekf_localizer ekf_localizer"
            "autoware_stop_filter stop_filter"
            "autoware_twist2accel twist2accel"
            "autoware_pose_instability_detector pose_instability_detector"
            "autoware_localization_error_monitor localization_error_monitor"
        )
        local _loc_miss=()
        local _g _p _loc_ok
        for _g in "${_loc_pkg_groups[@]}"; do
            _loc_ok=0
            for _p in $_g; do
                if ros2 pkg prefix "$_p" &>/dev/null; then
                    _loc_ok=1
                    break
                fi
            done
            if [ "$_loc_ok" -eq 0 ]; then
                _loc_miss+=("($_g)")
            fi
        done
        unset _loc_ok
        if [ ${#_loc_miss[@]} -ne 0 ]; then
            echo "Error: ローカリゼーション用パッケージが未インストール(いずれかの名前で存在すること): ${_loc_miss[*]}" | tee -a $LAUNCH_LOG_FILE >&2
            echo "  対処) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-up-to tier4_localization_launch" | tee -a $LAUNCH_LOG_FILE >&2
            echo "  または Autoware underlay を source した上で再実行: source /path/to/autoware/install/setup.bash" | tee -a $LAUNCH_LOG_FILE >&2
            exit 1
        fi
    fi

    if [ "${LAUNCH_SENSING:-true}" = "true" ] && [ "$SENSOR_MODEL" = "aip_xx1" ]; then
        if ! ros2 pkg prefix pe_ars408_ros &>/dev/null; then
            echo "Error: pe_ars408_ros(Continental ARS408)が未インストール。aip の radar launch に必須。" | tee -a $LAUNCH_LOG_FILE >&2
            echo "  対処) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-select pe_ars408_ros" | tee -a $LAUNCH_LOG_FILE >&2
            echo "  緩和) LAUNCH_SENSING=false $LAUNCH_SCRIPT_NAME ...  (bag に点群等があればローカライゼーションは進む場合あり)" | tee -a $LAUNCH_LOG_FILE >&2
            exit 1
        fi
    fi

    if [ "${LAUNCH_API:-true}" = "true" ]; then
        local _api_pkg_groups=(
            "autoware_default_adapi default_ad_api"
        )
        if [ "${LAUNCH_RVIZ_ADAPTORS:-true}" = "true" ]; then
            _api_pkg_groups+=("autoware_adapi_adaptors ad_api_adaptors")
        fi
        local _api_miss=()
        local _api_ok
        for _g in "${_api_pkg_groups[@]}"; do
            _api_ok=0
            for _p in $_g; do
                if ros2 pkg prefix "$_p" &>/dev/null; then
                    _api_ok=1
                    break
                fi
            done
            if [ "$_api_ok" -eq 0 ]; then
                _api_miss+=("($_g)")
            fi
        done
        unset _api_ok
        if [ ${#_api_miss[@]} -gt 0 ]; then
            echo "Error: API 用パッケージが未インストール(いずれかの名前で存在すること): ${_api_miss[*]}" | tee -a $LAUNCH_LOG_FILE >&2
            echo "  対処) cd $CALL_DIR && source /opt/ros/humble/setup.bash && colcon build --packages-up-to tier4_autoware_api_launch" | tee -a $LAUNCH_LOG_FILE >&2
            exit 1
        fi
    fi
}
