#!/bin/bash

filter_home_underlay_paths() {
    local _input="${1:-}"
    [ -z "$_input" ] && return 0
    echo "$_input" | tr ':' '\n' | awk -v keep="$CALL_DIR/install" '
        length($0) > 0 && (index($0, keep) == 1 || index($0, ENVIRON["HOME"]) == 0) { print }
    ' | tr '\n' ':' | sed 's/:$//'
}

setup_ros_env() {
    cd "$SCRIPT_DIR"

    AMENT_PREFIX_PATH=$(filter_home_underlay_paths "$AMENT_PREFIX_PATH")
    export AMENT_PREFIX_PATH
    LD_LIBRARY_PATH=$(filter_home_underlay_paths "$LD_LIBRARY_PATH")
    export LD_LIBRARY_PATH

    source /opt/ros/humble/setup.bash
    source "$CALL_DIR/install/setup.bash"
    LD_LIBRARY_PATH=$(filter_home_underlay_paths "$LD_LIBRARY_PATH")
    export LD_LIBRARY_PATH

    local _CALL_INSTALL="$CALL_DIR/install"
    if [ -d "$_CALL_INSTALL" ]; then
        export AMENT_PREFIX_PATH="$_CALL_INSTALL${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
    fi
    unset _CALL_INSTALL

    local _T4_LOC_PREFIX="$CALL_DIR/install/tier4_localization_launch"
    if [ -d "$_T4_LOC_PREFIX/share/tier4_localization_launch" ]; then
        local _t4_new=""
        local _t4_ifs="$IFS"
        IFS=':'
        for _t4_p in $AMENT_PREFIX_PATH; do
            [ -z "$_t4_p" ] && continue
            case "$_t4_p" in
                */tier4_localization_launch) continue ;;
            esac
            _t4_new="${_t4_new:+${_t4_new}:}${_t4_p}"
        done
        IFS="$_t4_ifs"
        export AMENT_PREFIX_PATH="${_T4_LOC_PREFIX}${_t4_new:+:${_t4_new}}"
    fi
    unset _T4_LOC_PREFIX

    local _T4_MAP_PREFIX="$CALL_DIR/install/tier4_map_launch"
    if [ -d "$_T4_MAP_PREFIX/share/tier4_map_launch" ]; then
        local _t4m_new=""
        local _t4m_ifs="$IFS"
        IFS=':'
        for _t4m_p in $AMENT_PREFIX_PATH; do
            [ -z "$_t4m_p" ] && continue
            case "$_t4m_p" in
                */tier4_map_launch) continue ;;
            esac
            _t4m_new="${_t4m_new:+${_t4m_new}:}${_t4m_p}"
        done
        IFS="$_t4m_ifs"
        export AMENT_PREFIX_PATH="${_T4_MAP_PREFIX}${_t4m_new:+:${_t4m_new}}"
    fi
    unset _T4_MAP_PREFIX
}
