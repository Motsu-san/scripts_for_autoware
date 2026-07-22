#!/bin/bash

setup_rviz_display() {
    if [ "${LAUNCH_RVIZ:-true}" != "true" ]; then
        return 0
    fi
    local _log="${LAUNCH_LOG_FILE:-/dev/stderr}"
    local _xauth="${XAUTHORITY:-$HOME/.Xauthority}"
    local _d

    if [ -n "${DISPLAY:-}" ] && xdpyinfo >/dev/null 2>&1; then
        export XAUTHORITY="${XAUTHORITY:-$_xauth}"
        return 0
    fi

    if [ -n "${DISPLAY:-}" ]; then
        echo "Warning: DISPLAY=$DISPLAY is not reachable; trying auto-detect (:10 xrdp, :20 Xvfb)..." | tee -a "$_log"
    else
        echo "DISPLAY is unset; auto-detecting for RViz (:10 xrdp, :20 Xvfb)..." | tee -a "$_log"
    fi

    for _d in ":10" ":20"; do
        if XAUTHORITY="$_xauth" DISPLAY="$_d" xdpyinfo >/dev/null 2>&1; then
            export DISPLAY="$_d"
            export XAUTHORITY="$_xauth"
            echo "RViz display: DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY" | tee -a "$_log"
            if [ "$_d" = ":10" ]; then
                echo "  (xrdp desktop — open Remote Desktop to see RViz windows)" | tee -a "$_log"
            elif [ "$_d" = ":20" ]; then
                echo "  (Xvfb — use VNC or x11vnc on :20 if you need to view the window)" | tee -a "$_log"
            fi
            return 0
        fi
    done

    echo "Error: RViz is enabled but no X display is available." | tee -a "$_log"
    echo "  Connect via xrdp, or: export DISPLAY=:10 XAUTHORITY=\$HOME/.Xauthority" | tee -a "$_log"
    echo "  Or disable RViz: LAUNCH_RVIZ=false $LAUNCH_SCRIPT_NAME ..." | tee -a "$_log"
    return 1
}
