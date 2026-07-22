#!/bin/bash
# Usage: ./capture_rviz_display.sh [OUTPUT_DIR]
#   OUTPUT_DIR: where to save the mp4 (default: ~/Videos). Created if missing.

OUTPUT_DIR="${1:-$HOME/Videos}"
mkdir -p "$OUTPUT_DIR"
DATE=$(date +"%Y-%m-%d_%H%M%S")
OUTPUT_FILE="$OUTPUT_DIR/rviz_cap_$DATE.mp4"
# OUTPUT_FILE_OGV=$OUTPUT_DIR/$(basename $OUTPUT_FILE .mp4).ogv
# WINDOW_NAME='/home/motsu/pilot-auto.xx1/install/autoware_launch/share/autoware_launch/rviz/autoware.rviz - RViz'
# WINDOU_ID=$(xwininfo -d $DISPLAY -name "${WINDOW_NAME}" | grep 'Window id:' | awk '{print $4}')

# RDP settings
VIDEO_WIDTH=1338
VIDEO_HEIGHT=1244
LEFTUPPER_X=751
LEFTUPPER_Y=133
VIDEO_SIZE="${VIDEO_WIDTH}x${VIDEO_HEIGHT}"
FPS=10

# exec so the process becomes ffmpeg; caller can stop recording with kill -INT <PID>
exec ffmpeg -video_size $VIDEO_SIZE -framerate $FPS -f x11grab -i $DISPLAY+$LEFTUPPER_X,$LEFTUPPER_Y $OUTPUT_FILE

# WINDOU_ID=$(xwininfo -d $DISPLAY -name "${WINDOW_NAME}" | grep 'Window id:' | awk '{print $4}')

# recordmydesktop --windowid $WINDOU_ID --no-sound --no-cursor --no-frame \
#     --width ${VIDEO_WIDTH} --height ${VIDEO_HEIGHT} --fps $FPS -o $OUTPUT_FILE_OGV &
