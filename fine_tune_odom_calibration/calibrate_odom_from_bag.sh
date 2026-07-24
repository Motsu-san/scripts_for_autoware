#!/usr/bin/env bash
# mean_ndt_pose.yaml を基準に、オドメトリのみ走行 rosbag から
# speed_scale_factor / yaw_rate バイアス補正を推定する。
#
# 事前に Autoware WS を source することを推奨(velocity_status デシリアライズ用):
#   source /opt/ros/humble/setup.bash
#   source ~/autoware/install/setup.bash
#   --individual-params-root ~/pilot-auto.../src/autoware/individual_params
#   または --vehicle-velocity-param-yaml / --imu-corrector-param-yaml を直接指定
#
# Usage:
#   ./calibrate_odom_from_bag.sh <mean_ndt_pose.yaml> <odom_rosbag> \
#     --initial-pose-yaml <initial_pose.yaml> [options]

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/calibrate_odom_from_bag.py" "$@"
