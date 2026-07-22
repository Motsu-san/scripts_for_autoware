#!/usr/bin/env bash
# ndt_mean_pose.yaml を基準に、mean_pose 集計 JSON の各 run との縦・横・ヨー誤差（平均・ばらつき）を YAML に出力する。
# per_scan_summary がある場合は各 run の pose_header_stamp_sec に最も近い参照 pose を基準に差分を計算する。
#
# dt_pose_from_pointcloud_header_sec が MAX_POSE_POINTCLOUD_DT_SEC を超える run は集計から除外する（既定: 0.21）。
# 除外なし: MAX_POSE_POINTCLOUD_DT_SEC= ./compare_mean_pose_yaml.sh ... （空で --no-pose-pointcloud-dt-filter）
#
# Usage:
#   ./compare_mean_pose_yaml.sh <ndt_mean_pose.yaml> <mean_pose.json> [--yaml-out path] [--json-out path]

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
: "${MAX_POSE_POINTCLOUD_DT_SEC:=0.21}"

CMD=(python3 "$SCRIPT_DIR/compare_mean_pose_yaml.py")
if [[ -n "$MAX_POSE_POINTCLOUD_DT_SEC" ]]; then
    CMD+=(--max-pose-pointcloud-dt-sec "$MAX_POSE_POINTCLOUD_DT_SEC")
else
    CMD+=(--no-pose-pointcloud-dt-filter)
fi
exec "${CMD[@]}" "$@"
