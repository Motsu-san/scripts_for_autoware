#!/bin/bash

# 使用方法: ./record_rosbag_localization_replay.sh <SAVE_DIR> [TOPIC_TYPE]
# TOPIC_TYPE: default, lidar-marker, full-sensing, pose-comparison, convergence, occlusion
# デフォルト: default

SAVE_DIR=$1
TOPIC_TYPE=${2:-default}

# SAVE_DIRが既に存在する場合は削除（ros2 bag recordが既存ディレクトリを許可しない場合があるため）
if [ -d "$SAVE_DIR" ]; then
    echo "Warning: SAVE_DIR already exists, removing: $SAVE_DIR"
    rm -rf "$SAVE_DIR"
fi

# 引数チェック
if [ -z "$SAVE_DIR" ]; then
    echo "Error: SAVE_DIR is required"
    echo "Usage: $0 <SAVE_DIR> [TOPIC_TYPE]"
    echo "  TOPIC_TYPE: default, lidar-marker, full-sensing, pose-comparison, convergence, occlusion"
    exit 1
fi

# トピックリストの選択
case "$TOPIC_TYPE" in
    "default")
        # For localization replay (Default)
        TOPICS=(
          /sensing/lidar/concatenated/pointcloud \
          /sensing/imu/imu_data \
          /vehicle/status/velocity_status \
          /sensing/gnss/pose_with_covariance \
          /tf_static
        )
        ;;
    "lidar-marker_replay")
        # For localization replay (For lidar-marker without sensing components)
        TOPICS=(
          /localization/util/downsample/pointcloud \
          /sensing/lidar/top_left/rectified/pointcloud_ex \
          /sensing/lidar/top_right/rectified/pointcloud_ex \
          /sensing/lidar/front_lower/rectified/pointcloud_ex \
          /sensing/lidar/top_left_lower/rectified/pointcloud_ex \
          /sensing/lidar/top_right_lower/rectified/pointcloud_ex \
          /sensing/imu/imu_data \
          /sensing/vehicle_velocity_converter/twist_with_covariance \
          /sensing/gnss/pose_with_covariance \
          /tf_static
        )
        ;;
    "full-sensing_replay")
        # For localization replay (with full sensing components)
        TOPICS=(
          /sensing/lidar/top/velodyne_packets \
          /sensing/lidar/top_left_lower/pandar_packets \
          /sensing/lidar/top_left/pandar_packets \
          /sensing/lidar/top_right_lower/pandar_packets \
          /sensing/lidar/top_right/pandar_packets \
          /sensing/lidar/front_upper/pandar_packets \
          /sensing/lidar/left_upper/pandar_packets \
          /sensing/lidar/right_upper/pandar_packets \
          /sensing/lidar/rear_upper/pandar_packets \
          /sensing/lidar/front_lower/pandar_packets \
          /sensing/lidar/left_lower/pandar_packets \
          /sensing/lidar/right_lower/pandar_packets \
          /sensing/lidar/rear_lower/pandar_packets \
          /sensing/lidar/front_center/pandar_packets \
          /sensing/lidar/rear_center/pandar_packets \
          /vehicle/status/steering_status \
          /vehicle/status/velocity_status \
          /sensing/imu/tamagawa/imu_raw \
          /sensing/lidar/front_center/livox/imu \
          /sensing/gnss/septentrio/nav_sat_fix \
          /sensing/gnss/ublox/nav_sat_fix \
          /tf_static
        )
        ;;
    "output")
        TOPICS=(
          /diagnostics \
          /localization/kinematic_state \
          /localization/pose_estimator/exe_time_ms \
          /localization/pose_estimator/iteration_num \
          /localization/pose_estimator/pose \
          /localization/pose_estimator/pose_with_covariance \
          /localization/pose_estimator/transform_probability \
          /localization/pose_estimator/nearest_voxel_transformation_likelihood \
          /localization/pose_estimator/initial_to_result_relative_pose \
          /localization/pose_estimator/ndt_marker \
          /localization/pose_twist_fusion_filter/pose \
          /localization/pose_twist_fusion_filter/biased_pose_with_covariance \
          /localization/pose_twist_fusion_filter/kinematic_state \
          /localization/pose_twist_fusion_filter/pose_instability_detector/debug/diff_pose
        )
        ;;
    "output_lidar-marker")
        # For pose comparison evaluation on lidar_marker_localizer
        TOPICS=(
          /diagnostics \
          /localization/kinematic_state \
          /localization/pose_estimator/exe_time_ms \
          /localization/pose_estimator/iteration_num \
          /localization/pose_estimator/pose \
          /localization/pose_estimator/pose_with_covariance \
          /localization/pose_estimator/transform_probability \
          /localization/pose_estimator/nearest_voxel_transformation_likelihood \
          /localization/pose_estimator/initial_to_result_relative_pose \
          /localization/pose_estimator/ndt_marker \
          /localization/pose_twist_fusion_filter/pose \
          /localization/pose_twist_fusion_filter/biased_pose_with_covariance \
          /localization/pose_twist_fusion_filter/kinematic_state \
          /localization/pose_twist_fusion_filter/pose_instability_detector/debug/diff_pose \
          /localization/pose_estimator/lidar_marker_localizer/top_left/lidar_marker_localizer/debug/pose_with_covariance
        )
        ;;
    "convergence")
        # For convergence evaluation / For fluctuation evaluation
        TOPICS=(
          /diagnostics \
          /localization/util/downsample/pointcloud \
          /localization/pose_estimator/pose \
          /localization/twist_estimator/twist_with_covariance \
          /localization/pose_twist_fusion_filter/biased_pose_with_covariance
        )
        ;;
    "occlusion")
        # For occlusion adding
        TOPICS=(
          /sensing/imu/tamagawa/imu_raw \
          /sensing/gnss/ublox/fix_velocity \
          /sensing/gnss/ublox/nav_sat_fix \
          /sensing/gnss/ublox/navpvt \
          /vehicle/status/velocity_status \
          /sensing/lidar/top/pointcloud_raw_ex \
          /tf_static \
          /clock \
          /localization/pose_twist_fusion_filter/biased_pose_with_covariance
        )
        ;;
    "output_localization_evaluation_scrpts")
        # For localization evaluation scripts
        # https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts
        TOPICS=(
          /localization/pose_estimator/ndt_marker \
          /localization/pose_estimator/initial_to_result_relative_pose \
          /localization/pose_estimator/pose \
          /localization/pose_estimator/pose_with_covariance \
          /localization/pose_estimator/iteration_num \
          /localization/pose_estimator/nearest_voxel_transformation_likelihood \
          /diagnostics \
          /localization/util/downsample/pointcloud \
          /system/processing_time_checker/metrics \
          /localization/pose_estimator/points_aligned \
          /map/vector_map_marker \
          /driving_log_replayer/localization/results \
          /localization/reference_kinematic_state \
          /map/pointcloud_map \
          /tf \
          /localization/kinematic_state \
          /localization/pose_estimator/transform_probability \
          /localization/acceleration \
          /localization/pose_estimator/exe_time_ms
        )
        ;;
    *)
        echo "Error: Unknown TOPIC_TYPE: $TOPIC_TYPE"
        echo "Available types: default, lidar-marker, full-sensing, pose-comparison, convergence, occlusion"
        exit 1
        ;;
esac

echo "Recording rosbag to: $SAVE_DIR"
echo "Topic type: $TOPIC_TYPE"
echo "Topics: ${TOPICS[*]}"

ros2 bag record -o "$SAVE_DIR" --use-sim-time "${TOPICS[@]}"

# NDT用 - concatenated pointcloud
# "/sensing/lidar/concatenated/pointcloud"

# lidar-marker用 - 各LiDARのrectified pointcloud_ex
# "/sensing/lidar/top/pointcloud"
# "/sensing/lidar/front_lower/rectified/pointcloud_ex"
# "/sensing/lidar/top_left_lower/rectified/pointcloud_ex"
# "/sensing/lidar/top_right_lower/rectified/pointcloud_ex"

# pose_initializer用
# "/sensing/gnss/pose_with_covariance"

# gyro_odometer用
# "/sensing/imu/imu_data"
# "/sensing/vehicle_velocity_converter/twist_with_covariance"

# TF(静的な座標変換)
# "/tf_static"
