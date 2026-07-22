#!/usr/bin/env python3

# Copyright 2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Localizationの位置姿勢の差を比較するスクリプト

使用方法:
    python3 compare_localization_diff.py \
        --target_bag /path/to/target_bag \
        --reference_bag /path/to/reference_bag \
        --output_dir ./output
"""

import argparse
import math
import os
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# 日本語フォントの設定
try:
    # 利用可能な日本語フォントを順に試す
    japanese_fonts = [
        'Noto Sans CJK JP',
        'Noto Sans CJK',
        'Takao',
        'TakaoGothic',
        'IPAexGothic',
        'IPAPGothic',
        'VL Gothic',
        'Yu Gothic',
    ]

    font_found = False
    for font_name in japanese_fonts:
        try:
            matplotlib.rcParams['font.family'] = font_name
            # フォントが利用可能かテスト
            test_fig = plt.figure(figsize=(1, 1))
            test_ax = test_fig.add_subplot(111)
            test_ax.text(0.5, 0.5, 'テスト', fontfamily=font_name)
            plt.close(test_fig)
            font_found = True
            print(f"Japanese font set: {font_name}")
            break
        except:
            continue

    if not font_found:
        # フォントが見つからない場合、システムのデフォルトフォントリストから日本語フォントを探す
        import matplotlib.font_manager as fm
        available_fonts = [f.name for f in fm.fontManager.ttflist]
        for font_name in japanese_fonts:
            if font_name in available_fonts:
                matplotlib.rcParams['font.family'] = font_name
                print(f"Japanese font set: {font_name}")
                font_found = True
                break

        if not font_found:
            print("Warning: Japanese font not found. Japanese characters may not display correctly.")
            print(f"Available fonts: {', '.join(set(available_fonts[:10]))}")
except Exception as e:
    print(f"Warning: Error occurred while setting font: {e}")
    print("Japanese characters may not display correctly.")

# matplotlibの日本語表示設定
matplotlib.rcParams['axes.unicode_minus'] = False  # マイナス記号の文字化けを防ぐ


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """
    QuaternionをEuler角(roll, pitch, yaw)に変換

    Args:
        qx, qy, qz, qw: Quaternionの成分

    Returns:
        (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def find_rosbag2_directory(bag_path: str) -> str:
    """
    rosbag2ディレクトリを探す
    指定されたパスがrosbag2ディレクトリか、その中にrosbag2ディレクトリがあるかを確認

    Args:
        bag_path: rosbag2のパスまたはディレクトリ

    Returns:
        rosbag2ディレクトリのパス
    """
    bag_path_obj = Path(bag_path)

    # 指定されたパスがrosbag2ディレクトリか確認（metadata.yamlが存在する）
    if bag_path_obj.is_dir() and (bag_path_obj / "metadata.yaml").exists():
        return str(bag_path_obj)

    # ディレクトリ内にrosbag2ディレクトリを探す（最新のものを選択）
    if bag_path_obj.is_dir():
        # サブディレクトリを探す
        rosbag2_dirs = [
            d for d in bag_path_obj.iterdir()
            if d.is_dir() and (d / "metadata.yaml").exists()
        ]

        if rosbag2_dirs:
            # 最新のディレクトリを選択（名前でソート）
            latest_dir = sorted(rosbag2_dirs, key=lambda x: x.name, reverse=True)[0]
            return str(latest_dir)

    # 見つからない場合は元のパスを返す
    return str(bag_path_obj)


def read_topic_timestamps(bag_path: str, topic_name: str) -> List[int]:
    """
    rosbag2から指定されたトピックのタイムスタンプを読み込む

    Args:
        bag_path: rosbag2のパス
        topic_name: トピック名

    Returns:
        [timestamp_ns, ...] のリスト（トピックが存在しない場合は空のリスト）
    """
    # rosbag2ディレクトリを探す
    actual_bag_path = find_rosbag2_directory(bag_path)

    storage_options = StorageOptions(uri=actual_bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # トピックが存在するかチェック
    if topic_name not in type_map:
        # トピックが存在しない場合は空のリストを返す
        return []

    timestamps = []

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == topic_name:
            timestamps.append(timestamp)

    return timestamps


def find_continuous_intervals(timestamps: List[int], max_gap_ns: int = 180000000) -> List[Tuple[int, int]]:
    """
    タイムスタンプのリストから連続区間を検出

    Args:
        timestamps: タイムスタンプのリスト（nanoseconds）
        max_gap_ns: 連続とみなす最大の間隔（nanoseconds、デフォルト: 180ms）

    Returns:
        [(start_timestamp_ns, end_timestamp_ns), ...] のリスト
    """
    if len(timestamps) == 0:
        return []

    intervals = []
    sorted_timestamps = sorted(timestamps)

    start_ts = sorted_timestamps[0]
    prev_ts = sorted_timestamps[0]

    for ts in sorted_timestamps[1:]:
        gap = ts - prev_ts
        if gap > max_gap_ns:
            # 不連続を検出
            intervals.append((start_ts, prev_ts))
            start_ts = ts
        prev_ts = ts

    # 最後の区間を追加
    intervals.append((start_ts, sorted_timestamps[-1]))

    return intervals


def read_kinematic_state_from_bag(bag_path: str, topic_name: str = "/localization/kinematic_state") -> List[Tuple[float, dict]]:
    """
    rosbag2から/localization/kinematic_stateトピックを読み込む

    Args:
        bag_path: rosbag2のパス
        topic_name: トピック名

    Returns:
        [(timestamp_ns, pose_data), ...] のリスト
        pose_dataは {'x', 'y', 'z', 'roll', 'pitch', 'yaw'} を含む辞書
    """
    # rosbag2ディレクトリを探す
    actual_bag_path = find_rosbag2_directory(bag_path)
    if actual_bag_path != bag_path:
        print(f"  Note: Detected rosbag2 directory: {actual_bag_path}")

    storage_options = StorageOptions(uri=actual_bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    data_list = []

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == topic_name:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            # 位置情報を取得
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            # 姿勢情報を取得 (quaternion -> euler)
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

            pose_data = {
                'x': x,
                'y': y,
                'z': z,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            }

            data_list.append((timestamp, pose_data))

    # SequentialReaderにはclose()メソッドがないため、削除
    # Pythonのガベージコレクションで自動的にクリーンアップされる
    return data_list


def read_diff_pose_from_bag(bag_path: str, topic_name: str = "/localization/pose_twist_fusion_filter/pose_instability_detector/debug/diff_pose") -> List[Tuple[float, dict]]:
    """
    rosbag2からdiff_poseトピックを読み込む

    Args:
        bag_path: rosbag2のパス
        topic_name: トピック名

    Returns:
        [(timestamp_ns, pose_data), ...] のリスト
        pose_dataは {'x', 'y', 'z', 'roll', 'pitch', 'yaw'} を含む辞書
    """
    # rosbag2ディレクトリを探す
    actual_bag_path = find_rosbag2_directory(bag_path)
    if actual_bag_path != bag_path:
        print(f"  Note: Detected rosbag2 directory: {actual_bag_path}")

    storage_options = StorageOptions(uri=actual_bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # トピックが存在するかチェック
    if topic_name not in type_map:
        print(f"  Warning: Topic {topic_name} not found in bag")
        return []

    data_list = []

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == topic_name:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            # 位置情報を取得
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z

            # 姿勢情報を取得 (quaternion -> euler)
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w

            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

            pose_data = {
                'x': x,
                'y': y,
                'z': z,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            }

            data_list.append((timestamp, pose_data))

    return data_list


def interpolate_pose(data_list: List[Tuple[float, dict]], target_timestamp: int) -> dict:
    """
    タイムスタンプで補間してposeを取得

    Args:
        data_list: [(timestamp_ns, pose_data), ...] のリスト
        target_timestamp: 補間したいタイムスタンプ (nanoseconds)

    Returns:
        補間されたpose_data
    """
    if len(data_list) == 0:
        raise ValueError("data_list is empty")

    if target_timestamp <= data_list[0][0]:
        return data_list[0][1].copy()
    if target_timestamp >= data_list[-1][0]:
        return data_list[-1][1].copy()

    # 二分探索で補間位置を探す
    left = 0
    right = len(data_list) - 1

    while right - left > 1:
        mid = (left + right) // 2
        if data_list[mid][0] < target_timestamp:
            left = mid
        else:
            right = mid

    t1, pose1 = data_list[left]
    t2, pose2 = data_list[right]

    if t2 == t1:
        return pose1.copy()

    # 線形補間
    alpha = (target_timestamp - t1) / (t2 - t1)

    interpolated = {}
    for key in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        # roll, pitch, yawの角度補間は注意が必要
        if key in ['roll', 'pitch', 'yaw']:
            # 角度の差分を計算（-πからπの範囲に正規化）
            diff = pose2[key] - pose1[key]
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            interpolated[key] = pose1[key] + alpha * diff
        else:
            interpolated[key] = pose1[key] + alpha * (pose2[key] - pose1[key])

    return interpolated


def calculate_differences(
    reference_data: List[Tuple[float, dict]],
    target_data: List[Tuple[float, dict]],
    use_vehicle_frame: bool = True
) -> Tuple[List[Tuple[float, dict]], List[Tuple[float, dict]], List[Tuple[float, dict]]]:
    """
    比較基準データと比較対象データの差分を計算

    車両座標系での差分を計算する場合:
    - 比較基準データの姿勢を基準として、比較対象データの位置を車両座標系に変換
    - 位置差分: 比較基準データの姿勢で回転させた後の差分（前後/左右/上下方向）
    - 姿勢差分: 比較基準データの姿勢を基準とした相対姿勢

    Args:
        reference_data: 比較基準となるデータ [(timestamp_ns, pose_data), ...]
        target_data: 比較対象のデータ [(timestamp_ns, pose_data), ...]
        use_vehicle_frame: Trueの場合、車両座標系で計算。Falseの場合、グローバル座標系で計算

    Returns:
        (diff_list, reference_pose_list, target_pose_list) のタプル
        - diff_list: [(timestamp_ns, diff_data), ...] のリスト
          diff_dataは {'x', 'y', 'z', 'roll', 'pitch', 'yaw'} の差分を含む
        - reference_pose_list: [(timestamp_ns, pose_data), ...] のリスト（比較基準データの位置姿勢）
        - target_pose_list: [(timestamp_ns, pose_data), ...] のリスト（比較対象データの位置姿勢）
    """
    # 共通のタイムスタンプ範囲を取得
    reference_start = reference_data[0][0]
    reference_end = reference_data[-1][0]
    target_start = target_data[0][0]
    target_end = target_data[-1][0]

    start_time = max(reference_start, target_start)
    end_time = min(reference_end, target_end)

    if start_time >= end_time:
        raise ValueError("No overlapping time range between bags")

    # 比較基準データのタイムスタンプに合わせて差分を計算
    diff_list = []
    reference_pose_list = []
    target_pose_list = []
    for timestamp, reference_pose in reference_data:
        if timestamp < start_time or timestamp > end_time:
            continue

        # 比較対象データから同じタイムスタンプのposeを補間
        target_pose = interpolate_pose(target_data, timestamp)

        if use_vehicle_frame:
            # 車両座標系での差分計算
            # 比較基準データの姿勢から回転行列を計算
            reference_rotation = Rotation.from_euler('xyz', [
                reference_pose['roll'],
                reference_pose['pitch'],
                reference_pose['yaw']
            ])
            reference_rotation_matrix = reference_rotation.as_matrix()

            # 比較対象データの位置を比較基準データの位置で平行移動
            pos_diff_global = np.array([
                target_pose['x'] - reference_pose['x'],
                target_pose['y'] - reference_pose['y'],
                target_pose['z'] - reference_pose['z']
            ])

            # 比較基準データの姿勢で回転させて車両座標系に変換
            # 回転行列の転置（逆回転）を適用
            pos_diff_vehicle = reference_rotation_matrix.T @ pos_diff_global

            # 姿勢差分: 比較対象データの姿勢を比較基準データの姿勢で回転
            target_rotation = Rotation.from_euler('xyz', [
                target_pose['roll'],
                target_pose['pitch'],
                target_pose['yaw']
            ])
            # 相対回転: reference_rotation^-1 * target_rotation
            relative_rotation = reference_rotation.inv() * target_rotation
            relative_euler = relative_rotation.as_euler('xyz')

            diff_data = {
                'x': pos_diff_vehicle[0],  # 前後方向（x軸）
                'y': pos_diff_vehicle[1],  # 左右方向（y軸）
                'z': pos_diff_vehicle[2],  # 上下方向（z軸）
                'roll': relative_euler[0],
                'pitch': relative_euler[1],
                'yaw': relative_euler[2]
            }
        else:
            # グローバル座標系での差分計算（従来の方法）
            diff_data = {
                'x': target_pose['x'] - reference_pose['x'],
                'y': target_pose['y'] - reference_pose['y'],
                'z': target_pose['z'] - reference_pose['z'],
                'roll': target_pose['roll'] - reference_pose['roll'],
                'pitch': target_pose['pitch'] - reference_pose['pitch'],
                'yaw': target_pose['yaw'] - reference_pose['yaw']
            }

        # roll, pitch, yawの差分を-πからπの範囲に正規化
        for angle_key in ['roll', 'pitch', 'yaw']:
            while diff_data[angle_key] > math.pi:
                diff_data[angle_key] -= 2 * math.pi
            while diff_data[angle_key] < -math.pi:
                diff_data[angle_key] += 2 * math.pi

        diff_list.append((timestamp, diff_data))
        # 地図座標系での位置姿勢も保存
        reference_pose_list.append((timestamp, reference_pose.copy()))
        target_pose_list.append((timestamp, target_pose.copy()))

    return diff_list, reference_pose_list, target_pose_list


def plot_differences(
    diff_list: List[Tuple[float, dict]],
    reference_pose_list: List[Tuple[float, dict]],
    target_pose_list: List[Tuple[float, dict]],
    output_dir: str,
    reference_name: str,
    target_name: str,
    reference_path: str = None,
    target_path: str = None
):
    """
    差分データをグラフ表示

    Args:
        diff_list: [(timestamp_ns, diff_data), ...] のリスト
        reference_pose_list: [(timestamp_ns, pose_data), ...] のリスト（比較基準データの位置姿勢）
        target_pose_list: [(timestamp_ns, pose_data), ...] のリスト（比較対象データの位置姿勢）
        output_dir: 出力ディレクトリ
        reference_name: 比較基準bagの名前
        target_name: 比較対象bagの名前
        reference_path: 比較基準bagのパス（lidar_marker_localizerの連続区間検出用）
        target_path: 比較対象bagのパス（lidar_marker_localizerの連続区間検出用）
    """
    if len(diff_list) == 0:
        print("Warning: No difference data available")
        return

    # タイムスタンプを秒に変換（最初のタイムスタンプを0秒とする）
    start_time = diff_list[0][0]
    times = [(ts - start_time) / 1e9 for ts, _ in diff_list]  # nanoseconds to seconds

    # lidar_marker_localizerの連続出力区間を検出（referenceがlidar_marker_localizerありの場合）
    lidar_marker_intervals = []
    if reference_path:
        debug_topic = "/localization/pose_estimator/lidar_marker_localizer/top_left/lidar_marker_localizer/debug/pose_with_covariance"
        try:
            print(f"Detecting continuous intervals for {debug_topic}...")
            timestamps = read_topic_timestamps(reference_path, debug_topic)
            if len(timestamps) > 0:
                # 180ms = 180000000 nanoseconds
                intervals = find_continuous_intervals(timestamps, max_gap_ns=180000000)
                # タイムスタンプを秒に変換（start_timeを基準に）
                for start_ts, end_ts in intervals:
                    start_sec = (start_ts - start_time) / 1e9
                    end_sec = (end_ts - start_time) / 1e9
                    # グラフの時間範囲内の区間のみ追加
                    if end_sec >= times[0] and start_sec <= times[-1]:
                        lidar_marker_intervals.append((max(start_sec, times[0]), min(end_sec, times[-1])))
                if len(lidar_marker_intervals) > 0:
                    print(f"  Found {len(intervals)} continuous intervals ({len(lidar_marker_intervals)} within graph time range)")
                else:
                    print(f"  Found {len(intervals)} continuous intervals, but none within graph time range")
            else:
                print(f"  No messages found for {debug_topic}")
                print(f"  (Topic may not exist in bag or bag may not contain lidar_marker_localizer data)")
                print(f"  Continuing without lidar_marker_localizer interval highlighting")
        except Exception as e:
            print(f"  Warning: Failed to detect intervals: {e}")
            print(f"  Continuing without lidar_marker_localizer interval highlighting")

    # データを抽出
    x_diff = [d['x'] for _, d in diff_list]
    y_diff = [d['y'] for _, d in diff_list]
    z_diff = [d['z'] for _, d in diff_list]
    roll_diff = [math.degrees(d['roll']) for _, d in diff_list]  # radians to degrees
    pitch_diff = [math.degrees(d['pitch']) for _, d in diff_list]
    yaw_diff = [math.degrees(d['yaw']) for _, d in diff_list]

    # グラフを作成
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f'Localization Difference Comparison\nReference: {reference_name} vs Target: {target_name}', fontsize=14)

    # 位置の差分（車両座標系）
    axes[0, 0].plot(times, x_diff, label='x', linewidth=1.5)
    # lidar_marker_localizerの連続区間を網掛け表示
    for start_sec, end_sec in lidar_marker_intervals:
        axes[0, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green', label='lidar_marker_active' if start_sec == lidar_marker_intervals[0][0] else '')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Position Difference [m]')
    axes[0, 0].set_title('X-axis Position Difference (Longitudinal)')
    axes[0, 0].grid(True)
    axes[0, 0].legend()

    axes[0, 1].plot(times, y_diff, label='y', linewidth=1.5, color='orange')
    for start_sec, end_sec in lidar_marker_intervals:
        axes[0, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Position Difference [m]')
    axes[0, 1].set_title('Y-axis Position Difference (Lateral)')
    axes[0, 1].grid(True)
    axes[0, 1].legend()

    axes[1, 0].plot(times, z_diff, label='z', linewidth=1.5, color='green')
    for start_sec, end_sec in lidar_marker_intervals:
        axes[1, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Position Difference [m]')
    axes[1, 0].set_title('Z-axis Position Difference (Vertical)')
    axes[1, 0].grid(True)
    axes[1, 0].legend()

    # 姿勢の差分
    axes[1, 1].plot(times, roll_diff, label='roll', linewidth=1.5, color='red')
    for start_sec, end_sec in lidar_marker_intervals:
        axes[1, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Orientation Difference [deg]')
    axes[1, 1].set_title('Roll Orientation Difference')
    axes[1, 1].grid(True)
    axes[1, 1].legend()

    axes[2, 0].plot(times, pitch_diff, label='pitch', linewidth=1.5, color='purple')
    for start_sec, end_sec in lidar_marker_intervals:
        axes[2, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_ylabel('Orientation Difference [deg]')
    axes[2, 0].set_title('Pitch Orientation Difference')
    axes[2, 0].grid(True)
    axes[2, 0].legend()

    axes[2, 1].plot(times, yaw_diff, label='yaw', linewidth=1.5, color='brown')
    for start_sec, end_sec in lidar_marker_intervals:
        axes[2, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[2, 1].set_xlabel('Time [s]')
    axes[2, 1].set_ylabel('Orientation Difference [deg]')
    axes[2, 1].set_title('Yaw Orientation Difference')
    axes[2, 1].grid(True)
    axes[2, 1].legend()

    plt.tight_layout()

    # 差分グラフを保存
    output_path = Path(output_dir) / 'localization_diff_comparison.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Difference graph saved: {output_path}")

    # 地図座標系での位置姿勢値のグラフを作成
    plt.close(fig)  # 前のグラフを閉じる

    # 比較基準データと比較対象データのタイムスタンプを秒に変換
    reference_start_time = reference_pose_list[0][0]
    reference_times = [(ts - reference_start_time) / 1e9 for ts, _ in reference_pose_list]

    target_start_time = target_pose_list[0][0]
    target_times = [(ts - target_start_time) / 1e9 for ts, _ in target_pose_list]

    # データを抽出
    reference_x = [p['x'] for _, p in reference_pose_list]
    reference_y = [p['y'] for _, p in reference_pose_list]
    reference_z = [p['z'] for _, p in reference_pose_list]
    reference_roll = [math.degrees(p['roll']) for _, p in reference_pose_list]
    reference_pitch = [math.degrees(p['pitch']) for _, p in reference_pose_list]
    reference_yaw = [math.degrees(p['yaw']) for _, p in reference_pose_list]

    target_x = [p['x'] for _, p in target_pose_list]
    target_y = [p['y'] for _, p in target_pose_list]
    target_z = [p['z'] for _, p in target_pose_list]
    target_roll = [math.degrees(p['roll']) for _, p in target_pose_list]
    target_pitch = [math.degrees(p['pitch']) for _, p in target_pose_list]
    target_yaw = [math.degrees(p['yaw']) for _, p in target_pose_list]

    # 位置姿勢値のグラフを作成
    fig2, axes2 = plt.subplots(3, 2, figsize=(14, 10))
    fig2.suptitle(f'Localization Pose Comparison (Map Frame)\nReference: {reference_name} vs Target: {target_name}', fontsize=14)

    # lidar_marker_localizerの連続区間を地図座標系のグラフにも適用（reference_start_timeを基準に変換）
    lidar_marker_intervals_map = []
    if reference_path and len(lidar_marker_intervals) > 0:
        for start_sec, end_sec in lidar_marker_intervals:
            # diff_listのstart_timeを基準にしているので、reference_start_timeに変換
            start_ts_map = reference_start_time + int(start_sec * 1e9)
            end_ts_map = reference_start_time + int(end_sec * 1e9)
            start_sec_map = (start_ts_map - reference_start_time) / 1e9
            end_sec_map = (end_ts_map - reference_start_time) / 1e9
            lidar_marker_intervals_map.append((start_sec_map, end_sec_map))

    # 位置の比較
    axes2[0, 0].plot(reference_times, reference_x, label=f'{reference_name} (reference)', linewidth=1.5, color='blue')
    axes2[0, 0].plot(target_times, target_x, label=f'{target_name} (target)', linewidth=1.5, color='red', linestyle='--')
    for start_sec, end_sec in lidar_marker_intervals_map:
        axes2[0, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green', label='lidar_marker_active' if start_sec == lidar_marker_intervals_map[0][0] else '')
    axes2[0, 0].set_xlabel('Time [s]')
    axes2[0, 0].set_ylabel('Position X [m]')
    axes2[0, 0].set_title('X Position (Map Frame)')
    axes2[0, 0].grid(True)
    axes2[0, 0].legend()

    axes2[0, 1].plot(reference_times, reference_y, label=f'{reference_name} (reference)', linewidth=1.5, color='blue')
    axes2[0, 1].plot(target_times, target_y, label=f'{target_name} (target)', linewidth=1.5, color='red', linestyle='--')
    for start_sec, end_sec in lidar_marker_intervals_map:
        axes2[0, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes2[0, 1].set_xlabel('Time [s]')
    axes2[0, 1].set_ylabel('Position Y [m]')
    axes2[0, 1].set_title('Y Position (Map Frame)')
    axes2[0, 1].grid(True)
    axes2[0, 1].legend()

    axes2[1, 0].plot(reference_times, reference_z, label=f'{reference_name} (reference)', linewidth=1.5, color='blue')
    axes2[1, 0].plot(target_times, target_z, label=f'{target_name} (target)', linewidth=1.5, color='red', linestyle='--')
    for start_sec, end_sec in lidar_marker_intervals_map:
        axes2[1, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes2[1, 0].set_xlabel('Time [s]')
    axes2[1, 0].set_ylabel('Position Z [m]')
    axes2[1, 0].set_title('Z Position (Map Frame)')
    axes2[1, 0].grid(True)
    axes2[1, 0].legend()

    # 姿勢の比較
    axes2[1, 1].plot(reference_times, reference_roll, label=f'{reference_name} (reference)', linewidth=1.5, color='blue')
    axes2[1, 1].plot(target_times, target_roll, label=f'{target_name} (target)', linewidth=1.5, color='red', linestyle='--')
    for start_sec, end_sec in lidar_marker_intervals_map:
        axes2[1, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes2[1, 1].set_xlabel('Time [s]')
    axes2[1, 1].set_ylabel('Roll [deg]')
    axes2[1, 1].set_title('Roll Orientation (Map Frame)')
    axes2[1, 1].grid(True)
    axes2[1, 1].legend()

    axes2[2, 0].plot(reference_times, reference_pitch, label=f'{reference_name} (reference)', linewidth=1.5, color='blue')
    axes2[2, 0].plot(target_times, target_pitch, label=f'{target_name} (target)', linewidth=1.5, color='red', linestyle='--')
    for start_sec, end_sec in lidar_marker_intervals_map:
        axes2[2, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes2[2, 0].set_xlabel('Time [s]')
    axes2[2, 0].set_ylabel('Pitch [deg]')
    axes2[2, 0].set_title('Pitch Orientation (Map Frame)')
    axes2[2, 0].grid(True)
    axes2[2, 0].legend()

    axes2[2, 1].plot(reference_times, reference_yaw, label=f'{reference_name} (reference)', linewidth=1.5, color='blue')
    axes2[2, 1].plot(target_times, target_yaw, label=f'{target_name} (target)', linewidth=1.5, color='red', linestyle='--')
    for start_sec, end_sec in lidar_marker_intervals_map:
        axes2[2, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes2[2, 1].set_xlabel('Time [s]')
    axes2[2, 1].set_ylabel('Yaw [deg]')
    axes2[2, 1].set_title('Yaw Orientation (Map Frame)')
    axes2[2, 1].grid(True)
    axes2[2, 1].legend()

    plt.tight_layout()

    # 位置姿勢値のグラフを保存
    output_pose_path = Path(output_dir) / 'localization_pose_comparison.png'
    plt.savefig(output_pose_path, dpi=300, bbox_inches='tight')
    print(f"Pose comparison graph saved: {output_pose_path}")
    plt.close(fig2)

    # 統計情報を表示
    print("\n=== Statistics ===")
    print(f"Number of data points: {len(diff_list)}")
    print(f"Time range: {times[0]:.2f}s ～ {times[-1]:.2f}s")
    print("\nPosition Difference [m]:")
    print(f"  X: Mean={np.mean(x_diff):.4f}, Std={np.std(x_diff):.4f}, "
          f"Max={np.max(np.abs(x_diff)):.4f}")
    print(f"  Y: Mean={np.mean(y_diff):.4f}, Std={np.std(y_diff):.4f}, "
          f"Max={np.max(np.abs(y_diff)):.4f}")
    print(f"  Z: Mean={np.mean(z_diff):.4f}, Std={np.std(z_diff):.4f}, "
          f"Max={np.max(np.abs(z_diff)):.4f}")
    print("\nOrientation Difference [deg]:")
    print(f"  Roll: Mean={np.mean(roll_diff):.4f}, Std={np.std(roll_diff):.4f}, "
          f"Max={np.max(np.abs(roll_diff)):.4f}")
    print(f"  Pitch: Mean={np.mean(pitch_diff):.4f}, Std={np.std(pitch_diff):.4f}, "
          f"Max={np.max(np.abs(pitch_diff)):.4f}")
    print(f"  Yaw: Mean={np.mean(yaw_diff):.4f}, Std={np.std(yaw_diff):.4f}, "
          f"Max={np.max(np.abs(yaw_diff)):.4f}")

    # CSVファイルにも保存（差分データ）
    csv_path = Path(output_dir) / 'localization_diff_data.csv'
    with open(csv_path, 'w') as f:
        f.write("time[s],x_diff[m],y_diff[m],z_diff[m],roll_diff[deg],pitch_diff[deg],yaw_diff[deg]\n")
        for i, (ts, diff) in enumerate(diff_list):
            f.write(f"{times[i]:.6f},{diff['x']:.6f},{diff['y']:.6f},{diff['z']:.6f},"
                   f"{roll_diff[i]:.6f},{pitch_diff[i]:.6f},{yaw_diff[i]:.6f}\n")
    print(f"CSV data saved: {csv_path}")

    # 地図座標系での位置姿勢値も保存
    reference_times = [(ts - reference_pose_list[0][0]) / 1e9 for ts, _ in reference_pose_list]
    target_times = [(ts - target_pose_list[0][0]) / 1e9 for ts, _ in target_pose_list]

    csv_pose_path = Path(output_dir) / 'localization_pose_data.csv'
    with open(csv_pose_path, 'w') as f:
        f.write("time[s],reference_x[m],reference_y[m],reference_z[m],reference_roll[deg],reference_pitch[deg],reference_yaw[deg],"
               f"target_x[m],target_y[m],target_z[m],target_roll[deg],target_pitch[deg],target_yaw[deg]\n")
        for i, ((ts_reference, reference_pose), (ts_target, target_pose)) in enumerate(zip(reference_pose_list, target_pose_list)):
            time_val = reference_times[i] if i < len(reference_times) else target_times[i] if i < len(target_times) else i
            f.write(f"{time_val:.6f},"
                   f"{reference_pose['x']:.6f},{reference_pose['y']:.6f},{reference_pose['z']:.6f},"
                   f"{math.degrees(reference_pose['roll']):.6f},{math.degrees(reference_pose['pitch']):.6f},{math.degrees(reference_pose['yaw']):.6f},"
                   f"{target_pose['x']:.6f},{target_pose['y']:.6f},{target_pose['z']:.6f},"
                   f"{math.degrees(target_pose['roll']):.6f},{math.degrees(target_pose['pitch']):.6f},{math.degrees(target_pose['yaw']):.6f}\n")
    print(f"Pose data (map frame) saved: {csv_pose_path}")


def plot_diff_pose(
    reference_diff_pose: List[Tuple[float, dict]],
    target_diff_pose: List[Tuple[float, dict]],
    output_dir: str,
    reference_name: str,
    target_name: str,
    reference_path: str = None,
    target_path: str = None
):
    """
    diff_poseデータをグラフ表示

    Args:
        reference_diff_pose: 比較基準bagのdiff_poseデータ [(timestamp_ns, pose_data), ...]
        target_diff_pose: 比較対象bagのdiff_poseデータ [(timestamp_ns, pose_data), ...]
        output_dir: 出力ディレクトリ
        reference_name: 比較基準bagの名前
        target_name: 比較対象bagの名前
        reference_path: 比較基準bagのパス（lidar_marker_localizerの連続区間検出用）
        target_path: 比較対象bagのパス（lidar_marker_localizerの連続区間検出用）
    """
    if len(reference_diff_pose) == 0 and len(target_diff_pose) == 0:
        print("Warning: No diff_pose data available")
        return

    # タイムスタンプを秒に変換（最初のタイムスタンプを0秒とする）
    all_timestamps = []
    if len(reference_diff_pose) > 0:
        all_timestamps.append(reference_diff_pose[0][0])
    if len(target_diff_pose) > 0:
        all_timestamps.append(target_diff_pose[0][0])

    if len(all_timestamps) == 0:
        return

    start_time = min(all_timestamps)

    # lidar_marker_localizerの連続出力区間を検出
    lidar_marker_intervals = []
    if reference_path:
        debug_topic = "/localization/pose_estimator/lidar_marker_localizer/top_left/lidar_marker_localizer/debug/pose_with_covariance"
        try:
            print(f"Detecting continuous intervals for {debug_topic} (diff_pose graph)...")
            timestamps = read_topic_timestamps(reference_path, debug_topic)
            if len(timestamps) > 0:
                # 180ms = 180000000 nanoseconds
                intervals = find_continuous_intervals(timestamps, max_gap_ns=180000000)
                # タイムスタンプを秒に変換（start_timeを基準に）
                for start_ts, end_ts in intervals:
                    start_sec = (start_ts - start_time) / 1e9
                    end_sec = (end_ts - start_time) / 1e9
                    lidar_marker_intervals.append((start_sec, end_sec))
                if len(lidar_marker_intervals) > 0:
                    print(f"  Found {len(intervals)} continuous intervals")
            else:
                print(f"  No messages found for {debug_topic}")
        except Exception as e:
            print(f"  Warning: Failed to detect intervals: {e}")

    # データを抽出
    reference_times = [(ts - start_time) / 1e9 for ts, _ in reference_diff_pose] if len(reference_diff_pose) > 0 else []
    reference_x = [d['x'] for _, d in reference_diff_pose] if len(reference_diff_pose) > 0 else []
    reference_y = [d['y'] for _, d in reference_diff_pose] if len(reference_diff_pose) > 0 else []
    reference_z = [d['z'] for _, d in reference_diff_pose] if len(reference_diff_pose) > 0 else []
    reference_roll = [math.degrees(d['roll']) for _, d in reference_diff_pose] if len(reference_diff_pose) > 0 else []
    reference_pitch = [math.degrees(d['pitch']) for _, d in reference_diff_pose] if len(reference_diff_pose) > 0 else []
    reference_yaw = [math.degrees(d['yaw']) for _, d in reference_diff_pose] if len(reference_diff_pose) > 0 else []

    target_times = [(ts - start_time) / 1e9 for ts, _ in target_diff_pose] if len(target_diff_pose) > 0 else []
    target_x = [d['x'] for _, d in target_diff_pose] if len(target_diff_pose) > 0 else []
    target_y = [d['y'] for _, d in target_diff_pose] if len(target_diff_pose) > 0 else []
    target_z = [d['z'] for _, d in target_diff_pose] if len(target_diff_pose) > 0 else []
    target_roll = [math.degrees(d['roll']) for _, d in target_diff_pose] if len(target_diff_pose) > 0 else []
    target_pitch = [math.degrees(d['pitch']) for _, d in target_diff_pose] if len(target_diff_pose) > 0 else []
    target_yaw = [math.degrees(d['yaw']) for _, d in target_diff_pose] if len(target_diff_pose) > 0 else []

    # グラフを作成
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f'Pose Instability Detector diff_pose\n{reference_name} vs {target_name}', fontsize=14)

    # 位置のグラフ
    if len(reference_x) > 0:
        axes[0, 0].plot(reference_times, reference_x, label=f'{reference_name}', linewidth=1.5, color='blue')
    if len(target_x) > 0:
        axes[0, 0].plot(target_times, target_x, label=f'{target_name}', linewidth=1.5, color='red', linestyle='--')
    if len(lidar_marker_intervals) > 0:
        for i, (start_sec, end_sec) in enumerate(lidar_marker_intervals):
            axes[0, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green', label='lidar_marker_active' if i == 0 else '')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Position X [m]')
    axes[0, 0].set_title('X Position (diff_pose)')
    axes[0, 0].grid(True)
    axes[0, 0].legend()

    if len(reference_y) > 0:
        axes[0, 1].plot(reference_times, reference_y, label=f'{reference_name}', linewidth=1.5, color='blue')
    if len(target_y) > 0:
        axes[0, 1].plot(target_times, target_y, label=f'{target_name}', linewidth=1.5, color='red', linestyle='--')
    if len(lidar_marker_intervals) > 0:
        for start_sec, end_sec in lidar_marker_intervals:
            axes[0, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Position Y [m]')
    axes[0, 1].set_title('Y Position (diff_pose)')
    axes[0, 1].grid(True)
    axes[0, 1].legend()

    if len(reference_z) > 0:
        axes[1, 0].plot(reference_times, reference_z, label=f'{reference_name}', linewidth=1.5, color='blue')
    if len(target_z) > 0:
        axes[1, 0].plot(target_times, target_z, label=f'{target_name}', linewidth=1.5, color='red', linestyle='--')
    if len(lidar_marker_intervals) > 0:
        for start_sec, end_sec in lidar_marker_intervals:
            axes[1, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Position Z [m]')
    axes[1, 0].set_title('Z Position (diff_pose)')
    axes[1, 0].grid(True)
    axes[1, 0].legend()

    # 姿勢のグラフ
    if len(reference_roll) > 0:
        axes[1, 1].plot(reference_times, reference_roll, label=f'{reference_name}', linewidth=1.5, color='blue')
    if len(target_roll) > 0:
        axes[1, 1].plot(target_times, target_roll, label=f'{target_name}', linewidth=1.5, color='red', linestyle='--')
    if len(lidar_marker_intervals) > 0:
        for start_sec, end_sec in lidar_marker_intervals:
            axes[1, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Roll [deg]')
    axes[1, 1].set_title('Roll Orientation (diff_pose)')
    axes[1, 1].grid(True)
    axes[1, 1].legend()

    if len(reference_pitch) > 0:
        axes[2, 0].plot(reference_times, reference_pitch, label=f'{reference_name}', linewidth=1.5, color='blue')
    if len(target_pitch) > 0:
        axes[2, 0].plot(target_times, target_pitch, label=f'{target_name}', linewidth=1.5, color='red', linestyle='--')
    if len(lidar_marker_intervals) > 0:
        for start_sec, end_sec in lidar_marker_intervals:
            axes[2, 0].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_ylabel('Pitch [deg]')
    axes[2, 0].set_title('Pitch Orientation (diff_pose)')
    axes[2, 0].grid(True)
    axes[2, 0].legend()

    if len(reference_yaw) > 0:
        axes[2, 1].plot(reference_times, reference_yaw, label=f'{reference_name}', linewidth=1.5, color='blue')
    if len(target_yaw) > 0:
        axes[2, 1].plot(target_times, target_yaw, label=f'{target_name}', linewidth=1.5, color='red', linestyle='--')
    if len(lidar_marker_intervals) > 0:
        for start_sec, end_sec in lidar_marker_intervals:
            axes[2, 1].axvspan(start_sec, end_sec, alpha=0.2, color='green')
    axes[2, 1].set_xlabel('Time [s]')
    axes[2, 1].set_ylabel('Yaw [deg]')
    axes[2, 1].set_title('Yaw Orientation (diff_pose)')
    axes[2, 1].grid(True)
    axes[2, 1].legend()

    plt.tight_layout()

    # グラフを保存
    output_path = Path(output_dir) / 'diff_pose_comparison.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"diff_pose graph saved: {output_path}")
    plt.close(fig)

    # CSVファイルにも保存（referenceとtargetを別々に保存）
    csv_path = Path(output_dir) / 'diff_pose_data.csv'
    with open(csv_path, 'w') as f:
        f.write("bag,time[s],x[m],y[m],z[m],roll[deg],pitch[deg],yaw[deg]\n")

        # referenceのデータ
        for i, (ts, pose_data) in enumerate(reference_diff_pose):
            t = (ts - start_time) / 1e9
            f.write(f"{reference_name},{t:.6f},"
                   f"{pose_data['x']:.6f},{pose_data['y']:.6f},{pose_data['z']:.6f},"
                   f"{math.degrees(pose_data['roll']):.6f},{math.degrees(pose_data['pitch']):.6f},{math.degrees(pose_data['yaw']):.6f}\n")

        # targetのデータ
        for i, (ts, pose_data) in enumerate(target_diff_pose):
            t = (ts - start_time) / 1e9
            f.write(f"{target_name},{t:.6f},"
                   f"{pose_data['x']:.6f},{pose_data['y']:.6f},{pose_data['z']:.6f},"
                   f"{math.degrees(pose_data['roll']):.6f},{math.degrees(pose_data['pitch']):.6f},{math.degrees(pose_data['yaw']):.6f}\n")

    print(f"diff_pose CSV data saved: {csv_path}")


def main():
    parser = argparse.ArgumentParser(
        description='lidar_marker_localizerの有無でLocalizationの位置姿勢の差を比較'
    )
    parser.add_argument(
        '--reference_bag',
        type=str,
        required=True,
        help='比較基準となるrosbag2のパス（例: lidar_marker_localizerあり）'
    )
    parser.add_argument(
        '--target_bag',
        type=str,
        required=True,
        help='比較対象のrosbag2のパス（例: lidar_marker_localizerなし）'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        default='./output',
        help='出力ディレクトリ（デフォルト: ./output）'
    )
    parser.add_argument(
        '--topic',
        type=str,
        default='/localization/kinematic_state',
        help='読み込むトピック名（デフォルト: /localization/kinematic_state）'
    )

    args = parser.parse_args()

    # 出力ディレクトリを作成
    os.makedirs(args.output_dir, exist_ok=True)

    # rosbag2からデータを読み込む
    print(f"Loading reference bag: {args.reference_bag}")
    reference_data = read_kinematic_state_from_bag(args.reference_bag, args.topic)
    print(f"  Loaded: {len(reference_data)} messages")

    print(f"Loading target bag: {args.target_bag}")
    target_data = read_kinematic_state_from_bag(args.target_bag, args.topic)
    print(f"  Loaded: {len(target_data)} messages")

    if len(reference_data) == 0:
        print(f"Error: No data found in {args.reference_bag}")
        return

    if len(target_data) == 0:
        print(f"Error: No data found in {args.target_bag}")
        return

    # 差分を計算（referenceを基準に、targetとの差分を計算）
    # 車両座標系での差分を計算（比較基準データの姿勢を基準とした座標系）
    print("Calculating differences in vehicle frame...")
    diff_list, reference_pose_list, target_pose_list = calculate_differences(reference_data, target_data, use_vehicle_frame=True)
    print(f"  Calculation completed: {len(diff_list)} data points")

    # グラフ表示と保存
    reference_name = Path(args.reference_bag).stem
    target_name = Path(args.target_bag).stem
    print("Creating graph...")
    plot_differences(diff_list, reference_pose_list, target_pose_list, args.output_dir, reference_name, target_name, args.reference_bag, args.target_bag)

    # diff_poseトピックの読み込みとグラフ作成
    diff_pose_topic = "/localization/pose_twist_fusion_filter/pose_instability_detector/debug/diff_pose"
    print(f"\nLoading diff_pose from reference bag: {args.reference_bag}")
    reference_diff_pose = read_diff_pose_from_bag(args.reference_bag, diff_pose_topic)
    print(f"  Loaded: {len(reference_diff_pose)} messages")

    print(f"Loading diff_pose from target bag: {args.target_bag}")
    target_diff_pose = read_diff_pose_from_bag(args.target_bag, diff_pose_topic)
    print(f"  Loaded: {len(target_diff_pose)} messages")

    if len(reference_diff_pose) > 0 or len(target_diff_pose) > 0:
        print("Creating diff_pose graph...")
        plot_diff_pose(reference_diff_pose, target_diff_pose, args.output_dir, reference_name, target_name, args.reference_bag, args.target_bag)
    else:
        print(f"  Warning: No diff_pose data found in either bag. Skipping diff_pose graph.")

    print("\nProcessing completed!")


if __name__ == "__main__":
    main()
