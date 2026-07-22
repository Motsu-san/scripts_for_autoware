#!/usr/bin/env python3
"""
ros2_launch_*.log ファイルから vehicle_id_fragment|vehicle_model|vehicle_id|sensor_model の組み合わせを抽出するスクリプト
"""
import os
import re
import glob
from pathlib import Path
from collections import defaultdict

def extract_vehicle_id_fragment(rosbag_path):
    """
    ROSBAGパスから vehicle_id_fragment を抽出
    例: /path/to/rosbag_replay/x2_dev_<uuid8>-<uuid>_<date>_<time>/...
    -> <uuid8> を抽出
    """
    # UUID形式のパターン（例: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx）から最初の8文字を抽出
    # または、x2_dev_xxxxxxxx のような形式から先頭8文字を抽出
    patterns = [
        r'([a-f0-9]{8})-',  # UUID形式の最初の8文字
        r'x2_dev_([a-f0-9]{8})',  # x2_dev_ の後の8文字
        r'/([a-f0-9]{8})/',  # パス内の8文字の16進数
    ]

    for pattern in patterns:
        match = re.search(pattern, rosbag_path, re.IGNORECASE)
        if match:
            return match.group(1)

    # 見つからない場合は、パスから推測を試みる
    # ディレクトリ名から抽出を試みる
    parts = rosbag_path.split('/')
    for part in parts:
        if 'dev' in part.lower() or 'vehicle' in part.lower():
            # x2_dev_xxxxxxxx-... のような形式から抽出
            match = re.search(r'([a-f0-9]{8})', part, re.IGNORECASE)
            if match:
                return match.group(1)

    return None

def parse_log_file(log_file_path):
    """
    ログファイルを解析して vehicle_id_fragment|vehicle_model|vehicle_id|sensor_model を抽出
    """
    vehicle_id_fragment = None
    vehicle_model = None
    vehicle_id = None
    sensor_model = None
    rosbag_path = None

    try:
        with open(log_file_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()

                # ROSBAGパスを抽出
                if line.startswith('ROSBAG:'):
                    rosbag_path = line.split('ROSBAG:', 1)[1].strip()
                    vehicle_id_fragment = extract_vehicle_id_fragment(rosbag_path)

                # VEHICLE_MODELを抽出
                elif line.startswith('VEHICLE_MODEL:'):
                    vehicle_model = line.split('VEHICLE_MODEL:', 1)[1].strip()

                # VEHICLE_IDを抽出
                elif line.startswith('VEHICLE_ID:'):
                    vehicle_id = line.split('VEHICLE_ID:', 1)[1].strip()

                # SENSOR_MODELを抽出
                elif line.startswith('SENSOR_MODEL:'):
                    sensor_model = line.split('SENSOR_MODEL:', 1)[1].strip()

        # すべての情報が揃っているか確認
        if vehicle_id_fragment and vehicle_model and vehicle_id and sensor_model:
            return f"{vehicle_id_fragment}|{vehicle_model}|{vehicle_id}|{sensor_model}"
        elif vehicle_model and vehicle_id and sensor_model:
            # vehicle_id_fragment が見つからない場合でも、他の情報があれば返す
            return f"{vehicle_id_fragment or 'N/A'}|{vehicle_model}|{vehicle_id}|{sensor_model}"
        else:
            return None

    except Exception as e:
        print(f"Error reading {log_file_path}: {e}", file=os.sys.stderr)
        return None

def main():
    log_dir = Path.home() / 'log'
    log_pattern = str(log_dir / 'ros2_launch_*.log')

    log_files = glob.glob(log_pattern)

    if not log_files:
        print(f"No log files found matching pattern: {log_pattern}", file=os.sys.stderr)
        return

    # 重複を避けるためにセットを使用
    unique_configs = set()
    configs_by_file = {}

    for log_file in sorted(log_files):
        config = parse_log_file(log_file)
        if config:
            unique_configs.add(config)
            configs_by_file[log_file] = config

    # 結果を出力
    print("# 抽出された vehicle_id_fragment|vehicle_model|vehicle_id|sensor_model の組み合わせ:")
    print()

    # ユニークな組み合わせを出力
    for config in sorted(unique_configs):
        print(config)

    print()
    print(f"# 合計: {len(unique_configs)} 種類の組み合わせが見つかりました")
    print(f"# 処理したログファイル数: {len([f for f in configs_by_file.values() if f])}")

if __name__ == '__main__':
    main()
