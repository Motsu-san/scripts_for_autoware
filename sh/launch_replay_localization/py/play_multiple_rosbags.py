#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
# 出力バッファリングを無効化（ログに即座に出力されるように）
sys.stdout = sys.__stdout__
sys.stderr = sys.__stderr__

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
複数のrosbagを同時に再生するPythonスクリプト

使用方法:
    python3 play_multiple_rosbags.py \
        --source-bag /path/to/original.bag \
        --recorded-bag /path/to/recorded.bag \
        --recorded-topics /localization/kinematic_state

    # トピック名をリマップする場合
    python3 play_multiple_rosbags.py \
        --source-bag /path/to/original.bag \
        --recorded-bag /path/to/recorded.bag \
        --recorded-topics /localization/kinematic_state \
        --remap /localization/kinematic_state:=/localization/kinematic_state_recorded

    # 再生速度とクロック周波数を指定（ros2 bag play -r 1.0 --clock 200 と同等）
    python3 play_multiple_rosbags.py \
        --source-bag /path/to/original.bag \
        --recorded-bag /path/to/recorded.bag \
        --recorded-topics /localization/kinematic_state \
        -r 1.0 \
        --clock 200
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


class RosbagPlayer:
    """rosbag再生を管理するクラス"""

    def __init__(self, bag_path, topics=None, exclude_topics=None,
                 remap=None, rate=1.0, clock_hz=None, start_offset=0, use_clock=True):
        self.bag_path = bag_path
        self.topics = topics or []
        self.exclude_topics = exclude_topics or []
        self.remap = remap or []
        self.rate = rate
        self.clock_hz = clock_hz
        self.start_offset = start_offset
        self.use_clock = use_clock
        self.process = None

    def start(self):
        """rosbagの再生を開始"""
        cmd = ['ros2', 'bag', 'play', self.bag_path]

        # 再生速度（-r）
        if self.rate != 1.0:
            cmd.extend(['-r', str(self.rate)])

        # クロックオプション（--clock [Hz]）
        # 注意: 複数のrosbagを同時に再生する場合、/clockトピックの競合を避けるため、
        # 記録バッグ（recorded bag）ではuse_clock=Falseに設定してください
        if self.use_clock:
            if self.clock_hz is not None:
                cmd.extend(['--clock', str(self.clock_hz)])
            else:
                cmd.extend(['--clock'])

        # 開始オフセット
        if self.start_offset > 0:
            cmd.extend(['--start-offset', str(self.start_offset)])

        # トピック指定（リマップ前のトピック名で指定）
        # 注意: --topicsオプションは1回だけ指定し、その後に複数のトピック名を並べる
        if self.topics:
            cmd.extend(['--topics'])
            for topic in self.topics:
                cmd.extend([topic])
                print(f"再生トピック: {topic}", flush=True)

        # トピック除外（ros2 bag playには--exclude-topicsオプションがないため、この機能は使用不可）
        # 代わりに--remapを使用してトピック名を変更して競合を避ける
        # if self.exclude_topics:
        #     for topic in self.exclude_topics:
        #         cmd.extend(['--exclude-topics', topic])

        # リマップ（--topicsで指定したトピックをリマップ）
        # 注意: --topicsで指定するトピック名はリマップ前の名前（rosbag内の元の名前）
        if self.remap:
            for remap in self.remap:
                cmd.extend(['--remap', remap])
                print(f"リマップ設定: {remap}", flush=True)

        print(f"実行コマンド: {' '.join(cmd)}", flush=True)
        print(f"デバッグ: バッグパス={self.bag_path}, トピック={self.topics}, リマップ={self.remap}", flush=True)
        self.process = subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr)
        return self.process

    def stop(self):
        """rosbagの再生を停止"""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            self.process = None


def parse_remap(remap_str):
    """リマップ文字列をパース（OLD:=NEW形式）"""
    if ':=/' not in remap_str:
        raise ValueError(f"無効なリマップ形式: {remap_str} (形式: OLD:=NEW)")
    return remap_str


def main():
    parser = argparse.ArgumentParser(
        description='複数のrosbagを同時に再生するスクリプト',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  # 基本的な使用方法
  %(prog)s --source-bag original.bag --recorded-bag recorded.bag \\
      --recorded-topics /localization/kinematic_state

  # トピック名をリマップ
  %(prog)s --source-bag original.bag --recorded-bag recorded.bag \\
      --recorded-topics /localization/kinematic_state \\
      --remap /localization/kinematic_state:=/localization/kinematic_state_recorded

  # 記録したrosbagのトピックのみを再生
  %(prog)s --source-bag original.bag --recorded-bag recorded.bag \\
      --recorded-topics /localization/kinematic_state --topics-only

  # 再生速度とクロック周波数を指定
  %(prog)s --source-bag original.bag --recorded-bag recorded.bag \\
      --recorded-topics /localization/kinematic_state \\
      -r 1.0 --clock 200
        """
    )

    parser.add_argument(
        '--source-bag',
        required=True,
        help='元のrosbagのパス'
    )

    parser.add_argument(
        '--recorded-bag',
        required=True,
        help='記録したrosbagのパス'
    )

    parser.add_argument(
        '--recorded-topics',
        nargs='+',
        required=True,
        help='記録したrosbagから再生するトピック'
    )

    parser.add_argument(
        '--remap',
        action='append',
        default=[],
        help='トピック名をリマップ（OLD:=NEW形式）。複数回指定可能'
    )

    parser.add_argument(
        '--topics-only',
        action='store_true',
        help='記録したrosbagの指定トピックのみを再生（注意: ros2 bag playには--exclude-topicsがないため、元のrosbagはそのまま再生されます。--remapを使用して競合を避けてください）'
    )

    parser.add_argument(
        '-r', '--rate',
        type=float,
        default=1.0,
        help='再生速度（-r、デフォルト: 1.0）'
    )

    parser.add_argument(
        '--clock',
        type=float,
        default=None,
        help='クロック周波数（Hz、--clock [Hz]）。指定しない場合はデフォルトのクロック'
    )

    parser.add_argument(
        '--start-offset',
        type=float,
        default=0,
        help='開始オフセット（秒、デフォルト: 0）'
    )

    args = parser.parse_args()

    # rosbagファイルの存在確認
    source_bag = Path(args.source_bag)
    recorded_bag = Path(args.recorded_bag)

    if not source_bag.exists():
        print(f"エラー: 元のrosbagが見つかりません: {source_bag}", file=sys.stderr)
        sys.exit(1)

    if not recorded_bag.exists():
        print(f"エラー: 記録したrosbagが見つかりません: {recorded_bag}", file=sys.stderr)
        sys.exit(1)

    print("=" * 50, flush=True)
    print("複数rosbag同時再生スクリプト", flush=True)
    print("=" * 50, flush=True)
    print(f"元のrosbag: {source_bag}", flush=True)
    print(f"記録したrosbag: {recorded_bag}", flush=True)
    print(f"再生トピック: {', '.join(args.recorded_topics)}", flush=True)
    print(f"再生速度: {args.rate}", flush=True)
    if args.clock is not None:
        print(f"クロック周波数: {args.clock} Hz", flush=True)
    if args.remap:
        print(f"リマップ: {', '.join(args.remap)}", flush=True)
    print("=" * 50, flush=True)
    print(flush=True)

    # rosbagプレイヤーを作成
    # 注意: ros2 bag playには--exclude-topicsオプションがないため、
    # --topics-onlyが指定されても元のrosbagはそのまま再生されます。
    # 競合を避けるには--remapオプションを使用してください。
    source_player = RosbagPlayer(
        bag_path=str(source_bag),
        exclude_topics=None,  # ros2 bag playには--exclude-topicsがないため常にNone
        rate=args.rate,
        clock_hz=args.clock,
        start_offset=args.start_offset
    )

    # 記録バッグのリマップリストを作成（/clockトピックのリマップを追加）
    recorded_remap = list(args.remap) if args.remap else []
    # /clockトピックをリマップして競合を避ける（記録バッグの/clockは使用しない）
    recorded_remap.append('/clock:=/clock_recorded')

    recorded_player = RosbagPlayer(
        bag_path=str(recorded_bag),
        topics=args.recorded_topics,
        remap=recorded_remap,
        rate=args.rate,
        clock_hz=args.clock,  # 記録バッグも--clockオプションを使用（タイムスタンプ同期のため）
        start_offset=args.start_offset,
        use_clock=True  # 記録バッグも--clockオプションを使用するが、/clockトピックはリマップして競合を避ける
    )

    # シグナルハンドラを設定
    def signal_handler(sig, frame):
        print("\n再生を停止しています...")
        source_player.stop()
        recorded_player.stop()
        print("停止しました")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # 元のrosbagを再生
        print("元のrosbagを再生中...", flush=True)
        source_player.start()

        # 少し待ってから記録したrosbagを再生
        time.sleep(1)

        print("記録したrosbagを再生中...", flush=True)
        recorded_player.start()

        print(flush=True)
        print("両方のrosbagが再生中です...", flush=True)
        print("停止するには Ctrl+C を押してください", flush=True)
        print(flush=True)

        # プロセスが終了するまで待機
        source_player.process.wait()
        recorded_player.process.wait()

    except KeyboardInterrupt:
        signal_handler(None, None)
    except Exception as e:
        print(f"エラーが発生しました: {e}", file=sys.stderr)
        source_player.stop()
        recorded_player.stop()
        sys.exit(1)


if __name__ == '__main__':
    main()
