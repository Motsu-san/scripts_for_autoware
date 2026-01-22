#!/bin/bash

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

# Usage: ./batch_compare_localization.sh <MAP_PATH> <ROSBAG_LIST_FILE> [OUTPUT_BASE_DIR] [AUTOWARE_DIR]
# ROSBAG_LIST_FILE: 各行にrosbagのパスを記載したファイル、またはスペース区切りのrosbagパス
# OUTPUT_BASE_DIR: 結果を保存するベースディレクトリ（デフォルト: $HOME/comparison_results）
# AUTOWARE_DIR: Autowareのディレクトリ（デフォルト: 現在のディレクトリ）

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALL_DIR=$(pwd)

# 引数のチェック
if [ $# -lt 2 ]; then
    echo "Usage: $0 <MAP_PATH> <ROSBAG_LIST_FILE_OR_PATHS> [OUTPUT_BASE_DIR] [AUTOWARE_DIR]"
    echo ""
    echo "Arguments:"
    echo "  MAP_PATH: マップのパス"
    echo "  ROSBAG_LIST_FILE_OR_PATHS: rosbagのパスのリストファイル、またはスペース区切りのrosbagパス"
    echo "  OUTPUT_BASE_DIR: 結果を保存するベースディレクトリ（デフォルト: \$HOME/comparison_results）"
    echo "  AUTOWARE_DIR: Autowareのディレクトリ（デフォルト: 現在のディレクトリ）"
    echo ""
    echo "Example:"
    echo "  $0 \"\$HOME/autoware_map\" \"\$HOME/rosbag_list.txt\""
    echo "  $0 \"\$HOME/autoware_map\" \"bag1.db3 bag2.db3 bag3.db3\""
    exit 1
fi

MAP_PATH="$1"
ROSBAG_INPUT="$2"
OUTPUT_BASE_DIR="${3:-$HOME/comparison_results}"
AUTOWARE_DIR="${4:-$CALL_DIR}"

# Autowareディレクトリの確認
if [ ! -f "$AUTOWARE_DIR/install/setup.bash" ]; then
    echo "Error: Autoware directory not found or not built: $AUTOWARE_DIR"
    echo "Please build Autoware first or specify the correct AUTOWARE_DIR"
    exit 1
fi

# マップパスの確認
if [ ! -d "$MAP_PATH" ]; then
    echo "Error: MAP_PATH does not exist: $MAP_PATH"
    exit 1
fi

# rosbagリストの読み込み
if [ -f "$ROSBAG_INPUT" ]; then
    # ファイルから読み込み
    readarray -t ROSBAG_LIST < <(grep -v '^#' "$ROSBAG_INPUT" | grep -v '^$' | tr -d '\r')
    echo "Reading rosbag list from file: $ROSBAG_INPUT"
else
    # スペース区切りの引数として扱う
    ROSBAG_LIST=($ROSBAG_INPUT)
    echo "Reading rosbag list from command line arguments"
fi

if [ ${#ROSBAG_LIST[@]} -eq 0 ]; then
    echo "Error: No rosbags found in input"
    exit 1
fi

echo "=========================================="
echo "Batch Localization Comparison Script"
echo "=========================================="
echo "MAP_PATH: $MAP_PATH"
echo "Number of rosbags: ${#ROSBAG_LIST[@]}"
echo "Rosbag list:"
for i in "${!ROSBAG_LIST[@]}"; do
    echo "  [$((i+1))] ${ROSBAG_LIST[$i]}"
done
echo "OUTPUT_BASE_DIR: $OUTPUT_BASE_DIR"
echo "AUTOWARE_DIR: $AUTOWARE_DIR"
echo ""

# 出力ディレクトリの作成
DATETIME=$(date '+%Y%m%d_%H%M%S')
RESULTS_DIR="$OUTPUT_BASE_DIR/comparison_$DATETIME"
mkdir -p "$RESULTS_DIR"

# ログファイル
LOG_FILE="$RESULTS_DIR/batch_comparison.log"
exec > >(tee -a "$LOG_FILE")
exec 2>&1

echo "Results will be saved to: $RESULTS_DIR"
echo "Log file: $LOG_FILE"
echo ""

# 各rosbagを処理
SUCCESS_COUNT=0
FAIL_COUNT=0
FAILED_BAGS=()

for i in "${!ROSBAG_LIST[@]}"; do
    ROSBAG="${ROSBAG_LIST[$i]}"

    # rosbagの固有名を抽出（~/rosbag_replay直下のフォルダ名）
    ROSBAG_ABSPATH=$(readlink -f "$ROSBAG" 2>/dev/null || echo "$ROSBAG")
    ROSBAG_REPLAY_DIR="$HOME/rosbag_replay"
    ROSBAG_REPLAY_DIR_ABSPATH=$(readlink -f "$ROSBAG_REPLAY_DIR" 2>/dev/null || echo "$ROSBAG_REPLAY_DIR")

    # rosbagのパスから~/rosbag_replay直下のフォルダ名を抽出
    ROSBAG_UNIQUE_NAME=""
    if [[ "$ROSBAG_ABSPATH" == "$ROSBAG_REPLAY_DIR_ABSPATH"/* ]]; then
        # ~/rosbag_replay以降のパスを取得
        RELATIVE_PATH="${ROSBAG_ABSPATH#$ROSBAG_REPLAY_DIR_ABSPATH/}"
        # 最初のディレクトリ名を取得（固有名）
        FIRST_DIR=$(echo "$RELATIVE_PATH" | cut -d'/' -f1)
        ROSBAG_UNIQUE_NAME="$FIRST_DIR"
    fi

    # rosbagのbasenameを取得
    ROSBAG_BASENAME=$(basename "$ROSBAG" .db3)
    ROSBAG_BASENAME=$(basename "$ROSBAG_BASENAME" .mcap)

    # 一意の名前を生成
    if [ -n "$ROSBAG_UNIQUE_NAME" ]; then
        # 固有名_basenameの形式
        ROSBAG_NAME="${ROSBAG_UNIQUE_NAME}_${ROSBAG_BASENAME}"
    else
        # 固有名が取得できない場合は、パスから推測
        ROSBAG_DIRNAME=$(basename "$(dirname "$ROSBAG")")
        if [ "$ROSBAG_DIRNAME" != "." ] && [ "$ROSBAG_DIRNAME" != "$ROSBAG_BASENAME" ]; then
            ROSBAG_NAME="${ROSBAG_DIRNAME}_${ROSBAG_BASENAME}"
        else
            ROSBAG_NAME="$ROSBAG_BASENAME"
        fi
        # インデックスを追加して確実に一意にする
        ROSBAG_NAME="${ROSBAG_NAME}_${i}"
    fi

    echo "=========================================="
    echo "[$((i+1))/${#ROSBAG_LIST[@]}] Processing: $ROSBAG_NAME"
    echo "Full path: $ROSBAG"
    if [ -n "$ROSBAG_UNIQUE_NAME" ]; then
        echo "Unique name (from ~/rosbag_replay): $ROSBAG_UNIQUE_NAME"
    fi
    echo "=========================================="

    # rosbagファイルの存在確認
    if [ ! -f "$ROSBAG" ]; then
        echo "Error: Rosbag file not found: $ROSBAG"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG")
        continue
    fi

    # このrosbag用の出力ディレクトリ（一意の名前を使用）
    ROSBAG_OUTPUT_DIR="$RESULTS_DIR/$ROSBAG_NAME"
    # 既に存在する場合は警告を出す
    if [ -d "$ROSBAG_OUTPUT_DIR" ]; then
        echo "Warning: Output directory already exists: $ROSBAG_OUTPUT_DIR"
        echo "  This may overwrite previous results. Using existing directory."
    fi
    mkdir -p "$ROSBAG_OUTPUT_DIR"

    # pose_source=0 (ndt) で実行
    echo ""
    echo "--- Running with pose_source=0 (ndt) ---"
    POSE_SOURCE_0_DIR="$ROSBAG_OUTPUT_DIR/pose_source_0_ndt"
    mkdir -p "$POSE_SOURCE_0_DIR"

    cd "$AUTOWARE_DIR"
    ROSBAG_DIR=$(dirname "$ROSBAG")

    # 既存のrecord_replay_*ディレクトリをバックアップ（クリーンアップ）
    echo "Cleaning up old record directories in $ROSBAG_DIR..."
    rm -rf "$ROSBAG_DIR"/record_replay_* 2>/dev/null || true

    echo "Starting Autoware with pose_source=0 (ndt)..."
    # launch_autoware.shはrosbagの再生が終了するまで待つ
    # バックグラウンドで実行して、OUTPUT_DIRが作成されるのを待つ
    # 引数の順序: <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE]
    if ! "$SCRIPT_DIR/launch_autoware.sh" "$MAP_PATH" "$ROSBAG" "0" "false" "output_lidar-marker" > "$POSE_SOURCE_0_DIR/launch.log" 2>&1; then
        echo "Error: Failed to run with pose_source=0"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=0 failed)")
        # Autowareを停止
        "$SCRIPT_DIR/kill_autoware.sh" || true
        sleep 5
        continue
    fi

    # 記録されたrosbagを探す（最新のrecord_replay_*ディレクトリ）
    # 少し待ってからディレクトリを探す（rosbag記録が完了するまで）
    echo "Waiting for rosbag recording to complete..."
    sleep 5
    # 最大30秒待つ（ディレクトリが作成されるまで）
    for i in {1..6}; do
        LATEST_RECORD_DIR=$(ls -td "$ROSBAG_DIR"/record_replay_* 2>/dev/null | head -n1)
        if [ -n "$LATEST_RECORD_DIR" ] && [ -d "$LATEST_RECORD_DIR" ]; then
            break
        fi
        echo "  Waiting for record directory... ($i/6)"
        sleep 5
    done

    if [ -z "$LATEST_RECORD_DIR" ] || [ ! -d "$LATEST_RECORD_DIR" ]; then
        echo "Error: Recorded rosbag directory not found in $ROSBAG_DIR"
        echo "Available directories:"
        ls -la "$ROSBAG_DIR" | grep record_replay || echo "  (none)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=0 recording not found)")
        continue
    fi

    # 記録されたrosbagをコピー
    echo "Copying recorded bag from: $LATEST_RECORD_DIR"
    echo "  Source directory contents:"
    ls -la "$LATEST_RECORD_DIR" | head -5

    # 既存のrecorded_bagディレクトリを削除してからコピー
    rm -rf "$POSE_SOURCE_0_DIR/recorded_bag"
    cp -r "$LATEST_RECORD_DIR" "$POSE_SOURCE_0_DIR/recorded_bag"
    BAG1_PATH="$POSE_SOURCE_0_DIR/recorded_bag"

    # コピーが成功したか確認
    if [ ! -d "$BAG1_PATH" ] || [ -z "$(ls -A "$BAG1_PATH" 2>/dev/null)" ]; then
        echo "Error: Failed to copy recorded bag. Directory is empty or does not exist."
        echo "  BAG1_PATH: $BAG1_PATH"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=0 copy failed)")
        continue
    fi

    # rosbag2のmetadata.yamlが存在するか確認
    if [ ! -f "$BAG1_PATH/metadata.yaml" ]; then
        echo "Warning: metadata.yaml not found in $BAG1_PATH"
        echo "  Looking for rosbag2 directory..."
        # サブディレクトリを探す
        ROSBAG2_SUBDIR=$(find "$BAG1_PATH" -name "metadata.yaml" -type f | head -1 | xargs dirname)
        if [ -n "$ROSBAG2_SUBDIR" ]; then
            echo "  Found rosbag2 directory: $ROSBAG2_SUBDIR"
            BAG1_PATH="$ROSBAG2_SUBDIR"
        else
            echo "Error: Could not find rosbag2 directory in $BAG1_PATH"
            FAIL_COUNT=$((FAIL_COUNT + 1))
            FAILED_BAGS+=("$ROSBAG (pose_source=0 rosbag2 not found)")
            continue
        fi
    fi

    echo "Recorded bag saved to: $BAG1_PATH"

    # Autowareを停止（念のため）
    "$SCRIPT_DIR/kill_autoware.sh" || true
    sleep 5

    # pose_source=1 (ndt_lidar-marker) で実行
    echo ""
    echo "--- Running with pose_source=1 (ndt_lidar-marker) ---"
    POSE_SOURCE_1_DIR="$ROSBAG_OUTPUT_DIR/pose_source_1_ndt_lidar_marker"
    mkdir -p "$POSE_SOURCE_1_DIR"

    cd "$AUTOWARE_DIR"

    # 既存のrecord_replay_*ディレクトリをバックアップ（クリーンアップ）
    echo "Cleaning up old record directories in $ROSBAG_DIR..."
    rm -rf "$ROSBAG_DIR"/record_replay_* 2>/dev/null || true

    echo "Starting Autoware with pose_source=1 (ndt_lidar-marker)..."
    # launch_autoware.shはrosbagの再生が終了するまで待つ
    # 引数の順序: <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE]
    if ! "$SCRIPT_DIR/launch_autoware.sh" "$MAP_PATH" "$ROSBAG" "1" "false" "output_lidar-marker" > "$POSE_SOURCE_1_DIR/launch.log" 2>&1; then
        echo "Error: Failed to run with pose_source=1"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=1 failed)")
        # Autowareを停止
        "$SCRIPT_DIR/kill_autoware.sh" || true
        sleep 5
        continue
    fi

    # 記録されたrosbagを探す
    # 少し待ってからディレクトリを探す（rosbag記録が完了するまで）
    echo "Waiting for rosbag recording to complete..."
    sleep 5
    # 最大30秒待つ（ディレクトリが作成されるまで）
    for i in {1..6}; do
        LATEST_RECORD_DIR=$(ls -td "$ROSBAG_DIR"/record_replay_* 2>/dev/null | head -n1)
        if [ -n "$LATEST_RECORD_DIR" ] && [ -d "$LATEST_RECORD_DIR" ]; then
            break
        fi
        echo "  Waiting for record directory... ($i/6)"
        sleep 5
    done

    if [ -z "$LATEST_RECORD_DIR" ] || [ ! -d "$LATEST_RECORD_DIR" ]; then
        echo "Error: Recorded rosbag directory not found in $ROSBAG_DIR"
        echo "Available directories:"
        ls -la "$ROSBAG_DIR" | grep record_replay || echo "  (none)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=1 recording not found)")
        continue
    fi

    # 記録されたrosbagをコピー
    echo "Copying recorded bag from: $LATEST_RECORD_DIR"
    echo "  Source directory contents:"
    ls -la "$LATEST_RECORD_DIR" | head -5

    # 既存のrecorded_bagディレクトリを削除してからコピー
    rm -rf "$POSE_SOURCE_1_DIR/recorded_bag"
    cp -r "$LATEST_RECORD_DIR" "$POSE_SOURCE_1_DIR/recorded_bag"
    BAG2_PATH="$POSE_SOURCE_1_DIR/recorded_bag"

    # コピーが成功したか確認
    if [ ! -d "$BAG2_PATH" ] || [ -z "$(ls -A "$BAG2_PATH" 2>/dev/null)" ]; then
        echo "Error: Failed to copy recorded bag. Directory is empty or does not exist."
        echo "  BAG2_PATH: $BAG2_PATH"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=1 copy failed)")
        continue
    fi

    # rosbag2のmetadata.yamlが存在するか確認
    if [ ! -f "$BAG2_PATH/metadata.yaml" ]; then
        echo "Warning: metadata.yaml not found in $BAG2_PATH"
        echo "  Looking for rosbag2 directory..."
        # サブディレクトリを探す
        ROSBAG2_SUBDIR=$(find "$BAG2_PATH" -name "metadata.yaml" -type f | head -1 | xargs dirname)
        if [ -n "$ROSBAG2_SUBDIR" ]; then
            echo "  Found rosbag2 directory: $ROSBAG2_SUBDIR"
            BAG2_PATH="$ROSBAG2_SUBDIR"
        else
            echo "Error: Could not find rosbag2 directory in $BAG2_PATH"
            FAIL_COUNT=$((FAIL_COUNT + 1))
            FAILED_BAGS+=("$ROSBAG (pose_source=1 rosbag2 not found)")
            continue
        fi
    fi

    echo "Recorded bag saved to: $BAG2_PATH"

    # Autowareを停止（念のため）
    "$SCRIPT_DIR/kill_autoware.sh" || true
    sleep 5

    # 比較スクリプトを実行
    echo ""
    echo "--- Running comparison script ---"
    COMPARISON_OUTPUT_DIR="$ROSBAG_OUTPUT_DIR/comparison_results"
    mkdir -p "$COMPARISON_OUTPUT_DIR"

    # rosbag2のパスを取得（recorded_bagディレクトリ内のrosbag2データベース）
    # rosbag2は通常、ディレクトリ内に複数のファイルがあるので、ディレクトリ自体をパスとして渡す
    if [ ! -d "$BAG1_PATH" ] || [ ! -d "$BAG2_PATH" ]; then
        echo "Error: Recorded bag directories not found"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (comparison failed)")
        continue
    fi

    # Pythonスクリプトを実行
    # lidar_marker_localizerあり（pose_source=1）を基準、なし（pose_source=0）を比較として設定
    if python3 $HOME/py_scripts/compare_localization_diff.py \
        --bag1 "$BAG2_PATH" \
        --bag2 "$BAG1_PATH" \
        --output_dir "$COMPARISON_OUTPUT_DIR" \
        > "$COMPARISON_OUTPUT_DIR/comparison.log" 2>&1; then
        echo "Comparison completed successfully"
        echo "Results saved to: $COMPARISON_OUTPUT_DIR"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    else
        echo "Error: Comparison failed. Check log: $COMPARISON_OUTPUT_DIR/comparison.log"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (comparison failed)")
    fi

    echo ""
done

# 結果サマリー
echo "=========================================="
echo "Batch Processing Summary"
echo "=========================================="
echo "Total rosbags: ${#ROSBAG_LIST[@]}"
echo "Successful: $SUCCESS_COUNT"
echo "Failed: $FAIL_COUNT"
echo ""

if [ $FAIL_COUNT -gt 0 ]; then
    echo "Failed rosbags:"
    for failed_bag in "${FAILED_BAGS[@]}"; do
        echo "  - $failed_bag"
    done
    echo ""
fi

echo "Results directory: $RESULTS_DIR"
echo "Log file: $LOG_FILE"
echo ""

# 成功した場合は終了コード0、失敗がある場合は1
if [ $FAIL_COUNT -eq 0 ]; then
    echo "All comparisons completed successfully!"
    exit 0
else
    echo "Some comparisons failed. Check the log for details."
    exit 1
fi
