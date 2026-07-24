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

# Usage: ./batch_compare_localization.sh <MAP_PATH> <ROSBAG_LIST_FILE> [OUTPUT_BASE_DIR] [AUTOWARE_DIR] [--reference_bag REF_BAG] [--target_bag TARGET_BAG]
# ROSBAG_LIST_FILE: 各行にrosbagのパスを記載したファイル、またはスペース区切りのrosbagパス
# OUTPUT_BASE_DIR: 結果を保存するベースディレクトリ(デフォルト: $HOME/comparison_results)
# AUTOWARE_DIR: Autowareのディレクトリ(デフォルト: 現在のディレクトリ)
# --reference_bag: 比較基準として使用するrosbagのパス(指定時、ndt_lidar-markerの再生をスキップ)
# --target_bag: 比較対象として使用するrosbagのパス(指定時、ndtの再生をスキップ)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALL_DIR=$(pwd)

# オプション引数の解析
REFERENCE_BAG=""
TARGET_BAG=""
POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        --reference_bag)
            REFERENCE_BAG="$2"
            shift 2
            ;;
        --target_bag)
            TARGET_BAG="$2"
            shift 2
            ;;
        *)
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done

# 位置引数を設定
set -- "${POSITIONAL_ARGS[@]}"

# 引数のチェック
if [ $# -lt 2 ]; then
    echo "Usage: $0 <MAP_PATH> <ROSBAG_LIST_FILE_OR_PATHS> [OUTPUT_BASE_DIR] [AUTOWARE_DIR] [--reference_bag REF_BAG] [--target_bag TARGET_BAG]"
    echo ""
    echo "Arguments:"
    echo "  MAP_PATH: マップのパス"
    echo "  ROSBAG_LIST_FILE_OR_PATHS: rosbagのパスのリストファイル、またはスペース区切りのrosbagパス"
    echo "  OUTPUT_BASE_DIR: 結果を保存するベースディレクトリ(デフォルト: \$HOME/comparison_results)"
    echo "  AUTOWARE_DIR: Autowareのディレクトリ(デフォルト: 現在のディレクトリ)"
    echo ""
    echo "Options:"
    echo "  --reference_bag REF_BAG: 比較基準として使用するrosbagのパス(指定時、ndt_lidar-markerの再生をスキップ)"
    echo "  --target_bag TARGET_BAG: 比較対象として使用するrosbagのパス(指定時、ndtの再生をスキップ)"
    echo ""
    echo "Example:"
    echo "  $0 \"\$HOME/autoware_map\" \"\$HOME/rosbag_list.txt\""
    echo "  $0 \"\$HOME/autoware_map\" \"bag1.db3 bag2.db3 bag3.db3\""
    echo "  $0 \"\$HOME/autoware_map\" \"bag1.db3\" --reference_bag \"\$HOME/ref_bag.db3\" --target_bag \"\$HOME/target_bag.db3\""
    exit 1
fi

MAP_PATH="$1"
ROSBAG_INPUT="$2"
OUTPUT_BASE_DIR="${3:-$HOME/comparison_results}"
AUTOWARE_DIR="${4:-$CALL_DIR}"

# オプション指定の確認
if [ -n "$REFERENCE_BAG" ]; then
    if [ ! -d "$REFERENCE_BAG" ] && [ ! -f "$REFERENCE_BAG" ]; then
        echo "Error: Reference bag not found: $REFERENCE_BAG"
        exit 1
    fi
    echo "Reference bag specified: $REFERENCE_BAG (ndt_lidar-marker recording will be skipped)"
fi

if [ -n "$TARGET_BAG" ]; then
    if [ ! -d "$TARGET_BAG" ] && [ ! -f "$TARGET_BAG" ]; then
        echo "Error: Target bag not found: $TARGET_BAG"
        exit 1
    fi
    echo "Target bag specified: $TARGET_BAG (ndt recording will be skipped)"
fi

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

    # rosbagの固有名を抽出(~/rosbag_replay直下のフォルダ名)
    ROSBAG_ABSPATH=$(readlink -f "$ROSBAG" 2>/dev/null || echo "$ROSBAG")
    ROSBAG_REPLAY_DIR="$HOME/rosbag_replay"
    ROSBAG_REPLAY_DIR_ABSPATH=$(readlink -f "$ROSBAG_REPLAY_DIR" 2>/dev/null || echo "$ROSBAG_REPLAY_DIR")

    # rosbagのパスから~/rosbag_replay直下のフォルダ名を抽出
    ROSBAG_UNIQUE_NAME=""
    if [[ "$ROSBAG_ABSPATH" == "$ROSBAG_REPLAY_DIR_ABSPATH"/* ]]; then
        # ~/rosbag_replay以降のパスを取得
        RELATIVE_PATH="${ROSBAG_ABSPATH#$ROSBAG_REPLAY_DIR_ABSPATH/}"
        # 最初のディレクトリ名を取得(固有名)
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

    # このrosbag用の出力ディレクトリ(一意の名前を使用)
    ROSBAG_OUTPUT_DIR="$RESULTS_DIR/$ROSBAG_NAME"
    # 既に存在する場合は警告を出す
    if [ -d "$ROSBAG_OUTPUT_DIR" ]; then
        echo "Warning: Output directory already exists: $ROSBAG_OUTPUT_DIR"
        echo "  This may overwrite previous results. Using existing directory."
    fi
    mkdir -p "$ROSBAG_OUTPUT_DIR"

    # ROSBAG_DIRを設定(後で使用するため)
    ROSBAG_DIR=$(dirname "$ROSBAG")

    # pose_source=0 (ndt) で実行
    # --target_bagが指定されている場合はスキップ
    if [ -n "$TARGET_BAG" ]; then
        echo ""
        echo "--- Skipping pose_source=0 (ndt) recording (using --target_bag instead) ---"
        # target_bagのパスを確認(rosbag2ディレクトリかファイルか)
        if [ -d "$TARGET_BAG" ] && [ -f "$TARGET_BAG/metadata.yaml" ]; then
            TARGET_BAG_PATH="$TARGET_BAG"
        elif [ -f "$TARGET_BAG" ]; then
            # .db3ファイルが指定されている場合、親ディレクトリを探す
            TARGET_BAG_PARENT=$(dirname "$TARGET_BAG")
            METADATA_FILE=$(find "$TARGET_BAG_PARENT" -name "metadata.yaml" -type f 2>/dev/null | head -1)
            if [ -n "$METADATA_FILE" ]; then
                TARGET_BAG_PATH=$(dirname "$METADATA_FILE")
            else
                # ファイルが指定されている場合は、そのファイルの親ディレクトリを使用
                TARGET_BAG_PATH="$TARGET_BAG_PARENT"
            fi
        else
            # rosbag2ディレクトリを探す
            METADATA_FILE=$(find "$TARGET_BAG" -name "metadata.yaml" -type f 2>/dev/null | head -1)
            if [ -n "$METADATA_FILE" ]; then
                TARGET_BAG_PATH=$(dirname "$METADATA_FILE")
            else
                TARGET_BAG_PATH="$TARGET_BAG"
            fi
        fi
        echo "Using target bag: $TARGET_BAG_PATH"
    else
        echo ""
        echo "--- Running with pose_source=0 (ndt) ---"
        POSE_SOURCE_0_DIR="$ROSBAG_OUTPUT_DIR/pose_source_0_ndt"
        mkdir -p "$POSE_SOURCE_0_DIR"

        # 既存のrecorded_bagが存在するか確認
        if [ -d "$POSE_SOURCE_0_DIR/recorded_bag" ] && [ -f "$POSE_SOURCE_0_DIR/recorded_bag/metadata.yaml" ]; then
            echo "Existing recorded bag found for pose_source=0, skipping recording..."
            TARGET_BAG_PATH="$POSE_SOURCE_0_DIR/recorded_bag"
            echo "Using existing recorded bag: $TARGET_BAG_PATH"
        else
        cd "$AUTOWARE_DIR"

        # 既存のrecord_replay_*ディレクトリのリストを取得(削除対象を記録)
        EXISTING_RECORD_DIRS_BEFORE=($(ls -d "$ROSBAG_DIR"/record_replay_* 2>/dev/null || true))
        echo "Existing record directories before launch: ${#EXISTING_RECORD_DIRS_BEFORE[@]}"
        if [ ${#EXISTING_RECORD_DIRS_BEFORE[@]} -gt 0 ]; then
            echo "  Preserving existing directories:"
            for dir in "${EXISTING_RECORD_DIRS_BEFORE[@]}"; do
                echo "    - $dir"
            done
        fi

        echo "Starting Autoware with pose_source=0 (ndt)..."
    # launch_autoware.shはrosbagの再生が終了するまで待つ
    # バックグラウンドで実行して、OUTPUT_DIRが作成されるのを待つ
    # 引数の順序: <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE]
    if ! "$SCRIPT_DIR/launch_autoware.sh" "$MAP_PATH" "$ROSBAG" "0" "false" "output_lidar-marker" > "$POSE_SOURCE_0_DIR/launch.log" 2>&1; then
        echo "Error: Failed to run with pose_source=0"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=0 failed)")
        # Autowareを停止
        "$SCRIPT_DIR/../launch_replay_localization/kill_autoware.sh" || true
        sleep 5
        continue
    fi

    # 記録されたrosbagを探す(新しく作成されたrecord_replay_*ディレクトリ)
    # 少し待ってからディレクトリを探す(rosbag記録が完了するまで)
    echo "Waiting for rosbag recording to complete..."
    sleep 5
    # 最大30秒待つ(ディレクトリが作成されるまで)
    NEW_RECORD_DIR=""
    for i in {1..6}; do
        # 全てのrecord_replay_*ディレクトリを取得
        ALL_RECORD_DIRS=($(ls -td "$ROSBAG_DIR"/record_replay_* 2>/dev/null || true))
        # 既存リストにない新しいディレクトリを探す
        for dir in "${ALL_RECORD_DIRS[@]}"; do
            IS_NEW=true
            for existing_dir in "${EXISTING_RECORD_DIRS_BEFORE[@]}"; do
                if [ "$dir" = "$existing_dir" ]; then
                    IS_NEW=false
                    break
                fi
            done
            if [ "$IS_NEW" = true ]; then
                NEW_RECORD_DIR="$dir"
                break
            fi
        done
        if [ -n "$NEW_RECORD_DIR" ] && [ -d "$NEW_RECORD_DIR" ]; then
            break
        fi
        echo "  Waiting for record directory... ($i/6)"
        sleep 5
    done

    LATEST_RECORD_DIR="$NEW_RECORD_DIR"

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
    TARGET_BAG_PATH="$POSE_SOURCE_0_DIR/recorded_bag"

    # コピーが成功したら、今回作成されたrecord_replay_*ディレクトリのみを削除
    if [ -d "$TARGET_BAG_PATH" ] && [ -n "$(ls -A "$TARGET_BAG_PATH" 2>/dev/null)" ]; then
        echo "Copy successful. Cleaning up source directory: $LATEST_RECORD_DIR"
        rm -rf "$LATEST_RECORD_DIR"
        echo "  Removed: $LATEST_RECORD_DIR"
    fi

    # コピーが成功したか確認
    if [ ! -d "$TARGET_BAG_PATH" ] || [ -z "$(ls -A "$TARGET_BAG_PATH" 2>/dev/null)" ]; then
        echo "Error: Failed to copy recorded bag. Directory is empty or does not exist."
        echo "  TARGET_BAG_PATH: $TARGET_BAG_PATH"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=0 copy failed)")
        continue
    fi

    # rosbag2のmetadata.yamlが存在するか確認(.db3ファイル形式の場合は不要)
    if [ ! -f "$TARGET_BAG_PATH/metadata.yaml" ]; then
        # .db3ファイルが存在するか確認(SQLite3形式のrosbag2)
        if [ -n "$(find "$TARGET_BAG_PATH" -maxdepth 1 -name "*.db3" -type f 2>/dev/null | head -1)" ]; then
            echo "  Found .db3 file (SQLite3 format rosbag2): $TARGET_BAG_PATH"
            # .db3ファイル形式の場合はそのまま使用(metadata.yamlは不要)
        else
            echo "Warning: metadata.yaml not found in $TARGET_BAG_PATH"
            echo "  Looking for rosbag2 directory..."
            # サブディレクトリを探す
            METADATA_FILE=$(find "$TARGET_BAG_PATH" -name "metadata.yaml" -type f 2>/dev/null | head -1)
            if [ -n "$METADATA_FILE" ]; then
                TARGET_BAG_PATH=$(dirname "$METADATA_FILE")
                echo "  Found rosbag2 directory: $TARGET_BAG_PATH"
            else
                echo "Error: Could not find rosbag2 directory or .db3 file in $TARGET_BAG_PATH"
                FAIL_COUNT=$((FAIL_COUNT + 1))
                FAILED_BAGS+=("$ROSBAG (pose_source=0 rosbag not found)")
                continue
            fi
        fi
    fi

            echo "Recorded bag saved to: $TARGET_BAG_PATH"

            # Autowareを停止(念のため)
            "$SCRIPT_DIR/../launch_replay_localization/kill_autoware.sh" || true
            sleep 5
        fi
    fi

    # pose_source=1 (ndt_lidar-marker) で実行
    # --reference_bagが指定されている場合はスキップ
    if [ -n "$REFERENCE_BAG" ]; then
        echo ""
        echo "--- Skipping pose_source=1 (ndt_lidar-marker) recording (using --reference_bag instead) ---"
        # reference_bagのパスを確認(rosbag2ディレクトリかファイルか)
        if [ -d "$REFERENCE_BAG" ] && [ -f "$REFERENCE_BAG/metadata.yaml" ]; then
            REFERENCE_BAG_PATH="$REFERENCE_BAG"
        elif [ -f "$REFERENCE_BAG" ]; then
            # .db3ファイルが指定されている場合、親ディレクトリを探す
            REFERENCE_BAG_PARENT=$(dirname "$REFERENCE_BAG")
            METADATA_FILE=$(find "$REFERENCE_BAG_PARENT" -name "metadata.yaml" -type f 2>/dev/null | head -1)
            if [ -n "$METADATA_FILE" ]; then
                REFERENCE_BAG_PATH=$(dirname "$METADATA_FILE")
            else
                # ファイルが指定されている場合は、そのファイルの親ディレクトリを使用
                REFERENCE_BAG_PATH="$REFERENCE_BAG_PARENT"
            fi
        else
            # rosbag2ディレクトリを探す
            METADATA_FILE=$(find "$REFERENCE_BAG" -name "metadata.yaml" -type f 2>/dev/null | head -1)
            if [ -n "$METADATA_FILE" ]; then
                REFERENCE_BAG_PATH=$(dirname "$METADATA_FILE")
            else
                REFERENCE_BAG_PATH="$REFERENCE_BAG"
            fi
        fi
        echo "Using reference bag: $REFERENCE_BAG_PATH"
    else
        echo ""
        echo "--- Running with pose_source=1 (ndt_lidar-marker) ---"
        POSE_SOURCE_1_DIR="$ROSBAG_OUTPUT_DIR/pose_source_1_ndt_lidar_marker"
        mkdir -p "$POSE_SOURCE_1_DIR"

        # 既存のrecorded_bagが存在するか確認
        if [ -d "$POSE_SOURCE_1_DIR/recorded_bag" ] && [ -f "$POSE_SOURCE_1_DIR/recorded_bag/metadata.yaml" ]; then
            echo "Existing recorded bag found for pose_source=1, skipping recording..."
            REFERENCE_BAG_PATH="$POSE_SOURCE_1_DIR/recorded_bag"
            echo "Using existing recorded bag: $REFERENCE_BAG_PATH"
        else
        cd "$AUTOWARE_DIR"

        # 既存のrecord_replay_*ディレクトリのリストを取得(削除対象を記録)
        EXISTING_RECORD_DIRS_BEFORE=($(ls -d "$ROSBAG_DIR"/record_replay_* 2>/dev/null || true))
        echo "Existing record directories before launch: ${#EXISTING_RECORD_DIRS_BEFORE[@]}"
        if [ ${#EXISTING_RECORD_DIRS_BEFORE[@]} -gt 0 ]; then
            echo "  Preserving existing directories:"
            for dir in "${EXISTING_RECORD_DIRS_BEFORE[@]}"; do
                echo "    - $dir"
            done
        fi

        echo "Starting Autoware with pose_source=1 (ndt_lidar-marker)..."
    # launch_autoware.shはrosbagの再生が終了するまで待つ
    # 引数の順序: <MAP_PATH> <ROSBAG_PATH> [POSE_SOURCE_ID] [SAVE_LAUNCH_LOG] [TOPIC_TYPE]
    if ! "$SCRIPT_DIR/launch_autoware.sh" "$MAP_PATH" "$ROSBAG" "1" "false" "output_lidar-marker" > "$POSE_SOURCE_1_DIR/launch.log" 2>&1; then
        echo "Error: Failed to run with pose_source=1"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=1 failed)")
        # Autowareを停止
        "$SCRIPT_DIR/../launch_replay_localization/kill_autoware.sh" || true
        sleep 5
        continue
    fi

    # 記録されたrosbagを探す(新しく作成されたrecord_replay_*ディレクトリ)
    # 少し待ってからディレクトリを探す(rosbag記録が完了するまで)
    echo "Waiting for rosbag recording to complete..."
    sleep 5
    # 最大30秒待つ(ディレクトリが作成されるまで)
    NEW_RECORD_DIR=""
    for i in {1..6}; do
        # 全てのrecord_replay_*ディレクトリを取得
        ALL_RECORD_DIRS=($(ls -td "$ROSBAG_DIR"/record_replay_* 2>/dev/null || true))
        # 既存リストにない新しいディレクトリを探す
        for dir in "${ALL_RECORD_DIRS[@]}"; do
            IS_NEW=true
            for existing_dir in "${EXISTING_RECORD_DIRS_BEFORE[@]}"; do
                if [ "$dir" = "$existing_dir" ]; then
                    IS_NEW=false
                    break
                fi
            done
            if [ "$IS_NEW" = true ]; then
                NEW_RECORD_DIR="$dir"
                break
            fi
        done
        if [ -n "$NEW_RECORD_DIR" ] && [ -d "$NEW_RECORD_DIR" ]; then
            break
        fi
        echo "  Waiting for record directory... ($i/6)"
        sleep 5
    done

    LATEST_RECORD_DIR="$NEW_RECORD_DIR"

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
    REFERENCE_BAG_PATH="$POSE_SOURCE_1_DIR/recorded_bag"

    # コピーが成功したら、今回作成されたrecord_replay_*ディレクトリのみを削除
    if [ -d "$REFERENCE_BAG_PATH" ] && [ -n "$(ls -A "$REFERENCE_BAG_PATH" 2>/dev/null)" ]; then
        echo "Copy successful. Cleaning up source directory: $LATEST_RECORD_DIR"
        rm -rf "$LATEST_RECORD_DIR"
        echo "  Removed: $LATEST_RECORD_DIR"
    fi

    # コピーが成功したか確認
    if [ ! -d "$REFERENCE_BAG_PATH" ] || [ -z "$(ls -A "$REFERENCE_BAG_PATH" 2>/dev/null)" ]; then
        echo "Error: Failed to copy recorded bag. Directory is empty or does not exist."
        echo "  REFERENCE_BAG_PATH: $REFERENCE_BAG_PATH"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (pose_source=1 copy failed)")
        continue
    fi

    # rosbag2のmetadata.yamlが存在するか確認(.db3ファイル形式の場合は不要)
    if [ ! -f "$REFERENCE_BAG_PATH/metadata.yaml" ]; then
        # .db3ファイルが存在するか確認(SQLite3形式のrosbag2)
        if [ -n "$(find "$REFERENCE_BAG_PATH" -maxdepth 1 -name "*.db3" -type f 2>/dev/null | head -1)" ]; then
            echo "  Found .db3 file (SQLite3 format rosbag2): $REFERENCE_BAG_PATH"
            # .db3ファイル形式の場合はそのまま使用(metadata.yamlは不要)
        else
            echo "Warning: metadata.yaml not found in $REFERENCE_BAG_PATH"
            echo "  Looking for rosbag2 directory..."
            # サブディレクトリを探す
            METADATA_FILE=$(find "$REFERENCE_BAG_PATH" -name "metadata.yaml" -type f 2>/dev/null | head -1)
            if [ -n "$METADATA_FILE" ]; then
                REFERENCE_BAG_PATH=$(dirname "$METADATA_FILE")
                echo "  Found rosbag2 directory: $REFERENCE_BAG_PATH"
            else
                echo "Error: Could not find rosbag2 directory or .db3 file in $REFERENCE_BAG_PATH"
                FAIL_COUNT=$((FAIL_COUNT + 1))
                FAILED_BAGS+=("$ROSBAG (pose_source=1 rosbag not found)")
                continue
            fi
        fi
    fi

            echo "Recorded bag saved to: $REFERENCE_BAG_PATH"

            # Autowareを停止(念のため)
            "$SCRIPT_DIR/../launch_replay_localization/kill_autoware.sh" || true
            sleep 5
        fi
    fi

    # 比較スクリプトを実行
    echo ""
    echo "--- Running comparison script ---"
    COMPARISON_OUTPUT_DIR="$ROSBAG_OUTPUT_DIR/comparison_results"
    mkdir -p "$COMPARISON_OUTPUT_DIR"

    # rosbag2のパスを取得(recorded_bagディレクトリ内のrosbag2データベース)
    # rosbag2は通常、ディレクトリ内に複数のファイルがあるので、ディレクトリ自体をパスとして渡す
    if [ -z "$TARGET_BAG_PATH" ] || [ -z "$REFERENCE_BAG_PATH" ]; then
        echo "Error: Bag paths not set (TARGET_BAG_PATH=$TARGET_BAG_PATH, REFERENCE_BAG_PATH=$REFERENCE_BAG_PATH)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (bag paths not set)")
        continue
    fi

    if [ ! -d "$TARGET_BAG_PATH" ] && [ ! -f "$TARGET_BAG_PATH" ]; then
        echo "Error: Target bag not found: $TARGET_BAG_PATH"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (target bag not found)")
        continue
    fi

    if [ ! -d "$REFERENCE_BAG_PATH" ] && [ ! -f "$REFERENCE_BAG_PATH" ]; then
        echo "Error: Reference bag not found: $REFERENCE_BAG_PATH"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (reference bag not found)")
        continue
    fi

    # rosbag2ディレクトリの確認(metadata.yamlが存在するか、または.db3ファイルか)
    # .db3ファイル形式(SQLite3)の場合はmetadata.yamlは不要
    TARGET_BAG_VALID=false
    if [ -f "$TARGET_BAG_PATH/metadata.yaml" ]; then
        TARGET_BAG_VALID=true
        echo "Target bag: rosbag2 format with metadata.yaml"
    elif [ -n "$(find "$TARGET_BAG_PATH" -maxdepth 1 -name "*.db3" -type f 2>/dev/null | head -1)" ]; then
        TARGET_BAG_VALID=true
        echo "Target bag: SQLite3 format (.db3 file)"
    else
        METADATA_FILE=$(find "$TARGET_BAG_PATH" -name "metadata.yaml" -type f 2>/dev/null | head -1)
        if [ -n "$METADATA_FILE" ]; then
            TARGET_BAG_PATH=$(dirname "$METADATA_FILE")
            TARGET_BAG_VALID=true
            echo "Target bag: rosbag2 format with metadata.yaml (found in subdirectory)"
        fi
    fi

    REFERENCE_BAG_VALID=false
    if [ -f "$REFERENCE_BAG_PATH/metadata.yaml" ]; then
        REFERENCE_BAG_VALID=true
        echo "Reference bag: rosbag2 format with metadata.yaml"
    elif [ -n "$(find "$REFERENCE_BAG_PATH" -maxdepth 1 -name "*.db3" -type f 2>/dev/null | head -1)" ]; then
        REFERENCE_BAG_VALID=true
        echo "Reference bag: SQLite3 format (.db3 file)"
    else
        METADATA_FILE=$(find "$REFERENCE_BAG_PATH" -name "metadata.yaml" -type f 2>/dev/null | head -1)
        if [ -n "$METADATA_FILE" ]; then
            REFERENCE_BAG_PATH=$(dirname "$METADATA_FILE")
            REFERENCE_BAG_VALID=true
            echo "Reference bag: rosbag2 format with metadata.yaml (found in subdirectory)"
        fi
    fi

    if [ "$TARGET_BAG_VALID" = false ]; then
        echo "Error: Could not validate target bag format: $TARGET_BAG_PATH"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (target bag format not recognized)")
        continue
    fi

    if [ "$REFERENCE_BAG_VALID" = false ]; then
        echo "Error: Could not validate reference bag format: $REFERENCE_BAG_PATH"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (reference bag format not recognized)")
        continue
    fi

    # Pythonスクリプトを実行
    # lidar_marker_localizerあり(pose_source=1)を基準、なし(pose_source=0)を比較として設定
    COMPARE_SCRIPT="$HOME/scripts_for_autoware/py/compare_localization_diff.py"
    if [ ! -f "$COMPARE_SCRIPT" ]; then
        echo "Error: Comparison script not found: $COMPARE_SCRIPT"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_BAGS+=("$ROSBAG (comparison script not found)")
        continue
    fi

    if python3 "$COMPARE_SCRIPT" \
        --reference_bag "$REFERENCE_BAG_PATH" \
        --target_bag "$TARGET_BAG_PATH" \
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
