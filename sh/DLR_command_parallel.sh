#!/bin/bash
# driving_log_replayer_v2を並列実行するスクリプト
# ROS_DOMAIN_IDを変更して複数インスタンスを同時実行

PARALLEL_COUNT=2  # 並列実行数（デフォルトかつ推奨: 2 これ以上は安定性に懸念がある）
TIMEOUT_SECONDS=1300

CALL_DIR=$(pwd)
SCENARIO_PATH="$HOME/your_scenario.yml"
DATASET_PATH="$HOME/your_dataset_path/"
DATASET_ID=$(basename $(dirname "$DATASET_PATH"))

# 引数で並列数を指定可能
if [ $# -ge 1 ]; then
    PARALLEL_COUNT=$1
fi

source $CALL_DIR/install/setup.bash

# Set traps to stop all background processes when the script exits
cleanup() {
    echo "Cleaning up background processes..."
    # 各インスタンスのPIDをkill（プロセスグループごと）
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Stopping instance with PID: $pid (and its process group)"
            # プロセスグループ全体をkill（負のPIDを使用）
            # まずTERMシグナルで正常終了を試みる
            kill -TERM -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
        fi
    done
    # 子プロセスが終了するまで少し待つ
    sleep 2
    # まだ生きているプロセスを強制終了
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL -"$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
        fi
    done
}
trap cleanup EXIT INT TERM HUP

echo "=========================================="
echo "driving_log_replayer_v2を${PARALLEL_COUNT}インスタンス並列実行します"
echo "各実行は${TIMEOUT_SECONDS}秒でタイムアウトします"
echo "=========================================="

# バックグラウンドで並列実行
PIDS=()
for i in $(seq 1 ${PARALLEL_COUNT}); do
    DATETIME=$(date '+%Y%m%d_%H%M%S')

    # 各インスタンスに異なるROS_DOMAIN_IDを設定
    # 現在のROS_DOMAIN_ID=3から開始して、インスタンスごとに+1
    DOMAIN_ID=$((3 + i - 1))

    echo ""
    echo "=========================================="
    echo "インスタンス ${i}/${PARALLEL_COUNT} を開始"
    echo "ROS_DOMAIN_ID=${DOMAIN_ID}"
    echo "開始時刻: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=========================================="

    # 出力ディレクトリをインスタンスごとに分ける
    OUTPUT_DIR="$HOME/pilot-auto/output/parallel_${i}_${DATETIME}"
    mkdir -p "${OUTPUT_DIR}"

    # 環境変数を設定してバックグラウンドで実行
    # プロセスグループを新規作成（setsidを使用）
    (
        export ROS_DOMAIN_ID=${DOMAIN_ID}
        export CYCLONEDDS_URI=file:///opt/autoware/cyclonedds_config.xml
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

        if [ "$TIMEOUT_SECONDS" -eq 0 ]; then
            ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
              scenario_path:=$SCENARIO_PATH \
              dataset_path:=$DATASET_PATH \
              DATASET_id:=$DATASET_ID \
              output_dir:=${OUTPUT_DIR} \
              rviz:=false \
              > $HOME/log/DLR_replay_parallel_${i}_${DATETIME}.log 2>&1
        else
            timeout $TIMEOUT_SECONDS ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
              scenario_path:=$SCENARIO_PATH \
              dataset_path:=$DATASET_PATH \
              DATASET_id:=$DATASET_ID \
              output_dir:=${OUTPUT_DIR} \
              rviz:=false \
              > $HOME/log/DLR_replay_parallel_${i}_${DATETIME}.log 2>&1
        fi

        EXIT_CODE=${PIPESTATUS[0]}
        echo "[Instance ${i}] Exit code: ${EXIT_CODE}" >> $HOME/log/DLR_replay_parallel_${i}_${DATETIME}.log
    ) &

    # プロセスグループIDを取得（setsidで新規作成された場合）
    PID=$!
    PIDS+=($PID)
    echo "インスタンス ${i} のPID: $PID"

    # インスタンス間で少し間隔を空ける（リソース競合を避ける）
    sleep 2
done

echo ""
echo "=========================================="
echo "全${PARALLEL_COUNT}インスタンスを開始しました"
echo "PID: ${PIDS[@]}"
echo "=========================================="
echo ""
echo "進捗を監視中... (Ctrl+Cで全インスタンスを停止)"

# すべてのプロセスが終了するまで待機
wait

echo ""
echo "=========================================="
echo "全${PARALLEL_COUNT}インスタンスの実行が完了しました"
echo "終了時刻: $(date '+%Y-%m-%d %H:%M:%S')"
echo "=========================================="
echo ""
echo "ログは以下のディレクトリに保存されています:"
ls -1 $HOME/log/DLR_replay_parallel_*_*.log 2>/dev/null | tail -${PARALLEL_COUNT}
