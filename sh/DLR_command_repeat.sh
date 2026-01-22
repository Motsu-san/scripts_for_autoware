#!/bin/bash
# DLR_command_parallel.shを繰り返し実行するスクリプト

REPEAT_COUNT=$1

# Set traps to stop all background processes when the script exits
trap "$HOME/scripts_for_autoware/sh/kill_autoware.sh" EXIT INT TERM HUP

echo "=========================================="
echo "DLR_command_parallel.shを${REPEAT_COUNT}回実行します"
echo "=========================================="

# 繰り返し実行
for i in $(seq 1 ${REPEAT_COUNT}); do
    DATETIME=$(date '+%Y%m%d_%H%M%S')

    echo ""
    echo "=========================================="
    echo "実行 ${i}/${REPEAT_COUNT}"
    echo "開始時刻: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=========================================="

    # 実行
    $HOME/sh/DLR_command_parallel.sh # 負荷的な制限により引数なしでデフォルト固定

    $HOME/scripts_for_autoware/sh/kill_autoware.sh

    # 次の実行まで少し待機（オプション）
    if [ ${i} -lt ${REPEAT_COUNT} ]; then
        echo "次の実行まで3秒待機..."
        sleep 3
    fi
done

echo ""
echo "=========================================="
echo "全${REPEAT_COUNT}回の実行が完了しました"
echo "終了時刻: $(date '+%Y-%m-%d %H:%M:%S')"
echo "=========================================="
echo ""
