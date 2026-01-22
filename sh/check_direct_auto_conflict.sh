#!/bin/bash
# DIRECTモードとAUTOモードの両方が実行されているかをチェックするスクリプト

LOG_DIR="$HOME/log"
LOG_PATTERN="DLR_replay_*.log"

echo "=========================================="
echo "DIRECTモードとAUTOモードの競合をチェック中..."
echo "ログディレクトリ: ${LOG_DIR}"
echo "=========================================="
echo ""

# チェックするメッセージ
DIRECT_MSG="Set user defined initial pose"
AUTO_MSG="fit_target: pointcloud_map, frame: map"

CONFLICT_COUNT=0
DIRECT_ONLY_COUNT=0
AUTO_ONLY_COUNT=0
NEITHER_COUNT=0
TOTAL_LOGS=0

# 各ログファイルをチェック
for file in ${LOG_DIR}/${LOG_PATTERN}; do
    if [ ! -f "$file" ]; then
        continue
    fi

    TOTAL_LOGS=$((TOTAL_LOGS + 1))

    HAS_DIRECT=false
    HAS_AUTO=false

    # DIRECTモードのメッセージをチェック
    if grep -q "$DIRECT_MSG" "$file" 2>/dev/null; then
        HAS_DIRECT=true
    fi

    # AUTOモードのメッセージをチェック
    if grep -q "$AUTO_MSG" "$file" 2>/dev/null; then
        HAS_AUTO=true
    fi

    # カウント
    if [ "$HAS_DIRECT" = true ] && [ "$HAS_AUTO" = true ]; then
        CONFLICT_COUNT=$((CONFLICT_COUNT + 1))
        if [ $CONFLICT_COUNT -le 10 ]; then
            echo "✓ 競合あり: $(basename $file)"
            echo "  - DIRECTモード: あり"
            echo "  - AUTOモード: あり"
            echo ""
        fi
    elif [ "$HAS_DIRECT" = true ]; then
        DIRECT_ONLY_COUNT=$((DIRECT_ONLY_COUNT + 1))
    elif [ "$HAS_AUTO" = true ]; then
        AUTO_ONLY_COUNT=$((AUTO_ONLY_COUNT + 1))
    else
        NEITHER_COUNT=$((NEITHER_COUNT + 1))
    fi
done

echo "=========================================="
echo "検索結果サマリー"
echo "=========================================="
echo "総ログファイル数: ${TOTAL_LOGS}"
echo ""
echo "【競合あり】DIRECTモードとAUTOモードの両方が実行: ${CONFLICT_COUNT}件"
echo "【DIRECTのみ】DIRECTモードのみ実行: ${DIRECT_ONLY_COUNT}件"
echo "【AUTOのみ】AUTOモードのみ実行: ${AUTO_ONLY_COUNT}件"
echo "【なし】どちらも実行されていない: ${NEITHER_COUNT}件"
echo ""

# 競合があるファイルの一覧を表示（最初の20件）
if [ ${CONFLICT_COUNT} -gt 0 ]; then
    echo "=========================================="
    echo "競合があるファイル一覧（最初の20件）"
    echo "=========================================="
    COUNT=0
    for file in ${LOG_DIR}/${LOG_PATTERN}; do
        if [ ! -f "$file" ]; then
            continue
        fi

        if grep -q "$DIRECT_MSG" "$file" 2>/dev/null && grep -q "$AUTO_MSG" "$file" 2>/dev/null; then
            COUNT=$((COUNT + 1))
            echo "$(basename $file)"
            if [ $COUNT -ge 20 ]; then
                break
            fi
        fi
    done

    if [ ${CONFLICT_COUNT} -gt 20 ]; then
        echo "... 他 ${CONFLICT_COUNT} - 20 件"
    fi
fi

echo ""
echo "検索完了"
