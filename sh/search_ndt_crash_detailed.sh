#!/bin/bash
# ndt_scan_matcherのクラッシュを詳細に検索するスクリプト

LOG_DIR="$HOME/log/DLR_replay_20251224_logs"
LOG_PATTERN="DLR_replay_*.log"

echo "=========================================="
echo "ndt_scan_matcherのクラッシュを詳細検索中..."
echo "ログディレクトリ: ${LOG_DIR}"
echo "=========================================="
echo ""

# 1. autoware_ndt_scan_matcher_nodeのprocess has diedを直接検索
echo "【1】autoware_ndt_scan_matcher_nodeのprocess has diedを検索"
echo "----------------------------------------"
CRASH_FILES=$(grep -l "autoware_ndt_scan_matcher_node" ${LOG_DIR}/${LOG_PATTERN} 2>/dev/null | while read file; do
    if grep -q "process has died" "$file"; then
        # 同じ行または近くにndt_scan_matcherがあるか確認
        if grep -B 2 -A 2 "process has died" "$file" | grep -q "ndt_scan_matcher\|autoware_ndt_scan_matcher"; then
            echo "$file"
        fi
    fi
done)

if [ -z "$CRASH_FILES" ]; then
    echo "見つかりませんでした"
else
    echo "以下のファイルで見つかりました:"
    echo "$CRASH_FILES" | while read file; do
        echo ""
        echo "ファイル: $(basename $file)"
        echo "---"
        # 該当行の前後10行を表示
        grep -B 10 -A 10 "process has died" "$file" | grep -B 10 -A 10 "ndt_scan_matcher\|autoware_ndt_scan_matcher" | head -25
        echo "---"
    done
fi
echo ""

# 2. exit code -11 または 139 を検索
echo "【2】exit code -11 (SIGSEGV) または 139 を検索"
echo "----------------------------------------"
EXIT_CODE_FILES=$(grep -l "exit code.*-11\|exit code.*139" ${LOG_DIR}/${LOG_PATTERN} 2>/dev/null)
if [ -z "$EXIT_CODE_FILES" ]; then
    echo "見つかりませんでした"
else
    echo "以下のファイルで見つかりました:"
    echo "$EXIT_CODE_FILES" | while read file; do
        echo ""
        echo "ファイル: $(basename $file)"
        echo "---"
        grep -B 5 -A 5 "exit code.*-11\|exit code.*139" "$file" | head -15
        echo "---"
    done
fi
echo ""

# 3. サマリー
echo "=========================================="
echo "検索サマリー"
echo "=========================================="

TOTAL_LOGS=$(ls -1 ${LOG_DIR}/${LOG_PATTERN} 2>/dev/null | wc -l)
echo "総ログファイル数: ${TOTAL_LOGS}"

# ndt_scan_matcher関連のprocess has died
NDT_CRASH_COUNT=$(grep -l "autoware_ndt_scan_matcher_node\|ndt_scan_matcher_node" ${LOG_DIR}/${LOG_PATTERN} 2>/dev/null | while read file; do
    if grep -q "process has died" "$file"; then
        if grep -B 2 -A 2 "process has died" "$file" | grep -q "ndt_scan_matcher\|autoware_ndt_scan_matcher"; then
            echo "1"
        fi
    fi
done | wc -l)

echo "ndt_scan_matcher関連のprocess has died: ${NDT_CRASH_COUNT}件"

# exit code -11 または 139
EXIT_CODE_COUNT=$(grep -l "exit code.*-11\|exit code.*139" ${LOG_DIR}/${LOG_PATTERN} 2>/dev/null | wc -l)
echo "exit code -11 または 139: ${EXIT_CODE_COUNT}件"

echo ""
echo "検索完了"
