#!/bin/bash

# トピック名を引数から取得（デフォルト: /localization/util/downsample/pointcloud）
TOPIC="${1:-/localization/util/downsample/pointcloud}"

echo "Starting timestamp regression checker..."
echo "Topic: $TOPIC"
echo "Press Ctrl+C to stop"
echo "============================================"

current_sec=""
current_nsec=""
prev_timestamp=""
count=0
regression_count=0
waiting_for_nanosec=false

ros2 topic echo "$TOPIC" --field header | while IFS= read -r line; do
    # 空行や区切り行をスキップ
    if [[ $line =~ ^---?$ ]] || [[ -z "${line// }" ]]; then
        continue
    fi

    # secの行を検出: "  sec: 1745384282"
    if [[ $line =~ ^[[:space:]]*sec:[[:space:]]*([0-9]+) ]]; then
        current_sec="${BASH_REMATCH[1]}"
        waiting_for_nanosec=true
        # echo "Debug: Found sec: $current_sec" >&2
    fi

    # nanosecの行を検出: "  nanosec: 947568128"
    if [[ $line =~ ^[[:space:]]*nanosec:[[:space:]]*([0-9]+) ]] && [[ "$waiting_for_nanosec" == true ]]; then
        current_nsec="${BASH_REMATCH[1]}"
        waiting_for_nanosec=false
        # echo "Debug: Found nanosec: $current_nsec" >&2

        # sec と nanosec が両方揃ったら処理
        if [[ -n "$current_sec" && -n "$current_nsec" ]]; then
            count=$((count + 1))

            # タイムスタンプを秒に変換
            if command -v bc >/dev/null 2>&1; then
                current_timestamp=$(echo "scale=9; $current_sec + $current_nsec / 1000000000" | bc)
            else
                current_timestamp=$(awk "BEGIN {printf \"%.9f\", $current_sec + $current_nsec/1000000000}")
            fi

            echo "[$count] Timestamp: $current_sec.$(printf "%09d" $current_nsec) (${current_timestamp}s)"

            if [[ -n "$prev_timestamp" ]]; then
                # 差分を計算
                if command -v bc >/dev/null 2>&1; then
                    delta=$(echo "scale=9; $current_timestamp - $prev_timestamp" | bc)
                    is_negative=$(echo "$delta < 0" | bc)
                else
                    delta=$(awk "BEGIN {printf \"%.9f\", $current_timestamp - $prev_timestamp}")
                    is_negative=$(awk "BEGIN {print ($delta < 0) ? 1 : 0}")
                fi

                if [[ "$is_negative" == "1" ]]; then
                    regression_count=$((regression_count + 1))
                    echo "*** REGRESSION DETECTED! ***"
                    echo "  Previous: $prev_timestamp"
                    echo "  Current:  $current_timestamp"
                    echo "  Delta:    ${delta}s"
                    echo "  Total regressions: $regression_count"
                    echo "============================================"
                else
                    echo "  Delta: ${delta}s (OK)"
                fi
            else
                echo "  First message"
            fi

            prev_timestamp="$current_timestamp"
            current_sec=""
            current_nsec=""
        fi
    fi
done
