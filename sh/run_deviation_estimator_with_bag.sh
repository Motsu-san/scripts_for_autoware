#!/bin/bash
#
# deviation_estimator を起動し、指定 rosbag を再生して推定完了後に launch を終了する。
# 手順書「パラメータ推定と精度検証」相当の作業を1本化する。
#
# Usage:
#   ./run_deviation_estimator_with_bag.sh <ROSBAG_PATH>
#
# 環境変数（任意）:
#   PILOT_AUTO_WS   pilot-auto ワークスペース（既定: 実行時のカレントディレクトリ。install/ がある場所に cd してから実行）
#   IMU_TOPIC       in_imu リマップ先（既定: /sensing/imu/tamagawa/imu_raw）
#   POSE_TOPIC      事前チェック用（既定: /localization/pose_estimator/pose_with_covariance）。launch を in_pose で変えたら合わせる
#   WHEEL_TOPIC     事前チェック用（既定: /vehicle/status/velocity_status）。launch を in_wheel で変えたら合わせる
#   PLAY_CLOCK_HZ   bag に /clock が無いとき ros2 bag play に渡す --clock（既定: 100）
#   USE_SIM_TIME    launch の use_sim_time（既定: true）
#   EXTRA_LAUNCH_ARGS  ros2 launch 末尾にそのまま追加する引数（例: in_pose_with_cov_name:=/foo）
#   LAUNCH_CLEANUP_TIMEOUT_SEC  bag 再生後に ros2 launch を SIGINT してから SIGKILL まで待つ秒数（既定: 15）
#   SKIP_BAG_TOPIC_CHECK  1 でトピック事前チェックをスキップ
#   STRICT_BAG_TOPICS     1 で必須トピック欠落時に即 exit 1（既定は警告のみで続行）
#
# rosbag に必要なもの（欠けると deviation_estimator が ERROR/WARN になる）:
#   - IMU（既定トピック上の sensor_msgs/Imu）
#   - 車輪速系（既定: /vehicle/status/velocity_status）
#   - ポーズ（既定: /localization/pose_estimator/pose_with_covariance）
#   - TF: base_link と IMU リンク（例: tamagawa/imu_link）を結ぶ変換が /tf または /tf_static で再生時に解決できること
#   localization 再生だけの bag では sensing/vehicle や TF が無く、上記のようなログになることがある。
#
# 出力:
#   <rosbag ディレクトリ>/param_estimation_result_<YYYYMMDD_HHMMSS>/ に results_dir を設定

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALL_DIR="$(pwd)"
PILOT_AUTO_WS="${PILOT_AUTO_WS:-$CALL_DIR}"
IMU_TOPIC="${IMU_TOPIC:-/sensing/imu/tamagawa/imu_raw}"
POSE_TOPIC="${POSE_TOPIC:-/localization/pose_estimator/pose_with_covariance}"
WHEEL_TOPIC="${WHEEL_TOPIC:-/vehicle/status/velocity_status}"
PLAY_CLOCK_HZ="${PLAY_CLOCK_HZ:-100}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
LAUNCH_CLEANUP_TIMEOUT_SEC="${LAUNCH_CLEANUP_TIMEOUT_SEC:-15}"

usage() {
    echo "Usage: $0 <ROSBAG_PATH>" >&2
    echo "" >&2
    echo "  ROSBAG_PATH : rosbag2 ディレクトリ、または metadata.yaml / .db3 のパス" >&2
    echo "" >&2
    echo "環境変数: PILOT_AUTO_WS IMU_TOPIC POSE_TOPIC WHEEL_TOPIC PLAY_CLOCK_HZ USE_SIM_TIME EXTRA_LAUNCH_ARGS" >&2
    echo "  LAUNCH_CLEANUP_TIMEOUT_SEC SKIP_BAG_TOPIC_CHECK STRICT_BAG_TOPICS" >&2
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    usage
    exit 0
fi

if [ $# -lt 1 ]; then
    usage
    exit 1
fi

ROSBAG="$1"

if [ ! -f "$ROSBAG" ] && [ ! -d "$ROSBAG" ]; then
    echo "Error: ROSBAG が見つかりません: $ROSBAG" >&2
    exit 1
fi

# rosbag の「親フォルダ」＝出力の基準ディレクトリ
if [ -d "$ROSBAG" ]; then
    ROSBAG_DIR="$ROSBAG"
else
    ROSBAG_DIR=$(dirname "$ROSBAG")
fi

INSTALL_SETUP="$PILOT_AUTO_WS/install/local_setup.bash"
if [ ! -f "$INSTALL_SETUP" ]; then
    echo "Error: pilot-auto の install が見つかりません: $INSTALL_SETUP" >&2
    echo "  ビルド済みワークスペースのルートで実行するか、PILOT_AUTO_WS にそのパスを指定してください。" >&2
    echo "  （既定の PILOT_AUTO_WS は実行時のカレント: $CALL_DIR）" >&2
    exit 1
fi

# ROS 2 ベース（あれば）→ pilot-auto
# set -u のまま source すると setup.bash 内の AMENT_TRACE_SETUP_FILES 等で落ちるため一時的に off
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
fi
# shellcheck source=/dev/null
source "$INSTALL_SETUP"
set -u

# bag に /clock がある場合は --clock を付けない（二重クロック防止）
# 他トピックと同じ bag_lists_topic で判定（表記ゆれで取りこぼさない）
bag_has_clock_topic() {
    bag_lists_topic "$1" "/clock"
}

# ros2 bag info の表記ゆれに対応してトピック名が一覧に含まれるか判定
# humble 典型: "Topic: /foo | Type: ..." および "| /foo |" 形式の両方
bag_lists_topic() {
    local bag_path="$1" topic="$2"
    ros2 bag info "$bag_path" 2>/dev/null | awk -v t="$topic" '
        index($0, "Topic: " t " |") > 0 { found = 1 }
        index($0, "Topic: " t "\t|") > 0 { found = 1 }
        index($0, "| " t " |") > 0 { found = 1 }
        index($0, "|" t " |") > 0 { found = 1 }
        index($0, "| " t "|") > 0 { found = 1 }
        index($0, "|" t "|") > 0 { found = 1 }
        END { exit(found ? 0 : 1) }
    '
}

# deviation_estimator / TF 前提の事前チェック（欠落は警告。STRICT_BAG_TOPICS=1 で失敗扱い）
check_bag_prerequisites() {
    if [ "${SKIP_BAG_TOPIC_CHECK:-0}" = "1" ]; then
        echo "Info: SKIP_BAG_TOPIC_CHECK=1 のためトピック事前チェックを省略します"
        return 0
    fi
    local missing=()
    bag_lists_topic "$ROSBAG" "$IMU_TOPIC" || missing+=("IMU: $IMU_TOPIC")
    # deviation_estimator in_wheel 既定: /vehicle/status/velocity_status（欠落時は No wheel odometry）
    if ! bag_lists_topic "$ROSBAG" "$WHEEL_TOPIC"; then
        if [ "$WHEEL_TOPIC" = "/vehicle/status/velocity_status" ]; then
            missing+=("vehicle(status): /vehicle/status/velocity_status (in_wheel 既定・欠けると No wheel odometry)")
        else
            missing+=("wheel (in_wheel): $WHEEL_TOPIC")
        fi
    fi
    bag_lists_topic "$ROSBAG" "$POSE_TOPIC" || missing+=("pose: $POSE_TOPIC")
    local tf_ok=0
    bag_lists_topic "$ROSBAG" "/tf" && tf_ok=1
    bag_lists_topic "$ROSBAG" "/tf_static" && tf_ok=1

    if [ "$tf_ok" -eq 0 ]; then
        echo "Warning: bag に /tf も /tf_static も見当たりません。" \
            "base_link〜IMU リンク（例: tamagawa/imu_link）の変換が再生時に解決できず、" \
            "「Please publish TF base_link to tamagawa/imu_link」等になることがあります。" >&2
        [ "${STRICT_BAG_TOPICS:-0}" = "1" ] && missing+=("TF: /tf または /tf_static")
    fi

    if [ ${#missing[@]} -eq 0 ]; then
        return 0
    fi

    echo "Warning: rosbag に deviation_estimator 用トピックが無いか、名前が一致しません:" >&2
    printf '  - %s\n' "${missing[@]}" >&2
    echo "  → localization だけの再記録 bag では /vehicle/... や IMU、/tf_static が欠けることがあります。" >&2
    echo "  → 元の sensing + vehicle + TF を含む bag を使うか、別 bag を同時再生してトピックを足してください。" >&2
    if [ "${STRICT_BAG_TOPICS:-0}" = "1" ]; then
        echo "Error: STRICT_BAG_TOPICS=1 のため中止します。" >&2
        exit 1
    fi
}

CLOCK_ARGS=()
if bag_has_clock_topic "$ROSBAG"; then
    echo "Info: bag に /clock あり → ros2 bag play は --clock なし"
else
    CLOCK_ARGS=(--clock "$PLAY_CLOCK_HZ")
    echo "Info: bag に /clock なし → ros2 bag play ${CLOCK_ARGS[*]}"
fi

check_bag_prerequisites

OUTPUT_DIR="$ROSBAG_DIR/param_estimation_result_$(date '+%Y%m%d_%H%M%S')"
mkdir -p "$OUTPUT_DIR"
echo "結果出力先 (results_dir): $OUTPUT_DIR"

ESTIMATOR_PID=""
# クリーンアップ中に Ctrl+C が来ると INT トラップが再入り、メッセージが二重になる。
# 再入防止 + 処理中は INT を無視し、ros2 launch には PG 全体へ SIGINT/SIGKILL を送る。
cleanup_estimator() {
    [ "${_ESTIMATOR_CLEANUP_DONE:-}" = "1" ] && return 0
    if [ -z "${ESTIMATOR_PID:-}" ] || ! kill -0 "$ESTIMATOR_PID" 2>/dev/null; then
        return 0
    fi
    _ESTIMATOR_CLEANUP_DONE=1

    trap '' INT
    trap '' TERM

    echo "deviation_estimator を終了します (PID $ESTIMATOR_PID)..."
    # バックグラウンドジョブの PGID は多くの場合 $ESTIMATOR_PID と一致。setsid 使用時も同様。
    kill -INT -- -"$ESTIMATOR_PID" 2>/dev/null || kill -INT "$ESTIMATOR_PID" 2>/dev/null || true
    # ros2 launch は子ノード終了まで戻らないことがあり、ここで wait だけだと長時間ブロックしがち
    local _max=$((LAUNCH_CLEANUP_TIMEOUT_SEC * 2))
    local _i=0
    while kill -0 "$ESTIMATOR_PID" 2>/dev/null && [ "$_i" -lt "$_max" ]; do
        sleep 0.5
        _i=$((_i + 1))
    done
    if kill -0 "$ESTIMATOR_PID" 2>/dev/null; then
        echo "Warning: ${LAUNCH_CLEANUP_TIMEOUT_SEC}s 以内に終了しなかったため SIGKILL します (PID $ESTIMATOR_PID)" >&2
        kill -KILL -- -"$ESTIMATOR_PID" 2>/dev/null || kill -KILL "$ESTIMATOR_PID" 2>/dev/null || true
    fi
    wait "$ESTIMATOR_PID" 2>/dev/null || true

    trap 'cleanup_estimator' EXIT INT TERM HUP
}
trap cleanup_estimator EXIT INT TERM HUP

LOG_FILE="$OUTPUT_DIR/deviation_estimator_launch.log"
echo "ros2 launch ログ: $LOG_FILE"

# deviation_estimator を先に起動 → bag 再生 → 終了後に cleanup で launch を止める
# setsid があれば新セッション化し、終了時に PG 単位でシグナルが届きやすくする。
# 注: パイプやプロセス置換で & すると $! が子ではなく tee 側になるため、リダイレクトのみにする。
if command -v setsid >/dev/null 2>&1; then
    setsid ros2 launch deviation_estimator deviation_estimator.launch.xml \
        "in_imu:=$IMU_TOPIC" \
        "results_dir:=$OUTPUT_DIR" \
        "use_sim_time:=$USE_SIM_TIME" \
        ${EXTRA_LAUNCH_ARGS:-} \
        >"$LOG_FILE" 2>&1 &
else
    ros2 launch deviation_estimator deviation_estimator.launch.xml \
        "in_imu:=$IMU_TOPIC" \
        "results_dir:=$OUTPUT_DIR" \
        "use_sim_time:=$USE_SIM_TIME" \
        ${EXTRA_LAUNCH_ARGS:-} \
        >"$LOG_FILE" 2>&1 &
fi
ESTIMATOR_PID=$!

sleep 2

echo "ros2 bag play を開始: $ROSBAG"
set +e
ros2 bag play "$ROSBAG" "${CLOCK_ARGS[@]}"
PLAY_EXIT=$?
set -e

if [ "$PLAY_EXIT" -ne 0 ]; then
    echo "Warning: ros2 bag play が非ゼロ終了 (exit $PLAY_EXIT)" >&2
fi

cleanup_estimator
trap - EXIT INT TERM HUP
ESTIMATOR_PID=""

echo "完了。出力: $OUTPUT_DIR"
exit "$PLAY_EXIT"
