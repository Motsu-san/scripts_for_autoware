#!/usr/bin/env bash
# N 回 launch_autoware.sh で元 rosbag を再生し、各回の記録（TOPIC_TYPE=output_pose_mean）から
# 指定時刻に最も近い pose（既定: /localization/pose_estimator/pose_with_covariance）を抽出し平均する。
# 結果は JSON に加え、mean_pose.yaml（initial_pose 互換 pose + メタデータ）に出力する。
#
# 再開: 記録済み bag パスを rosbag と同じディレクトリ上の状態ファイルに保存する。
#   --resume   状態ファイルがあれば検証し、不足分だけ再生・記録を続けてから集計
#   --reset    状態ファイルを削除して最初からやり直し
#   --aggregate-only  再生なし。状態ファイルに列挙された bag のみで集計（記録済みのみのとき）
#
# Usage:
#   cd <autoware_ws>
#   AUTOWARE_WS=$PWD ./measure_pose_mean.sh [options] <MAP_PATH> <SOURCE_ROSBAG> <TARGET_UNIX_SEC> [N_RUNS]
#
# Options:
#   --resume, --reset, --aggregate-only, --force-sample-vehicle, -h, --help
#   MEASURE_RESUME=1 は --resume と同義（環境変数）
#
# 環境変数:
#   AUTOWARE_WS          既定: 実行時の pwd
#   PLAYBACK_RATE        既定: 1.0
#   GT_POSE_TOPIC        aggregate_pose_mean_from_bags.py の --pose-topic（既定: NDT pose_with_covariance）
#   MEAN_POSE_YAML       出力 YAML パス（省略時は出力ディレクトリ内 mean_pose.yaml）
#   MEAN_POSE_OUTPUT_DIR 集計結果のディレクトリ（省略時は STATE_FILE_DIR/mean_pose_YYYYMMDD_hhmmss を新規作成）
#   INITIAL_POSE_YAML    initial_pose.yaml（任意。既定は STATE_DIR 配下）
#   GT_AGG_JSON          出力 JSON パス
#   EXTRA_LAUNCH_ARGS    launch 末尾に追加（--gnss-receiver は bag 自動判定の後に付くため、上書きしたい場合に指定）
#   SKIP_LAUNCH=1        従来どおり BAGS_LIST_FILE を使用（状態ファイルは使わない）
#   --aggregate-only 時は GT_POSE_TOPIC を状態ファイルと変えても可（集計のみのため）。
#   ALIGN_POINTCLOUD_TOPIC  集計時に点群 header でフレームを揃える（既定: downsample 点群）。
#                           空にすると従来どおり target のみで pose を選ぶ。
#   MAX_POSE_POINTCLOUD_DT_SEC  点群アライン時、pose と点群 header の時刻差がこの秒数を超える run を平均から除外。
#                               例: 0.01〜0.02（厳しめ）、0.05（緩め・約0.1s欠けを通す可能性あり）。未設定で除外なし。
#
# 呼び出す launch_autoware.sh には常に --gnss-receiver <ublox|septentrio>（SOURCE_ROSBAG の ros2 bag info から推定）。
# --force-sample-vehicle は本スクリプトのオプションで指定したときのみ付与。
# 状態ファイル（.measure_pose_mean_*.txt）は rosbag と同じ場所: .db3 ならその親ディレクトリ、ディレクトリ形式 bag ならそのディレクトリ内。
# 既定の initial_pose.yaml / mean_pose 出力の親は従来どおり: 親が record_replay_* のときはその親（例: final_merged）。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_SCRIPT="$SCRIPT_DIR/../launch_replay_localization/launch_autoware.sh"
AGGREGATE_PY="$SCRIPT_DIR/../measure_ndt_pose_mean/aggregate_pose_mean_from_bags.py"
AUTOWARE_WS="${AUTOWARE_WS:-$(pwd)}"

# ros2 bag info から NavSatFix トピック名で ublox / septentrio を推定（launch_autoware.sh の --gnss-receiver 用）
detect_gnss_receiver_from_bag() {
    local bag="$1"
    local info
    if [[ ! -e "$bag" ]]; then
        echo "Error: rosbag が存在しません: $bag" >&2
        exit 1
    fi
    if ! command -v ros2 &>/dev/null; then
        echo "Error: ros2 が見つかりません。先に source $AUTOWARE_WS/install/setup.bash するか、AUTOWARE_WS を正しく指定してください。" >&2
        exit 1
    fi
    info=$(ros2 bag info "$bag" 2>&1) || {
        echo "Error: ros2 bag info に失敗しました: $bag" >&2
        echo "$info" >&2
        exit 1
    }
    if echo "$info" | grep -qE 'Topic:.*septentrio/nav_sat_fix'; then
        echo "septentrio"
        return 0
    fi
    if echo "$info" | grep -qE 'Topic:.*ublox/nav_sat_fix'; then
        echo "ublox"
        return 0
    fi
    echo "Warning: bag 内に ublox/nav_sat_fix も septentrio/nav_sat_fix も見つかりません。既定: ublox（EXTRA_LAUNCH_ARGS で上書き可）" >&2
    echo "ublox"
}

RESUME=0
RESET=0
AGGREGATE_ONLY=0
FORCE_SAMPLE_VEHICLE=0
if [[ "${MEASURE_RESUME:-0}" == "1" ]]; then
    RESUME=1
fi

usage() {
    echo "Usage: AUTOWARE_WS=<ws> $0 [options] <MAP_PATH> <SOURCE_ROSBAG> <TARGET_UNIX_SEC> [N_RUNS]" >&2
    echo "  N_RUNS 既定: 10" >&2
    echo "Options:" >&2
    echo "  --resume           状態ファイルから不足分だけ再生・追記して集計" >&2
    echo "  --reset            状態ファイルを削除してから新規実行" >&2
    echo "  --aggregate-only   再生なし。状態ファイルの bag 一覧だけで集計" >&2
    echo "  --force-sample-vehicle  launch_autoware.sh に同フラグを渡す（vehicle_configs の sample 強制）" >&2
    echo "環境変数: PLAYBACK_RATE GT_POSE_TOPIC MEAN_POSE_YAML MEAN_POSE_OUTPUT_DIR INITIAL_POSE_YAML GT_AGG_JSON EXTRA_LAUNCH_ARGS" >&2
    echo "  SKIP_LAUNCH=1 BAGS_LIST_FILE  MEASURE_RESUME=1  ALIGN_POINTCLOUD_TOPIC  MAX_POSE_POINTCLOUD_DT_SEC" >&2
}

POSITIONAL=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --resume)
            RESUME=1
            shift
            ;;
        --reset)
            RESET=1
            shift
            ;;
        --aggregate-only)
            AGGREGATE_ONLY=1
            shift
            ;;
        --force-sample-vehicle)
            FORCE_SAMPLE_VEHICLE=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "Error: 不明なオプション: $1" >&2
            usage
            exit 1
            ;;
        *)
            POSITIONAL+=("$1")
            shift
            ;;
    esac
done
set -- "${POSITIONAL[@]}"

if [[ $# -lt 3 ]]; then
    usage
    exit 1
fi

if [[ "$AGGREGATE_ONLY" == "1" ]] && [[ "$RESUME" == "1" ]]; then
    echo "Error: --aggregate-only と --resume は併用できません" >&2
    exit 1
fi

if [[ ! -f "$LAUNCH_SCRIPT" ]]; then
    echo "Error: launch_autoware.sh not found: $LAUNCH_SCRIPT" >&2
    exit 1
fi
if [[ ! -f "$AGGREGATE_PY" ]]; then
    echo "Error: aggregate_pose_mean_from_bags.py not found: $AGGREGATE_PY" >&2
    exit 1
fi

MAP_PATH="$1"
SOURCE_ROSBAG="$2"
TARGET_UNIX_SEC="$3"
N_RUNS="${4:-10}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"
GT_POSE_TOPIC="${GT_POSE_TOPIC:-/localization/pose_estimator/pose_with_covariance}"

if [[ ! -d "$AUTOWARE_WS/install" ]]; then
    echo "Error: AUTOWARE_WS に install が見つかりません: $AUTOWARE_WS/install" >&2
    exit 1
fi
# colcon の install/setup.bash は COLCON_TRACE 等の未定義参照があり、set -u 下で source すると失敗する
set +u
# shellcheck disable=SC1090
source "$AUTOWARE_WS/install/setup.bash"
set -u

SOURCE_ABS="$(realpath -m "$SOURCE_ROSBAG" 2>/dev/null || echo "$SOURCE_ROSBAG")"
MAP_ABS="$(realpath -m "$MAP_PATH" 2>/dev/null || echo "$MAP_PATH")"
GNSS_RECEIVER_DETECTED="$(detect_gnss_receiver_from_bag "$SOURCE_ABS")"
# launch_autoware.sh: OUTPUT_DIR = dirname(ROSBAG) / record_replay_$DATETIME
RECORD_SCAN_DIR="$(dirname "$SOURCE_ABS")"
if [[ -d "$SOURCE_ABS" ]]; then
    STATE_FILE_DIR="$SOURCE_ABS"
else
    STATE_FILE_DIR="$RECORD_SCAN_DIR"
fi
_rs_base="$(basename "$RECORD_SCAN_DIR")"
if [[ "$_rs_base" == record_replay_* ]]; then
    STATE_DIR="$(dirname "$RECORD_SCAN_DIR")"
else
    STATE_DIR="$RECORD_SCAN_DIR"
fi
TARGET_TAG="${TARGET_UNIX_SEC//./_}"
STATE_FILE="$STATE_FILE_DIR/.measure_pose_mean_t${TARGET_TAG}_n${N_RUNS}.txt"

INITIAL_POSE_YAML="${INITIAL_POSE_YAML:-$STATE_DIR/initial_pose.yaml}"

echo "Info: 再生元: $SOURCE_ABS" >&2
echo "Info: 新規記録の探索ディレクトリ: $RECORD_SCAN_DIR" >&2
echo "Info: 状態ファイル: $STATE_FILE" >&2
echo "Info: 既定 initial_pose / 集計ベースディレクトリ (STATE_DIR): $STATE_DIR" >&2
echo "Info: bag 推定 --gnss-receiver: $GNSS_RECEIVER_DETECTED" >&2
if [[ "$FORCE_SAMPLE_VEHICLE" == "1" ]]; then
    echo "Info: launch に --force-sample-vehicle を付与します" >&2
fi

invoke_launch_autoware_for_measure() {
    cd "$AUTOWARE_WS" || exit 1
    local _args=( "$MAP_PATH" "$SOURCE_ROSBAG" 0 false output_pose_mean --rate "$PLAYBACK_RATE" --gnss-receiver "$GNSS_RECEIVER_DETECTED" )
    if [[ "$FORCE_SAMPLE_VEHICLE" == "1" ]]; then
        _args+=( --force-sample-vehicle )
    fi
    # shellcheck disable=SC2086
    PLAYBACK_RATE="$PLAYBACK_RATE" bash "$LAUNCH_SCRIPT" "${_args[@]}" ${EXTRA_LAUNCH_ARGS:-}
}

write_state_header() {
    cat > "$STATE_FILE" <<EOF
# measure_pose_mean state v1
# SOURCE_ROSBAG=$SOURCE_ABS
# RECORD_SCAN_DIR=$RECORD_SCAN_DIR
# STATE_FILE_DIR=$STATE_FILE_DIR
# MAP_PATH=$MAP_ABS
# GT_POSE_TOPIC=$GT_POSE_TOPIC
# N_RUNS=$N_RUNS
# TARGET_UNIX_SEC=$TARGET_UNIX_SEC
# GNSS_RECEIVER=$GNSS_RECEIVER_DETECTED
# FORCE_SAMPLE_VEHICLE=$FORCE_SAMPLE_VEHICLE
EOF
}

read_state_meta() {
    _meta_src=$(grep '^# SOURCE_ROSBAG=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# SOURCE_ROSBAG=//') || true
    _meta_rsd=$(grep '^# RECORD_SCAN_DIR=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# RECORD_SCAN_DIR=//') || true
    _meta_sfd=$(grep '^# STATE_FILE_DIR=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# STATE_FILE_DIR=//') || true
    _meta_std=$(grep '^# STATE_DIR=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# STATE_DIR=//') || true
    _meta_map=$(grep '^# MAP_PATH=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# MAP_PATH=//') || true
    _meta_topic=$(grep '^# GT_POSE_TOPIC=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# GT_POSE_TOPIC=//') || true
    _meta_n=$(grep '^# N_RUNS=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# N_RUNS=//') || true
    _meta_t=$(grep '^# TARGET_UNIX_SEC=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# TARGET_UNIX_SEC=//') || true
    _meta_gnss=$(grep '^# GNSS_RECEIVER=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# GNSS_RECEIVER=//') || true
    _meta_fsv=$(grep '^# FORCE_SAMPLE_VEHICLE=' "$STATE_FILE" 2>/dev/null | head -1 | sed 's/^# FORCE_SAMPLE_VEHICLE=//') || true
}

verify_state_matches() {
    read_state_meta
    if [[ "$_meta_src" != "$SOURCE_ABS" ]]; then
        echo "Error: 状態ファイルの SOURCE_ROSBAG と一致しません。" >&2
        echo "  state: $_meta_src" >&2
        echo "  now:   $SOURCE_ABS" >&2
        return 1
    fi
    if [[ -n "${_meta_rsd:-}" && "$_meta_rsd" != "$RECORD_SCAN_DIR" ]]; then
        echo "Error: 状態ファイルの RECORD_SCAN_DIR と一致しません（再生パスまたは .db3 の場所が変わった可能性）。" >&2
        echo "  state: $_meta_rsd" >&2
        echo "  now:   $RECORD_SCAN_DIR" >&2
        return 1
    fi
    if [[ -n "${_meta_sfd:-}" && "$_meta_sfd" != "$STATE_FILE_DIR" ]]; then
        echo "Error: 状態ファイルの STATE_FILE_DIR と一致しません。" >&2
        echo "  state: $_meta_sfd" >&2
        echo "  now:   $STATE_FILE_DIR" >&2
        return 1
    fi
    # 旧ヘッダ（# STATE_FILE_DIR なし）: 状態ファイルは STATE_DIR 直下だった。SOURCE / RECORD_SCAN_DIR の一致のみでよい。
    if [[ "$_meta_map" != "$MAP_ABS" ]]; then
        echo "Error: 状態ファイルの MAP_PATH と一致しません。" >&2
        return 1
    fi
    if [[ "$_meta_topic" != "$GT_POSE_TOPIC" ]]; then
        if [[ "$AGGREGATE_ONLY" == "1" ]]; then
            echo "Info: --aggregate-only: 状態ファイルの GT_POSE_TOPIC は $_meta_topic ですが、" >&2
            echo "      今回の GT_POSE_TOPIC=$GT_POSE_TOPIC で集計します（bag に該当トピックが必要）。" >&2
        else
            echo "Error: 状態ファイルの GT_POSE_TOPIC と一致しません。" >&2
            echo "  state: $_meta_topic  now: $GT_POSE_TOPIC" >&2
            return 1
        fi
    fi
    if [[ "$_meta_n" != "$N_RUNS" ]]; then
        echo "Error: 状態ファイルの N_RUNS と一致しません（${_meta_n} vs ${N_RUNS}）。" >&2
        return 1
    fi
    if [[ "$_meta_t" != "$TARGET_UNIX_SEC" ]]; then
        echo "Error: 状態ファイルの TARGET_UNIX_SEC と一致しません。" >&2
        return 1
    fi
    if [[ -n "${_meta_gnss:-}" && "$_meta_gnss" != "$GNSS_RECEIVER_DETECTED" ]]; then
        echo "Error: 状態ファイルの GNSS_RECEIVER（$_meta_gnss）と、現在の bag からの推定（$GNSS_RECEIVER_DETECTED）が一致しません。--reset するか SOURCE_ROSBAG を揃えてください。" >&2
        return 1
    fi
    if [[ -n "${_meta_fsv:-}" && "$_meta_fsv" != "$FORCE_SAMPLE_VEHICLE" ]]; then
        echo "Error: 状態ファイルの FORCE_SAMPLE_VEHICLE（$_meta_fsv）と今回（$FORCE_SAMPLE_VEHICLE）が一致しません。--resume では初回と同じ --force-sample-vehicle の有無にしてください。" >&2
        return 1
    fi
    return 0
}

load_bag_paths_from_state() {
    mapfile -t BAG_ARRAY < <(grep -v '^#' "$STATE_FILE" | grep -v '^$' || true)
}

if [[ "$RESET" == "1" ]]; then
    rm -f "$STATE_FILE"
    echo "Info: 状態ファイルを削除しました: $STATE_FILE" >&2
    # 同一コマンドで MEASURE_RESUME=1 等が有効でも、リセット後は新規実行扱いにする
    RESUME=0
fi

BAG_ARRAY=()

if [[ "${SKIP_LAUNCH:-0}" == "1" ]]; then
    if [[ -z "${BAGS_LIST_FILE:-}" || ! -f "$BAGS_LIST_FILE" ]]; then
        echo "Error: SKIP_LAUNCH=1 のときは BAGS_LIST_FILE に記録 bag パスを1行1つ書いたファイルを指定してください" >&2
        exit 1
    fi
    mapfile -t BAG_ARRAY < "$BAGS_LIST_FILE"
elif [[ "$AGGREGATE_ONLY" == "1" ]]; then
    if [[ ! -f "$STATE_FILE" ]]; then
        echo "Error: --aggregate-only ですが状態ファイルがありません: $STATE_FILE" >&2
        exit 1
    fi
    verify_state_matches || exit 1
    load_bag_paths_from_state
    if [[ ${#BAG_ARRAY[@]} -eq 0 ]]; then
        echo "Error: 状態ファイルに bag パスがありません" >&2
        exit 1
    fi
    echo "Info: --aggregate-only: ${#BAG_ARRAY[@]} 本の bag で集計します" >&2
elif [[ "$RESUME" == "1" ]]; then
    if [[ ! -f "$STATE_FILE" ]]; then
        echo "Error: --resume ですが状態ファイルがありません: $STATE_FILE" >&2
        echo "  初回は --resume なしで実行するか、--reset 後に再実行してください。" >&2
        exit 1
    fi
    verify_state_matches || exit 1
    load_bag_paths_from_state
    have=${#BAG_ARRAY[@]}
    need=$((N_RUNS - have))
    if [[ "$need" -lt 0 ]]; then
        echo "Warning: 状態ファイルの bag 数 ($have) が N_RUNS ($N_RUNS) より多いです。先頭 ${N_RUNS} 本だけ使います。" >&2
        BAG_ARRAY=("${BAG_ARRAY[@]:0:N_RUNS}")
        need=0
    fi
    echo "Info: --resume: 記録済み ${have} 本、あと ${need} 本の再生が必要です" >&2
    if [[ "$need" -gt 0 ]]; then
        for ((i = have + 1; i <= N_RUNS; i++)); do
            echo "========================================" >&2
            echo "Pose mean measurement run $i / $N_RUNS (resume)" >&2
            echo "========================================" >&2
            invoke_launch_autoware_for_measure

            latest=""
            latest=$(ls -td "$RECORD_SCAN_DIR"/record_replay_* 2>/dev/null | head -1 || true)
            if [[ -z "$latest" ]]; then
                echo "Error: 記録ディレクトリ record_replay_* が見つかりません: $RECORD_SCAN_DIR" >&2
                exit 1
            fi
            if [[ ! -f "$latest/metadata.yaml" ]]; then
                echo "Error: metadata.yaml が無い記録先: $latest" >&2
                exit 1
            fi
            echo "$latest" >> "$STATE_FILE"
            echo "Recorded bag dir: $latest" >&2
        done
    fi
    load_bag_paths_from_state
else
    if [[ -f "$STATE_FILE" ]]; then
        echo "Error: 状態ファイルが既に存在します: $STATE_FILE" >&2
        echo "  続きから実行する場合: 同じ引数に --resume を付ける" >&2
        echo "  最初からやり直す場合: --reset を付ける（またはファイルを削除）" >&2
        exit 1
    fi
    write_state_header
    for ((i = 1; i <= N_RUNS; i++)); do
        echo "========================================" >&2
        echo "Pose mean measurement run $i / $N_RUNS" >&2
        echo "========================================" >&2
        invoke_launch_autoware_for_measure

        latest=""
        latest=$(ls -td "$RECORD_SCAN_DIR"/record_replay_* 2>/dev/null | head -1 || true)
        if [[ -z "$latest" ]]; then
            echo "Error: 記録ディレクトリ record_replay_* が見つかりません: $RECORD_SCAN_DIR" >&2
            exit 1
        fi
        if [[ ! -f "$latest/metadata.yaml" ]]; then
            echo "Error: metadata.yaml が無い記録先: $latest" >&2
            exit 1
        fi
        echo "$latest" >> "$STATE_FILE"
        echo "Recorded bag dir: $latest" >&2
    done
    load_bag_paths_from_state
fi

if [[ ${#BAG_ARRAY[@]} -eq 0 ]]; then
    echo "Error: 集計対象の bag がありません" >&2
    exit 1
fi

if [[ ${#BAG_ARRAY[@]} -lt "$N_RUNS" ]] && [[ "${SKIP_LAUNCH:-0}" != "1" ]] && [[ "$AGGREGATE_ONLY" != "1" ]]; then
    echo "Warning: bag 本数 (${#BAG_ARRAY[@]}) が N_RUNS ($N_RUNS) 未満です。このまま集計します。" >&2
fi

# 集計結果は mean_pose_YYYYMMDD_hhmmss ディレクトリへ格納（MEAN_POSE_OUTPUT_DIR でディレクトリを直接指定可能）
if [[ -z "${MEAN_POSE_OUTPUT_DIR:-}" ]]; then
    MEAN_POSE_OUTPUT_DIR="$STATE_FILE_DIR/mean_pose_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$MEAN_POSE_OUTPUT_DIR"
MEAN_POSE_YAML="${MEAN_POSE_YAML:-$MEAN_POSE_OUTPUT_DIR/mean_pose.yaml}"
OUT_JSON="${GT_AGG_JSON:-$MEAN_POSE_OUTPUT_DIR/pose_mean_${TARGET_UNIX_SEC}_n${#BAG_ARRAY[@]}.json}"
mkdir -p "$(dirname "$MEAN_POSE_YAML")" "$(dirname "$OUT_JSON")"

echo "Info: 集計出力ディレクトリ: $MEAN_POSE_OUTPUT_DIR" >&2

# 点群でフレームを揃えてから pose を平均（記録に downsample 点群がある場合）
: "${ALIGN_POINTCLOUD_TOPIC:=/localization/util/downsample/pointcloud}"

AGG_CMD=(
    python3 "$AGGREGATE_PY"
    --target-unix-sec "$TARGET_UNIX_SEC"
    --pose-topic "$GT_POSE_TOPIC"
    --bags "${BAG_ARRAY[@]}"
    --output-json "$OUT_JSON"
    --output-mean-pose-yaml "$MEAN_POSE_YAML"
)
if [[ -n "$ALIGN_POINTCLOUD_TOPIC" ]]; then
    AGG_CMD+=(--align-pointcloud-topic "$ALIGN_POINTCLOUD_TOPIC")
    echo "Info: 集計は点群でアライン: $ALIGN_POINTCLOUD_TOPIC" >&2
fi
if [[ -n "${MAX_POSE_POINTCLOUD_DT_SEC:-}" ]]; then
    AGG_CMD+=(--max-pose-pointcloud-dt-sec "$MAX_POSE_POINTCLOUD_DT_SEC")
    echo "Info: pose と点群 header の差が ${MAX_POSE_POINTCLOUD_DT_SEC}s 超の run は平均から除外" >&2
fi
if [[ -f "$INITIAL_POSE_YAML" ]]; then
    AGG_CMD+=(--initial-pose-yaml "$INITIAL_POSE_YAML")
else
    echo "Info: initial_pose.yaml なし（スキップ）: $INITIAL_POSE_YAML" >&2
fi

"${AGG_CMD[@]}"

echo "Wrote: $OUT_JSON" >&2
echo "Wrote: $MEAN_POSE_YAML" >&2
echo "Output dir: $MEAN_POSE_OUTPUT_DIR" >&2
echo "State file: $STATE_FILE" >&2
