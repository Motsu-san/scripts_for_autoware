#!/usr/bin/env bash
# Kill Autoware / rosbag playback leftovers without touching offline bag
# processing (e.g. rosbag_download_filter_merge.sh) or webauto download jobs.
#
# Hybrid policy:
#   (1) Allowlist  — only these cmdline patterns are kill candidates
#   (2) Exclude    — bag加工 / webauto とその子孫は常に保護

echo ===== kill autoware

# --- (2) Exclude: never kill these (or their descendants) ---
EXCLUDE_PATTERNS=(
  'rosbag_download_filter_merge\.sh'
  'merge_rosbag\.sh'
  'ros2 bag reindex'
  'ros2 bag filter'
  'ros2 bag merge'
  'ros2 bag info'
  'webauto data rosbag'
  'webauto data log-file'
  # このスクリプト自身(デバッグ実行時の誤爆防止)
  'kill_autoware\.sh'
)

# --- (1) Allowlist: Autoware / 再生まわりだけ候補にする ---
# 広い "pkill ros2" / "pgrep ros" は使わない(bag 加工を巻き込むため)
ALLOW_PATTERNS=(
  'rviz2'
  'aggregator_node'
  'component_container'
  'robot_state_publisher'
  'topic_tools/relay'
  'AWSIM_demo\.x86_64'
  'rosbridge_websocket'
  'rqt_reconfigure'
  'ros2 launch'
  'ros2 bag play'
  'ros2 bag record'
  # Autoware ノード(component / standalone)
  '--ros-args'
  # python 経由の ros2 CLI / ノード(exclude で bag filter 等は除外)
  'python3.*ros2'
)

# pgrep -f でパターンに合う PID を列挙(数値のみ、重複除去)
collect_pids_by_patterns() {
  local pat pid
  local -a out=()
  for pat in "$@"; do
    while IFS= read -r pid; do
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      out+=("$pid")
    done < <(pgrep -f -- "$pat" 2>/dev/null || true)
  done
  if ((${#out[@]} > 0)); then
    printf '%s\n' "${out[@]}" | sort -nu
  fi
}

# 親 PID の子孫を再帰的に追加
expand_with_descendants() {
  local -A seen=()
  local -a queue=()
  local pid child
  while IFS= read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    queue+=("$pid")
  done
  while ((${#queue[@]} > 0)); do
    pid="${queue[0]}"
    queue=("${queue[@]:1}")
    [[ -n "${seen[$pid]:-}" ]] && continue
    seen["$pid"]=1
    while IFS= read -r child; do
      [[ "$child" =~ ^[0-9]+$ ]] || continue
      [[ -n "${seen[$child]:-}" ]] && continue
      queue+=("$child")
    done < <(pgrep -P "$pid" 2>/dev/null || true)
  done
  if ((${#seen[@]} > 0)); then
    printf '%s\n' "${!seen[@]}" | sort -nu
  fi
}

# 保護集合を構築
mapfile -t _exclude_roots < <(collect_pids_by_patterns "${EXCLUDE_PATTERNS[@]}")
mapfile -t PROTECTED_PIDS < <(printf '%s\n' "${_exclude_roots[@]}" | expand_with_descendants)

declare -A PROTECTED=()
for pid in "${PROTECTED_PIDS[@]}"; do
  [[ -n "$pid" ]] && PROTECTED["$pid"]=1
done

if ((${#PROTECTED[@]} > 0)); then
  echo "===== protect ${#PROTECTED[@]} pid(s) (bag加工 / webauto)"
fi

# 自プロセスと親シェルは殺さない
PROTECTED["$$"]=1
PROTECTED["$PPID"]=1

is_protected() {
  [[ -n "${PROTECTED[$1]:-}" ]]
}

# 許可リスト候補から保護を除いた PID を返す(子孫も候補に含める)
mapfile -t _allow_roots < <(collect_pids_by_patterns "${ALLOW_PATTERNS[@]}")
mapfile -t _allow_all < <(printf '%s\n' "${_allow_roots[@]}" | expand_with_descendants)

KILL_CANDIDATES=()
for pid in "${_allow_all[@]}"; do
  [[ -n "$pid" ]] || continue
  is_protected "$pid" && continue
  # 既に死んでいるものはスキップ
  kill -0 "$pid" 2>/dev/null || continue
  KILL_CANDIDATES+=("$pid")
done

kill_candidates() {
  local sig="$1"
  local pid
  for pid in "${KILL_CANDIDATES[@]}"; do
    is_protected "$pid" && continue
    kill -"$sig" "$pid" 2>/dev/null || true
  done
}

if ((${#KILL_CANDIDATES[@]} == 0)); then
  echo "===== no allowlisted processes to kill"
else
  echo "===== SIGINT ${#KILL_CANDIDATES[@]} allowlisted pid(s)"
  kill_candidates 2
  sleep 1

  # 生存している候補だけ TERM
  STILL_ALIVE=()
  for pid in "${KILL_CANDIDATES[@]}"; do
    kill -0 "$pid" 2>/dev/null || continue
    is_protected "$pid" && continue
    STILL_ALIVE+=("$pid")
  done
  KILL_CANDIDATES=("${STILL_ALIVE[@]}")
  if ((${#KILL_CANDIDATES[@]} > 0)); then
    echo "===== SIGTERM ${#KILL_CANDIDATES[@]} remaining pid(s)"
    kill_candidates 15
    sleep 1
  fi

  STILL_ALIVE=()
  for pid in "${KILL_CANDIDATES[@]}"; do
    kill -0 "$pid" 2>/dev/null || continue
    is_protected "$pid" && continue
    STILL_ALIVE+=("$pid")
  done
  KILL_CANDIDATES=("${STILL_ALIVE[@]}")
  if ((${#KILL_CANDIDATES[@]} > 0)); then
    echo "===== SIGKILL ${#KILL_CANDIDATES[@]} remaining pid(s)"
    kill_candidates 9
  fi
fi

echo ===== daemon reboot
# bag 加工を守るため、daemon 操作時点でも exclude 対象は触らない(ros2 CLI 自体は短命)
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null || true

echo ===== check topic is still alive
ros2 topic list 2>/dev/null || true
