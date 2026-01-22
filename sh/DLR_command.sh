#!/bin/bash
DATETIME=$(date '+%Y%m%d_%H%M%S')

CALL_DIR=$(pwd) # run at autoware workspace directory

OUTPUT_DIR="$HOME/DLR_output/run_${DATETIME}"

TIMEOUT_SECONDS=0

SCENARIO_PATH="$HOME/your_scenario.yml"
DATASET_PATH="$HOME/your_dataset_path/"
DATASET_ID=$(basename $(dirname "$DATASET_PATH"))

# source $HOME/autoware/install/setup.bash
source $CALL_DIR/install/setup.bash

# Set traps to stop all background processes when the script exits
trap "$HOME/scripts_for_autoware/sh/kill_autoware.sh" EXIT INT TERM HUP

if [ "$TIMEOUT_SECONDS" -eq 0 ]; then
  # TIMEOUT_SECONDSが0の場合は最後まで再生する（timeoutコマンドを使用しない）
  ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
    scenario_path:=$SCENARIO_PATH \
    DATASET_path:=$DATASET_PATH \
    DATASET_id:=$DATASET_ID \
    output_dir:=$OUTPUT_DIR | tee $HOME/log/DLR_replay_$DATETIME.log
else
  # TIMEOUT_SECONDSが0以外の場合は指定時間でタイムアウト
  timeout $TIMEOUT_SECONDS ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
    scenario_path:=$SCENARIO_PATH \
    DATASET_path:=$DATASET_PATH \
    DATASET_id:=$DATASET_ID \
    output_dir:=$OUTPUT_DIR | tee $HOME/log/DLR_replay_$DATETIME.log
fi
