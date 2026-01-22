#!/bin/bash
# See https://github.com/tier4/auto-evaluator/blob/main/auto_eval/localization/report_generator/README.md#2-rosbag%E3%83%87%E3%83%BC%E3%82%BF%E3%81%AE%E3%83%80%E3%82%A6%E3%83%B3%E3%83%AD%E3%83%BC%E3%83%89

SCRIPT_DIR=$HOME/repos/tier4/auto-evaluator/auto_eval/localization
SAVE_DIR=$HOME/evaluator_result_rosbag/release_v0.42.0
result_url_list=(
    "https://evaluation.tier4.jp/evaluation/reports/8297e839-c967-5afd-9dfa-83b38d0d9ab8?project_id=prd_jt"
    "https://evaluation.tier4.jp/evaluation/reports/0883a830-3ec8-5d39-bcb3-58a1790e2bdc?project_id=autoware_dev"
)

cd $SCRIPT_DIR

poetry run $SCRIPT_DIR/download_result_bag.py \
  --save_dir $SAVE_DIR \
  --result_url ${result_url_list[@]}
