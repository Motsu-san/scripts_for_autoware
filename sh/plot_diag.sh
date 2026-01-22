#!/bin/bash

SCRIPT_DIR=$HOME/repos/autoware_tools/localization/autoware_localization_evaluation_scripts/scripts
DATA_PARENT_DIR=$HOME/your_dlr_result_dir
DLR_RESULT_DIR=$DATA_PARENT_DIR/dlr_result
SAVE_DIR=$DATA_PARENT_DIR/plot_diag

mkdir -p $SAVE_DIR

for d in $DLR_RESULT_DIR/LM_regression_*; do
    echo $d
    SAVE_DIR_NAME=$(basename $d)
    python3 $SCRIPT_DIR/plot_diagnostics.py $d/result_bag --save_dir=$SAVE_DIR/$SAVE_DIR_NAME

done
