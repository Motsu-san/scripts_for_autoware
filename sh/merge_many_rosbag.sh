#!/bin/bash

# usage: ./merge_rosbag.sh

# merge
# INPUT_DIR=$1
INPUT_DIR="$HOME/rosbag_replay"
DIR_PREFIX='your_rosbag_dir_prefix'

find "$INPUT_DIR" -type d -name "$DIR_PREFIX*" | while read dir
do
    TARGET_DIR=$dir
    bash merge_rosbag.sh $TARGET_DIR
    # echo $TARGET_DIR
done
