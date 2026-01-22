#!/bin/bash

INPUT_DIR="$HOME/autoware_map/pointcloud_maps/"
OUTPUT_DIR="$HOME/autoware_map/output_pcd/"
PREFIX="your_pcd_map_dir_prefix"
CONFIG="$HOME/repos/MapIV/pointcloud_divider/config/default.yaml"

mkdir $OUTPUT_DIR

cd $HOME/repos/MapIV/pointcloud_divider
./scripts/pointcloud_divider.sh $INPUT_DIR $OUTPUT_DIR $PREFIX $CONFIG
