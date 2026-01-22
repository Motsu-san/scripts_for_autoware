#!/bin/bash
# ./iv_to_auto.sh 2>&1 | tee convert.log

source $HOME/auto_converter_ws/install/setup.bash

BAG_HOME_PATH="$HOME/rosbag_replay"

dirs=`find $BAG_HOME_PATH/[0-9]* -maxdepth 0 -type d`
for dir in $dirs;
do
    echo $dir
    INPUT_BAG_PATH=$dir
    # reindex
    ros2 bag reindex $INPUT_BAG_PATH -s sqlite3
    # convert multiple bags in the directory
    ros2 run iv_to_auto_bag_converter convert_multi $INPUT_BAG_PATH
    conv_dir=`find $BAG_HOME_PATH/converted_[0-9]* -maxdepth 0 -type d`
    INPUT_DIR=$conv_dir
    OUTPUT_DIR=${conv_dir/converted/converted_filtered}
    echo $OUTPUT_DIR
    # reindex
    ros2 bag reindex $INPUT_DIR -s sqlite3
    # Remove topics not related to localization
    ros2 bag filter $INPUT_DIR -o $OUTPUT_DIR -i \
        /sensing/lidar/top/velodyne_packets \
        /sensing/lidar/left_upper/pandar_packets \
        /sensing/lidar/right_upper/pandar_packets \
        /vehicle/status/steering_status \
        /vehicle/status/velocity_status \
        /sensing/imu/tamagawa/imu_raw \
        /sensing/lidar/front_center/livox/imu \
        /sensing/gnss/septentrio/nav_sat_fix \
        /sensing/gnss/ublox/nav_sat_fix \
        /tf_static
    rm -rf $INPUT_DIR
done

# convert one bag
# ros2 run iv_to_auto_bag_converter convert $INPUT_BAG_PATH $output_bag_path

# convert multiple bags in the directory
# ros2 run iv_to_auto_bag_converter convert_multi $INPUT_BAG_PATH

# convert multiple bags in the directory and delete input(iv) bags after conversion
# ros2 run iv_to_auto_bag_converter convert_multi $(input_bag_path) --delete [-q $(qos_override_file_name)]
