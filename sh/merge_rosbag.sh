#!/bin/bash

set -e  # Exit immediately if any command exits with a non-zero status

# usage: ./merge_rosbag.sh [rosbag_downroad_dir]

# merge
INPUT_DIR=$1
# Convert to absolute path if necessary
if [ "${INPUT_DIR:0:1}" != "/" ]; then
    INPUT_DIR="$(cd "$(dirname "$INPUT_DIR")" && pwd)/$(basename "$INPUT_DIR")"
fi

cd $HOME/extension_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash

TMP_DIR="$INPUT_DIR/tmp"
MERGE_DIR="$INPUT_DIR/merged"
ZST_EXTENSION=".zst"
mkdir -p $TMP_DIR

# Find and decompress each .zst file individually, deleting only on success
find "$INPUT_DIR" -type f -name "*${ZST_EXTENSION}" | while read zst_file; do
    if zstd -d "$zst_file" --output-dir-flat "$TMP_DIR"; then
        echo "Info: Successfully decompressed $zst_file"
        rm -f "$zst_file"
    else
        echo "Error: Failed to decompress $zst_file" >&2
        exit 1
    fi
done

# Merge bags incrementally to save disk space
# First, rename files to have zero-padded numbers for proper sorting
for file in $TMP_DIR/*.db3; do
    if [[ "$file" =~ (.*)_([0-9]+)\.db3$ ]]; then
        base="${BASH_REMATCH[1]}"
        number="${BASH_REMATCH[2]}"
        # Zero-pad the number to 4 digits
        padded_number=$(printf "%04d" "$number")
        new_name="${base}_${padded_number}.db3"
        if [ "$file" != "$new_name" ]; then
            mv "$file" "$new_name"
        fi
    fi
done

# Get sorted list of db3 files
db3_files=($(ls $TMP_DIR/*.db3 | sort))
if [ ${#db3_files[@]} -eq 0 ]; then
    echo "Error: No .db3 files found in $TMP_DIR" >&2
    exit 1
fi

# Start with first file
first_file="${db3_files[0]}"
if ros2 bag merge -o $MERGE_DIR "$first_file"; then
    rm -f "$first_file"
else
    echo "Error: Failed to merge first bag" >&2
    exit 1
fi

# Merge remaining files one by one
for ((i=1; i<${#db3_files[@]}; i++)); do
    current_file="${db3_files[i]}"
    temp_merge="$INPUT_DIR/temp_merge_$i"

    if ros2 bag merge -o "$temp_merge" "$MERGE_DIR" "$current_file"; then
        rm -r "$MERGE_DIR"
        mv "$temp_merge" "$MERGE_DIR"
        rm -f "$current_file"
    else
        echo "Error: Failed to merge $current_file" >&2
        exit 1
    fi
done

# rm -r $TMP_DIR

# filter & reindex
FILTER_DIR="$INPUT_DIR/merged_filtered"

echo "Starting bag filtering at $(date)..."
START_FILTER=$(date +%s)

TOPICS=(
    /sensing/lidar/top/velodyne_packets
    /sensing/lidar/front_upper/pandar_packets
    /sensing/lidar/left_upper/pandar_packets
    /sensing/lidar/right_upper/pandar_packets
    /sensing/lidar/rear_upper/pandar_packets
    /sensing/lidar/front_lower/pandar_packets
    /sensing/lidar/left_lower/pandar_packets
    /sensing/lidar/right_lower/pandar_packets
    /sensing/lidar/rear_lower/pandar_packets
    /vehicle/status/steering_status
    /vehicle/status/velocity_status
    /sensing/imu/tamagawa/imu_raw
    /sensing/lidar/front_center/livox/imu
    /sensing/gnss/septentrio/nav_sat_fix
    /sensing/gnss/ublox/nav_sat_fix
    /tf_static
)

if ros2 bag filter -o $FILTER_DIR $MERGE_DIR -i "${TOPICS[@]}"; then
    END_FILTER=$(date +%s)
    DURATION_FILTER=$((END_FILTER - START_FILTER))
    echo "Filtering completed at $(date). Duration: ${DURATION_FILTER} seconds."
    if [ -d "$MERGE_DIR" ]; then
        rm -r "$MERGE_DIR"
    fi
else
    echo "Error: Failed to filter bags" >&2
    exit 1
fi
