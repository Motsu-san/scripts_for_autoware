#!/usr/bin/env python3
"""
rosbag2 PointCloud2 type converter: PointXYZIRADRT -> PointXYZIRC

- Reads a rosbag2 file containing PointCloud2 messages with PointXYZIRADRT layout
- Converts each message to PointXYZIRC layout (drops 'ring', 'azimuth', 'distance', 'timestamp' fields, keeps x, y, z, intensity, ring, class)
- Writes to a new rosbag2 file

Usage:
  python3 convert_pointcloud_type.py <input_bag> <output_bag> <topic_name>

Dependencies:
  - ROS2 Humble or later
  - rosbag2_py
  - sensor_msgs
  - numpy

"""
import sys
import numpy as np
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
import rclpy
from rclpy.serialization import deserialize_message, serialize_message


# Actual PointCloud2 fields (with padding):
# x: float32 (0), y: float32 (4), z: float32 (8), [padding: float32 (12)], intensity: float32 (16), ring: uint32 (20), azimuth: float32 (24), distance: float32 (28), return_type: uint8 (32), [padding: 7 bytes (33-39)], time_stamp: float64 (40)

POINTXYZIRADRT_FIELDS = [
    ('x', np.float32),         # 0
    ('y', np.float32),         # 4
    ('z', np.float32),         # 8
    ('_pad0', np.float32),     # 12 (padding)
    ('intensity', np.float32), # 16
    ('ring', np.uint32),       # 20
    ('azimuth', np.float32),   # 24
    ('distance', np.float32),  # 28
    ('return_type', np.uint8), # 32
    ('_pad1', 'V7'),           # 33-39 (padding)
    ('time_stamp', np.float64) # 40
]


# Output PointXYZIRCAEDT: x, y, z, intensity(uint8), return_type(uint8), channel(uint16), azimuth(float32), elevation(float32), distance(float32), time_stamp(uint32)
POINTXYZIRCAEDT_FIELDS = [
    ('x', np.float32),         # 0
    ('y', np.float32),         # 4
    ('z', np.float32),         # 8
    ('intensity', np.uint8),   # 12
    ('return_type', np.uint8), # 13
    ('channel', np.uint16),    # 14
    ('azimuth', np.float32),   # 16
    ('elevation', np.float32), # 20
    ('distance', np.float32),  # 24
    ('time_stamp', np.uint32), # 28
]


def convert_pointcloud(msg: PointCloud2) -> PointCloud2:
    # Build numpy dtype for old and new
    old_dtype = np.dtype(POINTXYZIRADRT_FIELDS)
    new_dtype = np.dtype(POINTXYZIRCAEDT_FIELDS)
    arr = np.frombuffer(msg.data, dtype=old_dtype)
    new_arr = np.zeros(arr.shape, dtype=new_dtype)
    new_arr['x'] = arr['x']
    new_arr['y'] = arr['y']
    new_arr['z'] = arr['z']
    # intensity: float32→uint8（スケーリングやclippingが必要ならここで）
    new_arr['intensity'] = np.clip(arr['intensity'], 0, 255).astype(np.uint8)
    # return_type: uint8
    new_arr['return_type'] = arr['return_type']
    # channel: ring→uint16（uint32→uint16）
    new_arr['channel'] = arr['ring'].astype(np.uint16)
    # azimuth, distance: そのままコピー
    new_arr['azimuth'] = arr['azimuth']
    # elevation: 0埋め（元データに無い場合）
    new_arr['elevation'] = 0
    new_arr['distance'] = arr['distance']
    # time_stamp: float64→uint32（ミリ秒変換例。用途に応じて調整）
    new_arr['time_stamp'] = (arr['time_stamp'] * 1000).astype(np.uint32)
    # Build new PointCloud2 message
    new_msg = PointCloud2()
    new_msg.header = msg.header
    new_msg.height = msg.height
    new_msg.width = msg.width
    new_msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
        PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
        PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
        PointField(name='azimuth', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='elevation', offset=20, datatype=PointField.FLOAT32, count=1),
        PointField(name='distance', offset=24, datatype=PointField.FLOAT32, count=1),
        PointField(name='time_stamp', offset=28, datatype=PointField.UINT32, count=1),
    ]
    new_msg.is_bigendian = msg.is_bigendian
    new_msg.point_step = new_dtype.itemsize
    new_msg.row_step = new_msg.point_step * new_msg.width
    new_msg.is_dense = msg.is_dense
    new_msg.data = new_arr.tobytes()
    return new_msg

def main():
    if len(sys.argv) != 4:
        print("Usage: python3 convert_pointcloud_type.py <input_bag> <output_bag> <topic_names>")
        print("  <topic_names>: comma-separated list of topic names to convert")
        print("Example: python3 convert_pointcloud_type.py input_bag output_bag /points_raw,/lidar_points")
        sys.exit(1)
    input_bag, output_bag, topic_names = sys.argv[1:4]
    topic_list = [t.strip() for t in topic_names.split(',') if t.strip()]

    rclpy.init()
    # 1st pass: count messages for progress (per topic)
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR]: Failed to open input bag file '{input_bag}'. {e}")
        rclpy.shutdown()
        sys.exit(1)
    total_msgs = {t: 0 for t in topic_list}
    try:
        while reader.has_next():
            topic_name, _, _ = reader.read_next()
            if topic_name in topic_list:
                total_msgs[topic_name] += 1
    except Exception as e:
        print(f"[ERROR]: Failed to read input bag file. {e}")
        rclpy.shutdown()
        sys.exit(1)
    del reader

    # if no topics were found to convert
    found_topics = [t for t in topic_list if total_msgs[t] > 0]
    if not found_topics:
        print("[WARNING]: No topics were found to convert. Please verify the specified topic name or input bag file.")
        rclpy.shutdown()
        sys.exit(1)

    # 2nd pass: actual conversion with progress
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR]: Failed to reopen input bag file for conversion. {e}")
        rclpy.shutdown()
        sys.exit(1)
    try:
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3'), converter_options)
    except Exception as e:
        print(f"[ERROR]: Failed to open output bag file '{output_bag}'. {e}")
        rclpy.shutdown()
        sys.exit(1)
    try:
        topics = reader.get_all_topics_and_types()
        for t in topics:
            writer.create_topic(t)
    except Exception as e:
        print(f"[ERROR]: Failed to get or create topics. {e}")
        rclpy.shutdown()
        sys.exit(1)
    converted = {t: 0 for t in topic_list}
    def print_progress():
        for t in topic_list:
            if total_msgs[t] > 0:
                percent = 100 * converted[t] // total_msgs[t]
                print(f"Converting [{t}]: {converted[t]}/{total_msgs[t]} ({percent}%)")
    try:
        while reader.has_next():
            try:
                (topic_name, data, t) = reader.read_next()
            except Exception as e:
                print(f"[ERROR]: Failed to read message. {e}")
                continue
            if topic_name in topic_list:
                try:
                    msg = deserialize_message(data, PointCloud2)
                except Exception as e:
                    print(f"[ERROR]: Failed to deserialize message on topic '{topic_name}'. {e}")
                    continue
                try:
                    new_msg = convert_pointcloud(msg)
                except Exception as e:
                    print(f"[ERROR]: Failed to convert message on topic '{topic_name}'. {e}")
                    continue
                try:
                    data = serialize_message(new_msg)
                except Exception as e:
                    print(f"[ERROR]: Failed to serialize converted message on topic '{topic_name}'. {e}")
                    continue
                converted[topic_name] += 1
                print_progress()
            try:
                writer.write(topic_name, data, t)
            except Exception as e:
                print(f"[ERROR]: Failed to write message to output bag. {e}")
                continue
    except Exception as e:
        print(f"[ERROR]: Unexpected error during conversion. {e}")
        rclpy.shutdown()
        sys.exit(1)
    for t in topic_list:
        if total_msgs[t] > 0:
            print(f"Converting [{t}]: {total_msgs[t]}/{total_msgs[t]} (100%)")
    print(f"Conversion complete. Output bag: {output_bag}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
