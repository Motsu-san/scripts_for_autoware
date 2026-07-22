#!/usr/bin/env python3
"""
Rewrite bag message header.stamp to match bag-level (wall clock) timestamp.

Background:
  Simulator bags store header.stamp as relative sim time (~15 s),
  but the bag-level timestamp (bag_ts) is wall clock (~1775565887 s).
  When USE_SIM_TIME=false, EKF Localizer uses wall clock for pose timestamps,
  so NDT cannot interpolate EKF pose at the 17-billion-second-offset sensor stamps.

Fix:
  Set each message's header.stamp (and /clock) equal to the bag-level timestamp,
  so all timestamps are in wall clock space.
"""
import argparse
import struct
import sys

import rclpy.serialization
import rosbag2_py
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

try:
    from autoware_vehicle_msgs.msg import VelocityReport
    HAS_AUTOWARE_VEHICLE_MSGS = True
except ImportError:
    HAS_AUTOWARE_VEHICLE_MSGS = False
    print('[WARN] autoware_vehicle_msgs not found; VelocityReport will be copied unchanged', file=sys.stderr)


def ns_to_time_msg(ns: int) -> TimeMsg:
    t = TimeMsg()
    t.sec = ns // 1_000_000_000
    t.nanosec = ns % 1_000_000_000
    return t


def rewrite_msg(topic: str, raw: bytes, bag_ts_ns: int, topic_metadata_map: dict = None) -> bytes:
    """Deserialize, overwrite header.stamp with bag_ts, re-serialize."""
    new_stamp = ns_to_time_msg(bag_ts_ns)

    if topic in ('/sensing/lidar/concatenated/pointcloud',):
        msg = rclpy.serialization.deserialize_message(raw, PointCloud2)
        msg.header.stamp = new_stamp
        return rclpy.serialization.serialize_message(msg)

    elif topic == '/sensing/imu/imu_data':
        msg = rclpy.serialization.deserialize_message(raw, Imu)
        msg.header.stamp = new_stamp
        return rclpy.serialization.serialize_message(msg)

    elif topic == '/sensing/gnss/pose_with_covariance':
        msg = rclpy.serialization.deserialize_message(raw, PoseWithCovarianceStamped)
        msg.header.stamp = new_stamp
        return rclpy.serialization.serialize_message(msg)

    elif topic == '/model/tunnel_vehicle_autoware/odometry':
        msg = rclpy.serialization.deserialize_message(raw, Odometry)
        msg.header.stamp = new_stamp
        return rclpy.serialization.serialize_message(msg)

    elif topic in ('/tf', '/tf_static'):
        msg = rclpy.serialization.deserialize_message(raw, TFMessage)
        for transform in msg.transforms:
            transform.header.stamp = new_stamp
        return rclpy.serialization.serialize_message(msg)

    elif topic == '/clock':
        msg = rclpy.serialization.deserialize_message(raw, Clock)
        msg.clock = new_stamp
        return rclpy.serialization.serialize_message(msg)

    elif topic == '/vehicle/status/velocity_status':
        if HAS_AUTOWARE_VEHICLE_MSGS:
            msg = rclpy.serialization.deserialize_message(raw, VelocityReport)
            msg.header.stamp = new_stamp
            return rclpy.serialization.serialize_message(msg)
        else:
            return raw  # copy unchanged

    # Unknown topic: pass through unchanged
    return raw


def get_topic_metadata_map(reader: rosbag2_py.SequentialReader) -> dict:
    """Return {topic_name: TopicMetadata} from bag metadata."""
    meta = reader.get_metadata()
    return {
        t.topic_metadata.name: t.topic_metadata
        for t in meta.topics_with_message_count
    }


def main():
    parser = argparse.ArgumentParser(description='Rewrite bag header.stamp to wall clock time')
    parser.add_argument('input_bag', help='Path to input bag directory')
    parser.add_argument('output_bag', help='Path to output bag directory (must not exist)')
    args = parser.parse_args()

    # ---- Open input bag ----
    storage_options_in = rosbag2_py.StorageOptions(uri=args.input_bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options_in, converter_options)

    topic_metadata_map = get_topic_metadata_map(reader)

    # ---- Open output bag ----
    storage_options_out = rosbag2_py.StorageOptions(uri=args.output_bag, storage_id='sqlite3')
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options_out, converter_options)

    # Register all topics, preserving original QoS profiles
    for topic_name, src_meta in topic_metadata_map.items():
        topic_meta = rosbag2_py.TopicMetadata(
            name=src_meta.name,
            type=src_meta.type,
            serialization_format=src_meta.serialization_format,
            offered_qos_profiles=src_meta.offered_qos_profiles,
        )
        writer.create_topic(topic_meta)

    print(f'Topics: {list(topic_metadata_map.keys())}')
    print('Rewriting timestamps...')

    count = 0
    while reader.has_next():
        topic, raw, bag_ts = reader.read_next()
        new_raw = rewrite_msg(topic, raw, bag_ts, topic_metadata_map)
        writer.write(topic, new_raw, bag_ts)

        count += 1
        if count % 50000 == 0:
            print(f'  Processed {count} messages...')

    print(f'Done. Total messages processed: {count}')
    print(f'Output bag: {args.output_bag}')


if __name__ == '__main__':
    main()
