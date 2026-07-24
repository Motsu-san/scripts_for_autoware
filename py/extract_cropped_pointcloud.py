#!/usr/bin/env python3
"""
rosbag2 から指定 PointCloud2 トピックを指定ボックス領域でクロップして抽出する。

- 入力: rosbag2 + PointCloud2 トピック名 + ボックス範囲 (min_x, max_x, min_y, max_y, min_z, max_z)
- 出力: クロップした点群のみが含まれる rosbag2(指定トピックのみ処理、他トピックはそのままコピー可)

crop_box_filter (autoware_pointcloud_preprocessor) と同様の動作:
  negative=False: ボックス外の点を除去 → ボックス内の点のみ残す(指定領域でクロップ)
  negative=True:  ボックス内の点を除去(車体除去など)

注意: /sensing/lidar/front_lower/pandar_packets は生パケットのため PointCloud2 ではありません。
      点群をクロップするには、以下のいずれかが必要です:
  1) rosbag にすでに PointCloud2 トピックがある場合(例: rectified/pointcloud_ex)→ 本スクリプトでそのトピックを指定
  2) pandar_packets のみの場合 → Autoware で rosbag 再生し、点群に変換したトピックを crop_box_filter で処理して記録するか、
     一度点群トピックを record してから本スクリプトでオフラインクロップ

Usage:
  python3 extract_cropped_pointcloud.py <input_bag> <output_bag> <pointcloud_topic> \\
      [--min-x MIN_X] [--max-x MAX_X] [--min-y MIN_Y] [--max-y MAX_Y] [--min-z MIN_Z] [--max-z MAX_Z] \\
      [--negative] [--copy-other-topics]

Example:
  python3 extract_cropped_pointcloud.py ./input_bag ./out_bag /sensing/lidar/front_lower/rectified/pointcloud_ex \\
      --min-x -50 --max-x 50 --min-y -30 --max-y 30 --min-z -2 --max-z 2

Dependencies:
  - ROS2 Humble or later
  - rosbag2_py, sensor_msgs, numpy, rclpy
"""

import argparse
import sys
import numpy as np
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
import rclpy
from rclpy.serialization import deserialize_message, serialize_message

# PointField datatype -> numpy dtype (for size/read)
PF2NP = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def get_xyz_offsets(msg: PointCloud2):
    """PointCloud2 の x,y,z のオフセット(byte)とdtypeを返す。"""
    name_to_offset = {}
    name_to_dtype = {}
    for f in msg.fields:
        name_to_offset[f.name] = f.offset
        if f.datatype not in PF2NP:
            raise ValueError(f"Unsupported point field type: {f.datatype}")
        name_to_dtype[f.name] = PF2NP[f.datatype]
    for cand in (("x", "y", "z"), ("X", "Y", "Z")):
        if all(n in name_to_offset for n in cand):
            return (
                (name_to_offset[cand[0]], name_to_dtype[cand[0]]),
                (name_to_offset[cand[1]], name_to_dtype[cand[1]]),
                (name_to_offset[cand[2]], name_to_dtype[cand[2]]),
            )
    raise ValueError(f"Could not find x,y,z fields. Fields: {[f.name for f in msg.fields]}")


def crop_pointcloud(
    msg: PointCloud2,
    min_x: float, max_x: float,
    min_y: float, max_y: float,
    min_z: float, max_z: float,
    negative: bool,
) -> PointCloud2:
    """
    指定ボックスで点群をクロップする。crop_box_filter と同様。
    negative=False: ボックス内の点のみ残す(指定領域でクロップ)
    negative=True:  ボックス内の点を除去
    """
    (ox, dt_x), (oy, dt_y), (oz, dt_z) = get_xyz_offsets(msg)
    elem_x = np.dtype(dt_x).itemsize
    elem_y = np.dtype(dt_y).itemsize
    elem_z = np.dtype(dt_z).itemsize
    npts = len(msg.data) // msg.point_step
    # shape (npts, point_step) の uint8 で扱い、x,y,z をスライスで取得
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(npts, msg.point_step)
    x = np.frombuffer(raw[:, ox : ox + elem_x].tobytes(), dtype=dt_x)
    y = np.frombuffer(raw[:, oy : oy + elem_y].tobytes(), dtype=dt_y)
    z = np.frombuffer(raw[:, oz : oz + elem_z].tobytes(), dtype=dt_z)
    inside = (
        (x >= min_x) & (x <= max_x) &
        (y >= min_y) & (y <= max_y) &
        (z >= min_z) & (z <= max_z)
    )
    if negative:
        mask = ~inside
    else:
        mask = inside
    out_raw = raw[mask]
    if out_raw.size == 0:
        # 空の点群でも valid なメッセージを返す
        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = 1
        out_msg.width = 0
        out_msg.fields = msg.fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = msg.point_step
        out_msg.row_step = 0
        out_msg.is_dense = msg.is_dense
        out_msg.data = b""
        return out_msg
    out_msg = PointCloud2()
    out_msg.header = msg.header
    out_msg.height = 1
    out_msg.width = int(out_raw.shape[0])
    out_msg.fields = msg.fields
    out_msg.is_bigendian = msg.is_bigendian
    out_msg.point_step = msg.point_step
    out_msg.row_step = out_msg.point_step * out_msg.width
    out_msg.is_dense = msg.is_dense
    out_msg.data = out_raw.tobytes()
    return out_msg


def main():
    parser = argparse.ArgumentParser(
        description="Extract cropped PointCloud2 from rosbag2 by box region (crop_box_filter compatible)."
    )
    parser.add_argument("input_bag", help="Input rosbag2 directory")
    parser.add_argument("output_bag", help="Output rosbag2 directory")
    parser.add_argument("topic", help="PointCloud2 topic to crop (e.g. /sensing/lidar/front_lower/rectified/pointcloud_ex)")
    parser.add_argument("--min-x", type=float, default=-1.0, help="Box min_x (m)")
    parser.add_argument("--max-x", type=float, default=1.0, help="Box max_x (m)")
    parser.add_argument("--min-y", type=float, default=-1.0, help="Box min_y (m)")
    parser.add_argument("--max-y", type=float, default=1.0, help="Box max_y (m)")
    parser.add_argument("--min-z", type=float, default=-1.0, help="Box min_z (m)")
    parser.add_argument("--max-z", type=float, default=1.0, help="Box max_z (m)")
    parser.add_argument("--negative", action="store_true", help="Remove points inside box (keep outside)")
    parser.add_argument("--copy-other-topics", action="store_true", help="Copy non-target topics to output bag as-is")
    args = parser.parse_args()

    topic_to_crop = args.topic
    rclpy.init()

    storage_options = rosbag2_py.StorageOptions(uri=args.input_bag, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")

    # 1st pass: count messages on target topic
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR] Failed to open input bag: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    total_msgs = 0
    try:
        while reader.has_next():
            topic_name, _, _ = reader.read_next()
            if topic_name == topic_to_crop:
                total_msgs += 1
    except Exception as e:
        print(f"[ERROR] Failed to read input bag: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    del reader

    if total_msgs == 0:
        print(f"[WARNING] No messages found on topic '{topic_to_crop}'. Check topic name and bag.", file=sys.stderr)

    # 2nd pass: process and write
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output_bag, storage_id="sqlite3"),
        converter_options,
    )
    topics_and_types = reader.get_all_topics_and_types()
    if args.copy_other_topics:
        for t in topics_and_types:
            writer.create_topic(t)
    else:
        for t in topics_and_types:
            if t.name == topic_to_crop:
                writer.create_topic(t)
                break

    converted = 0
    written_other = 0
    try:
        while reader.has_next():
            topic_name, data, t = reader.read_next()
            if topic_name == topic_to_crop:
                try:
                    msg = deserialize_message(data, PointCloud2)
                except Exception as e:
                    print(f"[ERROR] Deserialize failed on {topic_name}: {e}", file=sys.stderr)
                    continue
                try:
                    new_msg = crop_pointcloud(
                        msg,
                        args.min_x, args.max_x,
                        args.min_y, args.max_y,
                        args.min_z, args.max_z,
                        args.negative,
                    )
                    data = serialize_message(new_msg)
                except Exception as e:
                    print(f"[ERROR] Crop failed on {topic_name}: {e}", file=sys.stderr)
                    continue
                writer.write(topic_name, data, t)
                converted += 1
                if total_msgs > 0 and converted % max(1, total_msgs // 20) == 0:
                    print(f"Cropped [{topic_to_crop}]: {converted}/{total_msgs}")
            else:
                if args.copy_other_topics:
                    writer.write(topic_name, data, t)
                    written_other += 1
    except Exception as e:
        print(f"[ERROR] During processing: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    print(f"Cropped {converted} messages on '{topic_to_crop}'. Output: {args.output_bag}")
    if args.copy_other_topics:
        print(f"Copied {written_other} messages from other topics.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
