#!/usr/bin/env python3
"""
rosbag2 PointCloud2 レイアウト変換（Autoware pointcloud_preprocessor 互換向け）

対応パターン（各メッセージごとに自動判定）:
  - PointXYZI (intensity が FLOAT32 の典型レイアウト) → PointXYZIRC
  - intensity が UINT8 の x,y,z,float 相当 4 フィールド → PointXYZIRC
  - PointXYZIRADRT (numpy 構造体と一致する point_step) → PointXYZIRCAEDT
  - 既に PointXYZIRC / PointXYZIRCAEDT の場合はコピーのみ（無変換）

PointXYZIRC / PointXYZIRCAEDT の定義は autoware_pointcloud_preprocessor の
is_data_layout_compatible_with_point_xyzirc / _xyzircaedt に合わせています。

Usage:
  python3 convert_pointcloud_type.py <input_bag> <output_bag> <topic_names>

  <topic_names>: カンマ区切りのトピック名（例: /sensing/lidar/concatenated/pointcloud）

Example:
  python3 convert_pointcloud_type.py in_bag out_bag /sensing/lidar/concatenated/pointcloud

Dependencies:
  - ROS2 Humble or later
  - rosbag2_py, sensor_msgs, numpy
"""
from __future__ import annotations

import struct
import sys
from typing import Dict, List, Literal, Tuple

import numpy as np
import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField

# PointXYZIRADRT (Velodyne 系 + padding) — 従来スクリプトと同一レイアウト
POINTXYZIRADRT_FIELDS = [
    ("x", np.float32),
    ("y", np.float32),
    ("z", np.float32),
    ("_pad0", np.float32),
    ("intensity", np.float32),
    ("ring", np.uint32),
    ("azimuth", np.float32),
    ("distance", np.float32),
    ("return_type", np.uint8),
    ("_pad1", "V7"),
    ("time_stamp", np.float64),
]

POINTXYZIRADRT_DTYPE = np.dtype(POINTXYZIRADRT_FIELDS)

# PointXYZIRCAEDT 出力（従来どおり）
POINTXYZIRCAEDT_FIELDS = [
    ("x", np.float32),
    ("y", np.float32),
    ("z", np.float32),
    ("intensity", np.uint8),
    ("return_type", np.uint8),
    ("channel", np.uint16),
    ("azimuth", np.float32),
    ("elevation", np.float32),
    ("distance", np.float32),
    ("time_stamp", np.uint32),
]
POINTXYZIRCAEDT_DTYPE = np.dtype(POINTXYZIRCAEDT_FIELDS)

POINT_STEP_XYZIRC = 16


def _endian_char(msg: PointCloud2) -> str:
    return ">" if msg.is_bigendian else "<"


def _unpack_one(fmt_base: str, data: bytes, offset: int, endian: str):
    return struct.unpack_from(endian + fmt_base, data, offset)[0]


def is_layout_xyzirc(msg: PointCloud2) -> bool:
    """autoware_pointcloud_preprocessor memory.cpp と同条件（フィールド順固定）。"""
    f = msg.fields
    if len(f) != 6:
        return False
    checks: List[Tuple[int, str, int, int]] = [
        (0, "x", 0, PointField.FLOAT32),
        (1, "y", 4, PointField.FLOAT32),
        (2, "z", 8, PointField.FLOAT32),
        (3, "intensity", 12, PointField.UINT8),
        (4, "return_type", 13, PointField.UINT8),
        (5, "channel", 14, PointField.UINT16),
    ]
    for idx, name, off, dt in checks:
        fld = f[idx]
        if fld.name != name or fld.offset != off or fld.datatype != dt or fld.count != 1:
            return False
    return msg.point_step == POINT_STEP_XYZIRC


def is_layout_xyzircaedt(msg: PointCloud2) -> bool:
    f = msg.fields
    if len(f) != 10:
        return False
    checks: List[Tuple[int, str, int, int]] = [
        (0, "x", 0, PointField.FLOAT32),
        (1, "y", 4, PointField.FLOAT32),
        (2, "z", 8, PointField.FLOAT32),
        (3, "intensity", 12, PointField.UINT8),
        (4, "return_type", 13, PointField.UINT8),
        (5, "channel", 14, PointField.UINT16),
        (6, "azimuth", 16, PointField.FLOAT32),
        (7, "elevation", 20, PointField.FLOAT32),
        (8, "distance", 24, PointField.FLOAT32),
        (9, "time_stamp", 28, PointField.UINT32),
    ]
    for idx, name, off, dt in checks:
        fld = f[idx]
        if fld.name != name or fld.offset != off or fld.datatype != dt or fld.count != 1:
            return False
    return msg.point_step >= 32


def is_layout_xyzi_autoware(msg: PointCloud2) -> bool:
    """PointXYZI: x,y,z,intensity いずれも FLOAT32、オフセット 0,4,8,12。"""
    f = msg.fields
    if len(f) != 4:
        return False
    exp = [
        ("x", 0, PointField.FLOAT32),
        ("y", 4, PointField.FLOAT32),
        ("z", 8, PointField.FLOAT32),
        ("intensity", 12, PointField.FLOAT32),
    ]
    for i, (name, off, dt) in enumerate(exp):
        fld = f[i]
        if fld.name != name or fld.offset != off or fld.datatype != dt or fld.count != 1:
            return False
    return True


def _field_by_name(msg: PointCloud2) -> Dict[str, PointField]:
    return {fld.name: fld for fld in msg.fields}


def is_layout_xyzi_loose(msg: PointCloud2) -> bool:
    """x,y,z が FLOAT32、intensity が FLOAT32 または UINT8（オフセットは msg に従う）。"""
    lu = _field_by_name(msg)
    for req in ("x", "y", "z", "intensity"):
        if req not in lu:
            return False
        if lu[req].datatype != PointField.FLOAT32 and not (
            req == "intensity" and lu[req].datatype == PointField.UINT8
        ):
            return False
        if lu[req].count != 1:
            return False
    return True


def is_layout_iradrt_numpy(msg: PointCloud2) -> bool:
    n = int(msg.width) * int(msg.height)
    if n <= 0:
        return False
    return int(msg.point_step) == int(POINTXYZIRADRT_DTYPE.itemsize) and len(msg.data) >= n * int(
        msg.point_step
    )


LayoutKind = Literal["pass", "xyzi_autoware", "xyzi_loose", "iradrt"]


def detect_layout(msg: PointCloud2) -> LayoutKind:
    if is_layout_xyzircaedt(msg) or is_layout_xyzirc(msg):
        return "pass"
    if is_layout_xyzi_autoware(msg):
        return "xyzi_autoware"
    if is_layout_xyzi_loose(msg):
        return "xyzi_loose"
    if is_layout_iradrt_numpy(msg):
        return "iradrt"
    names = [f"{fld.name}@{fld.offset}({fld.datatype})" for fld in msg.fields]
    raise ValueError(
        "未対応の PointCloud2 レイアウトです（PointXYZI / PointXYZIRC / PointXYZIRCAEDT / "
        f"PointXYZIRADRT のいずれでもありません）。fields={names} point_step={msg.point_step}"
    )


def _float32_fmt() -> str:
    return "f"


def _read_intensity_uint8(value, dt: int) -> int:
    if dt == PointField.FLOAT32:
        return int(max(0, min(255, round(float(value)))))
    if dt == PointField.UINT8:
        return int(value) & 0xFF
    raise ValueError(f"intensity の datatype が未対応です: {dt}")


def convert_xyzi_to_xyzirc(msg: PointCloud2, kind: Literal["xyzi_autoware", "xyzi_loose"]) -> PointCloud2:
    e = _endian_char(msg)
    n = int(msg.width) * int(msg.height)
    ps = int(msg.point_step)
    if n <= 0 or ps <= 0 or len(msg.data) < n * ps:
        raise ValueError("無効な width/height/point_step/data 長")

    if kind == "xyzi_autoware":
        ox, oy, oz, oi = 0, 4, 8, 12
        idt = PointField.FLOAT32
    else:
        lu = _field_by_name(msg)
        ox, oy, oz = lu["x"].offset, lu["y"].offset, lu["z"].offset
        oi = lu["intensity"].offset
        idt = lu["intensity"].datatype

    out = bytearray(n * POINT_STEP_XYZIRC)
    fmt_f = _float32_fmt()
    for i in range(n):
        base = i * ps
        x = _unpack_one(fmt_f, msg.data, base + ox, e)
        y = _unpack_one(fmt_f, msg.data, base + oy, e)
        z = _unpack_one(fmt_f, msg.data, base + oz, e)
        if idt == PointField.FLOAT32:
            raw_i = _unpack_one(fmt_f, msg.data, base + oi, e)
        else:
            raw_i = _unpack_one("B", msg.data, base + oi, e)
        iv = _read_intensity_uint8(raw_i, idt)
        struct.pack_into(e + "fffBBH", out, i * POINT_STEP_XYZIRC, x, y, z, iv, 0, 0)

    new_msg = PointCloud2()
    new_msg.header = msg.header
    new_msg.height = msg.height
    new_msg.width = msg.width
    new_msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
        PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
        PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
    ]
    new_msg.is_bigendian = msg.is_bigendian
    new_msg.point_step = POINT_STEP_XYZIRC
    new_msg.row_step = new_msg.point_step * new_msg.width
    new_msg.is_dense = msg.is_dense
    new_msg.data = bytes(out)
    return new_msg


def convert_iradrt_to_xyzircaedt(msg: PointCloud2) -> PointCloud2:
    arr = np.frombuffer(msg.data, dtype=POINTXYZIRADRT_DTYPE, count=int(msg.width) * int(msg.height))
    new_arr = np.zeros(arr.shape, dtype=POINTXYZIRCAEDT_DTYPE)
    new_arr["x"] = arr["x"]
    new_arr["y"] = arr["y"]
    new_arr["z"] = arr["z"]
    new_arr["intensity"] = np.clip(arr["intensity"], 0, 255).astype(np.uint8)
    new_arr["return_type"] = arr["return_type"]
    new_arr["channel"] = arr["ring"].astype(np.uint16)
    new_arr["azimuth"] = arr["azimuth"]
    new_arr["elevation"] = 0
    new_arr["distance"] = arr["distance"]
    new_arr["time_stamp"] = (arr["time_stamp"] * 1000).astype(np.uint32)

    new_msg = PointCloud2()
    new_msg.header = msg.header
    new_msg.height = msg.height
    new_msg.width = msg.width
    new_msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
        PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
        PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        PointField(name="azimuth", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="elevation", offset=20, datatype=PointField.FLOAT32, count=1),
        PointField(name="distance", offset=24, datatype=PointField.FLOAT32, count=1),
        PointField(name="time_stamp", offset=28, datatype=PointField.UINT32, count=1),
    ]
    new_msg.is_bigendian = msg.is_bigendian
    new_msg.point_step = int(POINTXYZIRCAEDT_DTYPE.itemsize)
    new_msg.row_step = new_msg.point_step * new_msg.width
    new_msg.is_dense = msg.is_dense
    new_msg.data = new_arr.tobytes()
    return new_msg


def convert_pointcloud_dispatch(msg: PointCloud2) -> PointCloud2:
    """変更不要のときは同一 msg インスタンスを返す（呼び出し側で is で元バイナリ流用可）。"""
    kind = detect_layout(msg)
    if kind == "pass":
        return msg
    if kind == "xyzi_autoware":
        return convert_xyzi_to_xyzirc(msg, "xyzi_autoware")
    if kind == "xyzi_loose":
        return convert_xyzi_to_xyzirc(msg, "xyzi_loose")
    if kind == "iradrt":
        return convert_iradrt_to_xyzircaedt(msg)
    raise AssertionError(f"internal: unknown layout kind {kind}")


def main() -> None:
    if len(sys.argv) != 4:
        print("Usage: python3 convert_pointcloud_type.py <input_bag> <output_bag> <topic_names>")
        print("  <topic_names>: comma-separated list of topic names to convert")
        print(
            "Example: python3 convert_pointcloud_type.py input_bag output_bag "
            "/sensing/lidar/concatenated/pointcloud"
        )
        sys.exit(1)
    input_bag, output_bag, topic_names = sys.argv[1:4]
    topic_list = [t.strip() for t in topic_names.split(",") if t.strip()]

    rclpy.init()
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
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

    found_topics = [t for t in topic_list if total_msgs[t] > 0]
    if not found_topics:
        print(
            "[WARNING]: No topics were found to convert. Please verify the specified topic name "
            "or input bag file."
        )
        rclpy.shutdown()
        sys.exit(1)

    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR]: Failed to reopen input bag file for conversion. {e}")
        rclpy.shutdown()
        sys.exit(1)

    try:
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=output_bag, storage_id="sqlite3"), converter_options)
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
    skipped_pass = {t: 0 for t in topic_list}
    failed = {t: 0 for t in topic_list}

    def print_progress() -> None:
        for t in topic_list:
            if total_msgs[t] > 0:
                percent = 100 * converted[t] // total_msgs[t]
                print(f"Converting [{t}]: {converted[t]}/{total_msgs[t]} ({percent}%)")

    try:
        while reader.has_next():
            try:
                topic_name, data, t = reader.read_next()
            except Exception as e:
                print(f"[ERROR]: Failed to read message. {e}")
                continue

            out_data = data
            if topic_name in topic_list:
                try:
                    msg = deserialize_message(data, PointCloud2)
                except Exception as e:
                    print(f"[ERROR]: Failed to deserialize message on topic '{topic_name}'. {e}")
                    failed[topic_name] += 1
                else:
                    try:
                        new_msg = convert_pointcloud_dispatch(msg)
                        if new_msg is msg:
                            out_data = data
                            skipped_pass[topic_name] += 1
                        else:
                            out_data = serialize_message(new_msg)
                            converted[topic_name] += 1
                            print_progress()
                    except Exception as e:
                        print(
                            f"[WARNING]: Convert skipped on '{topic_name}', copying original message. {e}",
                            file=sys.stderr,
                        )
                        out_data = data
                        failed[topic_name] += 1

            try:
                writer.write(topic_name, out_data, t)
            except Exception as e:
                print(f"[ERROR]: Failed to write message to output bag. {e}")
    except Exception as e:
        print(f"[ERROR]: Unexpected error during conversion. {e}")
        rclpy.shutdown()
        sys.exit(1)

    for t in topic_list:
        if total_msgs[t] > 0:
            print(f"Converting [{t}]: {total_msgs[t]}/{total_msgs[t]} (100%)")
    for t in topic_list:
        if skipped_pass[t]:
            print(f"  [{t}] already XYZIRC/AEDT (passthrough): {skipped_pass[t]} msgs")
        if failed[t]:
            print(f"  [{t}] deserialize/convert warnings (original copied): {failed[t]} msgs")
    print(f"Conversion complete. Output bag: {output_bag}")
    rclpy.shutdown()


# 後方互換: 旧名
convert_pointcloud = convert_iradrt_to_xyzircaedt

if __name__ == "__main__":
    main()
