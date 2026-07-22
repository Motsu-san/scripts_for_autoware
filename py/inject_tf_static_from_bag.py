#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
別の sqlite3 rosbag から /tf_static を取り出し、宛先 bag の先頭時刻に合わせて注入した複製を作る。

宛先の metadata.starting_time より margin_ns 手前を「最後の tf_static」の時刻とし、
ソース内の相対間隔を保ったまま過去方向へ写す（先頭の他トピックより必ず前になるようにする）。

使用例:
  source /opt/ros/humble/setup.bash
  python3 inject_tf_static_from_bag.py \\
    --dest-bag .../merge_temp_0.db3 \\
    --src-tf-bag .../merge_temp_0.db3 \\
    --output-bag-dir .../merge_temp_0_with_tf_static
"""

from __future__ import annotations

import argparse
import sys

from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    SequentialWriter,
    StorageFilter,
    StorageOptions,
    TopicMetadata,
)


def inject_tf_static(dest_uri: str, src_uri: str, out_uri: str, margin_ns: int) -> None:
    conv = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    dest_reader = SequentialReader()
    dest_reader.open(StorageOptions(uri=dest_uri, storage_id="sqlite3"), conv)
    meta = dest_reader.get_metadata()
    start_ns = meta.starting_time.nanoseconds

    src_reader = SequentialReader()
    src_reader.open(StorageOptions(uri=src_uri, storage_id="sqlite3"), conv)
    src_reader.set_filter(StorageFilter(topics=["/tf_static"]))
    tf_msgs: list[tuple[str, bytes, int]] = []
    while src_reader.has_next():
        topic, data, ts = src_reader.read_next()
        tf_msgs.append((topic, data, ts))
    del src_reader

    if not tf_msgs:
        print("エラー: ソース bag に /tf_static メッセージがありません", file=sys.stderr)
        sys.exit(1)

    tf_msgs.sort(key=lambda x: x[2])
    last_ts = tf_msgs[-1][2]
    remapped: list[tuple[str, bytes, int]] = []
    for topic, data, ts in tf_msgs:
        new_ts = start_ns - margin_ns - (last_ts - ts)
        if new_ts < 0:
            print("エラー: 写し替え後タイムスタンプが負になりました。--margin-ns を小さくしてください。", file=sys.stderr)
            sys.exit(1)
        remapped.append((topic, data, new_ts))

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=out_uri, storage_id="sqlite3"), conv)
    for tm in dest_reader.get_all_topics_and_types():
        writer.create_topic(
            TopicMetadata(
                tm.name,
                tm.type,
                tm.serialization_format,
                tm.offered_qos_profiles,
            )
        )

    for topic, data, ts in remapped:
        writer.write(topic, data, ts)

    n = 0
    while dest_reader.has_next():
        topic, data, ts = dest_reader.read_next()
        writer.write(topic, data, ts)
        n += 1
        if n % 50000 == 0:
            print(f"  宛先メッセージ {n} 件書き込み…", flush=True)

    del writer
    del dest_reader
    print(
        f"完了: /tf_static {len(remapped)} 件を注入し、宛先 {n} 件を複製しました -> {out_uri}",
        flush=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="宛先 rosbag に別 bag の /tf_static を先頭時刻に合わせて注入して複製する",
    )
    parser.add_argument("--dest-bag", required=True, help="宛先 sqlite3 の .db3（単体ファイル可）")
    parser.add_argument("--src-tf-bag", required=True, help="/tf_static を取り出す sqlite3 の .db3")
    parser.add_argument(
        "--output-bag-dir",
        required=True,
        help="まだ存在しない出力ディレクトリ（metadata.yaml と *_0.db3 が作成される）",
    )
    parser.add_argument(
        "--margin-ns",
        type=int,
        default=1_000_000,
        help="先頭メッセージ時刻より手前に置くマージン（ナノ秒）。既定 1ms",
    )
    args = parser.parse_args()

    inject_tf_static(args.dest_bag, args.src_tf_bag, args.output_bag_dir, args.margin_ns)


if __name__ == "__main__":
    main()
