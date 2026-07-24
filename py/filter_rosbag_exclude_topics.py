#!/usr/bin/env python3
"""
rosbag2 (sqlite3) から指定トピックを除外した複製を作る。

使用例:
  python3 filter_rosbag_exclude_topics.py \\
    --input-bag /path/to/bag_dir_or.db3 \\
    --output-bag /path/to/output_bag_dir \\
    --exclude-topic /localization/pose_twist_fusion_filter/biased_pose_with_covariance \\
    --exclude-topic /localization/pose_estimator/pose_with_covariance
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Set

from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    TopicMetadata,
)


def resolve_bag_uri(path: Path) -> str:
    if path.is_dir():
        return str(path)
    if path.is_file() and path.suffix == ".db3":
        return str(path.parent)
    raise FileNotFoundError(f"bag path not found: {path}")


def filter_bag(
    input_uri: str,
    output_uri: str,
    exclude_topics: Set[str],
) -> tuple[int, int]:
    conv = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(StorageOptions(uri=input_uri, storage_id="sqlite3"), conv)

    out_path = Path(output_uri)
    if out_path.exists():
        raise FileExistsError(f"output already exists: {output_uri}")

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=output_uri, storage_id="sqlite3"), conv)

    kept_topics = 0
    for tm in reader.get_all_topics_and_types():
        if tm.name in exclude_topics:
            print(f"  exclude topic: {tm.name} (type={tm.type})", flush=True)
            continue
        writer.create_topic(
            TopicMetadata(
                tm.name,
                tm.type,
                tm.serialization_format,
                tm.offered_qos_profiles,
            )
        )
        kept_topics += 1

    skipped = 0
    written = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic in exclude_topics:
            skipped += 1
            continue
        writer.write(topic, data, ts)
        written += 1
        if written % 50000 == 0:
            print(f"  written {written}, skipped {skipped}", flush=True)

    del writer
    del reader
    print(
        f"Done: kept_topics={kept_topics} written={written} skipped={skipped} -> {output_uri}",
        flush=True,
    )
    return written, skipped


def main() -> int:
    parser = argparse.ArgumentParser(description="Exclude topics from a rosbag2 sqlite3 bag")
    parser.add_argument(
        "--input-bag",
        type=Path,
        required=True,
        help="入力 bag ディレクトリまたは .db3 ファイル",
    )
    parser.add_argument(
        "--output-bag",
        type=Path,
        required=True,
        help="出力 bag ディレクトリ(新規作成)",
    )
    parser.add_argument(
        "--exclude-topic",
        action="append",
        default=[],
        help="除外するトピック(複数指定可)",
    )
    args = parser.parse_args()

    if not args.exclude_topic:
        print("Error: --exclude-topic を1つ以上指定してください", file=sys.stderr)
        return 2

    try:
        input_uri = resolve_bag_uri(args.input_bag)
    except FileNotFoundError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    exclude = set(args.exclude_topic)
    print(f"Input:  {input_uri}", flush=True)
    print(f"Output: {args.output_bag}", flush=True)
    print(f"Exclude: {sorted(exclude)}", flush=True)

    try:
        filter_bag(input_uri, str(args.output_bag), exclude)
    except FileExistsError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
