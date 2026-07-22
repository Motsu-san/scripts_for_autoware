#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rosbagに別のrosbagから特定のトピック（リマップ版）を追加するスクリプト

使用方法:
    python3 merge_rosbag_topic.py \\
        --source-bag /path/to/original.bag \\
        --add-bag /path/to/recorded.bag \\
        --add-topics /topic1 /topic2 \\
        --remap /topic1:=/topic1_recorded \\
        --output-bag /path/to/merged.bag
"""

import argparse
import sys
from pathlib import Path
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata


def merge_rosbag_with_remap(source_bag, add_bag, add_topics, remap_dict, output_bag):
    """rosbagに別のrosbagから特定のトピックをリマップして追加"""

    # リーダーの設定
    source_storage_options = StorageOptions(uri=str(source_bag), storage_id='sqlite3')
    add_storage_options = StorageOptions(uri=str(add_bag), storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    # ライターの設定
    output_storage_options = StorageOptions(uri=str(output_bag), storage_id='sqlite3')

    # ソースバッグを読み込み
    source_reader = SequentialReader()
    source_reader.open(source_storage_options, converter_options)

    # 追加バッグを読み込み
    add_reader = SequentialReader()
    add_reader.open(add_storage_options, converter_options)

    # ライターを作成
    writer = SequentialWriter()
    writer.open(output_storage_options, converter_options)

    # ソースバッグのトピックメタデータをコピー
    source_topics = source_reader.get_all_topics_and_types()
    for topic_metadata in source_topics:
        writer.create_topic(topic_metadata)

    # 追加バッグから指定トピックのメタデータをリマップしてコピー
    add_topics_set = set(add_topics)
    add_all_topics = add_reader.get_all_topics_and_types()

    for topic_metadata in add_all_topics:
        if topic_metadata.name in add_topics_set:
            # リマップ先のトピック名を取得
            new_name = remap_dict.get(topic_metadata.name, topic_metadata.name)

            # 新しいトピックメタデータを作成
            new_metadata = TopicMetadata(
                name=new_name,
                type=topic_metadata.type,
                serialization_format=topic_metadata.serialization_format
            )
            writer.create_topic(new_metadata)
            print(f"追加トピック: {topic_metadata.name} -> {new_name} ({topic_metadata.type})", flush=True)

    # ソースバッグの全メッセージを書き込み
    print("ソースバッグのメッセージを書き込み中...", flush=True)
    source_msg_count = 0
    while source_reader.has_next():
        topic_name, data, timestamp = source_reader.read_next()
        writer.write(topic_name, data, timestamp)
        source_msg_count += 1
        if source_msg_count % 10000 == 0:
            print(f"  {source_msg_count} メッセージ書き込み完了", flush=True)

    print(f"ソースバッグ: {source_msg_count} メッセージ書き込み完了", flush=True)

    # 追加バッグの指定トピックのメッセージをリマップして書き込み
    print("追加バッグのメッセージを書き込み中...", flush=True)
    add_msg_count = 0
    while add_reader.has_next():
        topic_name, data, timestamp = add_reader.read_next()
        if topic_name in add_topics_set:
            # リマップ先のトピック名で書き込み
            new_name = remap_dict.get(topic_name, topic_name)
            writer.write(new_name, data, timestamp)
            add_msg_count += 1
            if add_msg_count % 1000 == 0:
                print(f"  {add_msg_count} メッセージ書き込み完了", flush=True)

    print(f"追加バッグ: {add_msg_count} メッセージ書き込み完了", flush=True)

    # クリーンアップ
    del writer
    del source_reader
    del add_reader

    print(f"\nマージ完了: {output_bag}", flush=True)
    print(f"  ソースバッグ: {source_msg_count} メッセージ", flush=True)
    print(f"  追加バッグ: {add_msg_count} メッセージ", flush=True)
    print(f"  合計: {source_msg_count + add_msg_count} メッセージ", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description='rosbagに別のrosbagから特定のトピックをリマップして追加',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--source-bag', required=True, help='元のrosbagのパス')
    parser.add_argument('--add-bag', required=True, help='追加するrosbagのパス')
    parser.add_argument('--add-topics', nargs='+', required=True, help='追加するトピック')
    parser.add_argument('--remap', action='append', default=[], help='トピック名をリマップ（OLD:=NEW形式）')
    parser.add_argument('--output-bag', required=True, help='出力先rosbagのパス')

    args = parser.parse_args()

    # パスをPathオブジェクトに変換
    source_bag = Path(args.source_bag)
    add_bag = Path(args.add_bag)
    output_bag = Path(args.output_bag)

    # バッグの存在確認
    if not source_bag.exists():
        print(f"エラー: ソースバッグが見つかりません: {source_bag}", file=sys.stderr)
        sys.exit(1)

    if not add_bag.exists():
        print(f"エラー: 追加バッグが見つかりません: {add_bag}", file=sys.stderr)
        sys.exit(1)

    # リマップ辞書を作成
    remap_dict = {}
    for remap_str in args.remap:
        if ':=' not in remap_str:
            print(f"警告: 無効なリマップ形式: {remap_str} (形式: OLD:=NEW)", file=sys.stderr)
            continue
        old, new = remap_str.split(':=', 1)
        remap_dict[old] = new

    print("=" * 60)
    print("rosbagマージスクリプト")
    print("=" * 60)
    print(f"ソースバッグ: {source_bag}")
    print(f"追加バッグ: {add_bag}")
    print(f"追加トピック: {', '.join(args.add_topics)}")
    print(f"リマップ: {remap_dict}")
    print(f"出力先: {output_bag}")
    print("=" * 60)
    print()

    try:
        merge_rosbag_with_remap(source_bag, add_bag, args.add_topics, remap_dict, output_bag)
    except Exception as e:
        print(f"エラー: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
