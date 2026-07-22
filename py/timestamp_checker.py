#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sys
import argparse
from datetime import datetime

class TimestampChecker(Node):
    def __init__(self, topic_name):
        super().__init__('timestamp_checker')

        # 前回のタイムスタンプを保存
        self.previous_timestamp = None
        self.message_count = 0
        self.regression_count = 0

        # 統計情報
        self.max_regression = 0.0
        self.total_regression_time = 0.0

        # サブスクライバーを作成
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.pointcloud_callback,
            10
        )

        self.get_logger().info(f'Timestamp checker started. Monitoring {topic_name}')
        print("=" * 80)
        print("ROS2 Timestamp Regression Checker")
        print("=" * 80)
        print(f"{'Count':<8} {'Timestamp (sec.nsec)':<20} {'Delta (s)':<12} {'Status':<15} {'Time':<20}")
        print("-" * 80)

    def pointcloud_callback(self, msg):
        self.message_count += 1

        # 現在のタイムスタンプを取得
        current_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # タイムスタンプの表示形式
        timestamp_str = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"

        if self.previous_timestamp is not None:
            # タイムスタンプの差を計算
            delta = current_timestamp - self.previous_timestamp

            if delta < 0:
                # 逆行を検出
                self.regression_count += 1
                self.total_regression_time += abs(delta)
                if abs(delta) > self.max_regression:
                    self.max_regression = abs(delta)

                status = f"REGRESSION!"
                print(f"{self.message_count:<8} {timestamp_str:<20} {delta:<12.6f} {status:<15} {current_time:<20}")

                # 警告メッセージ
                self.get_logger().warn(f"Timestamp regression detected: {delta:.6f}s backwards")

            elif delta == 0:
                status = "DUPLICATE"
                print(f"{self.message_count:<8} {timestamp_str:<20} {delta:<12.6f} {status:<15} {current_time:<20}")
            else:
                status = "OK"
                if self.message_count % 10 == 0:  # 10メッセージごとに表示
                    print(f"{self.message_count:<8} {timestamp_str:<20} {delta:<12.6f} {status:<15} {current_time:<20}")
        else:
            status = "FIRST"
            print(f"{self.message_count:<8} {timestamp_str:<20} {'N/A':<12} {status:<15} {current_time:<20}")

        self.previous_timestamp = current_timestamp

        # 統計情報を定期的に表示
        if self.message_count % 100 == 0:
            self.print_statistics()

    def print_statistics(self):
        print("\n" + "=" * 80)
        print("STATISTICS")
        print("=" * 80)
        print(f"Total messages processed: {self.message_count}")
        print(f"Timestamp regressions detected: {self.regression_count}")
        if self.message_count > 0:
            regression_rate = (self.regression_count / self.message_count) * 100
            print(f"Regression rate: {regression_rate:.2f}%")
        if self.regression_count > 0:
            print(f"Maximum regression: {self.max_regression:.6f}s")
            avg_regression = self.total_regression_time / self.regression_count
            print(f"Average regression: {avg_regression:.6f}s")
        print("=" * 80)
        print(f"{'Count':<8} {'Timestamp (sec.nsec)':<20} {'Delta (s)':<12} {'Status':<15} {'Time':<20}")
        print("-" * 80)

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='ROS2 Timestamp Regression Checker')
    parser.add_argument(
        'topic',
        nargs='?',
        default='/localization/util/downsample/pointcloud',
        help='Topic name to monitor (default: /localization/util/downsample/pointcloud)'
    )
    args = parser.parse_args()

    rclpy.init()

    try:
        checker = TimestampChecker(args.topic)
        print(f"Monitoring topic: {args.topic}")
        print("Press Ctrl+C to stop and show final statistics\n")
        rclpy.spin(checker)
    except KeyboardInterrupt:
        print("\n\nStopping timestamp checker...")
        if 'checker' in locals():
            print("\n" + "=" * 80)
            print("FINAL STATISTICS")
            print("=" * 80)
            print(f"Total messages processed: {checker.message_count}")
            print(f"Timestamp regressions detected: {checker.regression_count}")
            if checker.message_count > 0:
                regression_rate = (checker.regression_count / checker.message_count) * 100
                print(f"Regression rate: {regression_rate:.2f}%")
            if checker.regression_count > 0:
                print(f"Maximum regression: {checker.max_regression:.6f}s")
                avg_regression = checker.total_regression_time / checker.regression_count
                print(f"Average regression: {avg_regression:.6f}s")
                print(f"Total regression time: {checker.total_regression_time:.6f}s")
            print("=" * 80)
    finally:
        if 'checker' in locals():
            checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
