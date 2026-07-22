#!/usr/bin/env python3
"""
Script to check the diagnostics publishing frequency of EKF localizer
Please run this script during rosbag playback.
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
import time
from collections import deque


class EKFDiagnosticsFrequencyChecker(Node):
    def __init__(self):
        super().__init__('ekf_diagnostics_frequency_checker')

        # EKF localizer node name (typically "ekf_localizer")
        self.ekf_node_name = "ekf_localizer"
        self.target_name = f"localization: {self.ekf_node_name}"

        # Queue to record timestamps (max 500 entries)
        # Ensures sufficient sample size even at 1Hz
        self.timestamps = deque(maxlen=500)

        # For debugging: record received diagnostics names
        self.seen_names = set()
        self.debug_count = 0
        self.debug_limit = 20  # Display debug info for first 20 messages
        self.matched_count = 0  # Count matched diagnostics

        # サブスクライバー
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )

        # Timer to periodically display statistics (every 2 seconds)
        # Maintains appropriate update frequency for statistics even at 1Hz
        self.timer = self.create_timer(2.0, self.print_statistics)

        self.get_logger().info(f'EKF diagnostics frequency checker started')
        self.get_logger().info(f'Looking for diagnostics with name: "{self.target_name}"')
        self.get_logger().info('Press Ctrl+C to stop')

    def diagnostics_callback(self, msg):
        """Callback when diagnostics message is received"""
        # Debug: display diagnostic names for first few messages
        if self.debug_count < self.debug_limit:
            for status in msg.status:
                if status.name not in self.seen_names:
                    self.seen_names.add(status.name)
                    hardware_id_str = f', hardware_id: "{status.hardware_id}"' if hasattr(status, 'hardware_id') and status.hardware_id else ''
                    self.get_logger().info(f'[DEBUG] Found diagnostics name: "{status.name}"{hardware_id_str}')
            self.debug_count += 1

        # Find EKF localizer diagnostics
        found = False
        matched_name = None
        matched_hardware_id = None

        for status in msg.status:
            hid = (status.hardware_id or "").strip()
            name = status.name or ""

            # hardware_id: node base name or namespaced "/ns/ekf_localizer" / "ns.ekf_localizer"
            hid_ok = False
            if hid:
                hid_ok = (
                    hid == self.ekf_node_name
                    or hid.endswith("/" + self.ekf_node_name)
                    or hid.endswith("." + self.ekf_node_name)
                )

            # Name: diagnostic_updater uses "ekf_localizer: localization: ekf_localizer" style
            name_ok_main = self.target_name in name and "callback_" not in name.lower()
            name_ok_callback = self.target_name in name and "callback_" in name.lower()

            if hid_ok and (name_ok_main or name_ok_callback):
                matched_name = status.name
                matched_hardware_id = status.hardware_id
                if self.debug_count < self.debug_limit:
                    self.get_logger().info(
                        f'[INFO] Found EKF diagnostics with name: "{status.name}", hardware_id: "{status.hardware_id}"'
                    )
                found = True
                break
            # Fallback: diagnostic_updater name prefix + localization string (no hardware_id reliance)
            if self.target_name in name or name.startswith(f"{self.ekf_node_name}: localization:"):
                matched_name = status.name
                if self.debug_count < self.debug_limit:
                    self.get_logger().info(f'[INFO] Found EKF diagnostics (name fallback): "{status.name}"')
                found = True
                break

        if found:
            self.matched_count += 1
            # Output log only for first few times
            if self.matched_count <= 5:
                self.get_logger().info(
                    f'[INFO] Matched EKF diagnostics ({self.matched_count}): "{matched_name}"'
                    + (f', hardware_id: "{matched_hardware_id}"' if matched_hardware_id else "")
                )

            # Message stamp = sim time when use_sim_time; independent of rosbag --rate wall-clock stretch
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.timestamps.append(t)

    def print_statistics(self):
        """Display statistics"""
        if len(self.timestamps) < 2:
            if self.debug_count >= self.debug_limit and len(self.seen_names) > 0:
                # Look for EKF-related diagnostics names
                ekf_related = [name for name in sorted(self.seen_names) if "ekf" in name.lower()]
                if ekf_related:
                    self.get_logger().warn(f'EKF diagnostics not found with exact name. Found EKF-related diagnostics: {ekf_related}')
                else:
                    self.get_logger().warn(f'EKF diagnostics not found. Seen diagnostics names: {sorted(self.seen_names)[:10]}...')
                self.get_logger().warn(f'Looking for: "{self.target_name}" or hardware_id="ekf_localizer"')
            else:
                if len(self.timestamps) == 1:
                    self.get_logger().info(f'Received 1 message. Waiting for next message...')
                else:
                    self.get_logger().info('Waiting for EKF diagnostics messages...')
            return

        # Calculate time intervals
        intervals = []
        for i in range(1, len(self.timestamps)):
            interval = self.timestamps[i] - self.timestamps[i-1]
            intervals.append(interval)

        if intervals:
            # Stamp-based intervals are in sim time (~1s for 1Hz diagnostics).
            # Do not cap at 2s: wall-clock gaps with use_sim_time=false and slow --rate exceeded 2s
            # and discarded all samples ("No valid intervals found").
            valid_intervals = [i for i in intervals if 0.001 < i < 120.0]

            if valid_intervals:
                avg_interval = sum(valid_intervals) / len(valid_intervals)
                min_interval = min(valid_intervals)
                max_interval = max(valid_intervals)
                avg_frequency = 1.0 / avg_interval if avg_interval > 0 else 0.0

                # Also calculate median (less affected by outliers)
                sorted_intervals = sorted(valid_intervals)
                median_interval = sorted_intervals[len(sorted_intervals) // 2]
                median_frequency = 1.0 / median_interval if median_interval > 0 else 0.0

                self.get_logger().info('=' * 60)
                self.get_logger().info(f'Number of messages received: {len(self.timestamps)}')
                self.get_logger().info(f'Number of valid intervals: {len(valid_intervals)}/{len(intervals)}')
                self.get_logger().info(f'Average publishing period: {avg_interval*1000:.3f} ms')
                self.get_logger().info(f'Median period: {median_interval*1000:.3f} ms')
                self.get_logger().info(f'Minimum publishing period: {min_interval*1000:.3f} ms')
                self.get_logger().info(f'Maximum publishing period: {max_interval*1000:.3f} ms')
                self.get_logger().info(f'Average publishing frequency: {avg_frequency:.3f} Hz')
                self.get_logger().info(f'Median frequency: {median_frequency:.3f} Hz')

                # Determine expected value (auto-detect in range 1Hz to 100Hz)
                if 0.9 <= avg_frequency <= 1.1:
                    self.get_logger().info('✓ Running at approximately 1Hz')
                elif 9.0 <= avg_frequency <= 11.0:
                    self.get_logger().info('✓ Running at approximately 10Hz')
                elif 45.0 <= avg_frequency <= 55.0:
                    self.get_logger().info('✓ Running at approximately 50Hz')
                else:
                    self.get_logger().info(f'Measured frequency: {avg_frequency:.3f} Hz')
                self.get_logger().info('=' * 60)
            else:
                self.get_logger().warn('No valid intervals found')


def main(args=None):
    rclpy.init(args=args)

    checker = EKFDiagnosticsFrequencyChecker()

    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
