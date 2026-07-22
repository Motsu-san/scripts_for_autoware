#!/usr/bin/env python3
"""
Subscribe to /sensing/gnss/pose_with_covariance and publish the first message as initial pose
to /initialpose and /localization_node/initialpose (for unified_localization).
Use when LAUNCH_UNIFIED_LOCALIZATION is true and pose_initializer is not launched.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
import argparse


class GnssToInitialPose(Node):
    def __init__(self, timeout_sec=60.0, once=True):
        super().__init__("gnss_to_initial_pose")
        self.timeout_sec = timeout_sec
        self.once = once
        self.done = False

        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.pub_localization_node = self.create_publisher(
            PoseWithCovarianceStamped, "/localization_node/initialpose", 10
        )

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self.cb_gnss,
            10,
        )
        self.get_logger().info(
            "Waiting for /sensing/gnss/pose_with_covariance (timeout=%s s)..." % timeout_sec
        )

    def cb_gnss(self, msg):
        if self.done and self.once:
            return
        self.done = True
        # Ensure frame_id is map (gnss_poser may already use map)
        msg.header.frame_id = "map"
        self.pub_initialpose.publish(msg)
        self.pub_localization_node.publish(msg)
        self.get_logger().info(
            "Published initial pose from GNSS: x=%.2f, y=%.2f, z=%.2f"
            % (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
        )
        self.get_logger().info(
            "  -> /initialpose and /localization_node/initialpose"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Publish first GNSS pose as initial pose for unified_localization"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for first GNSS message (default: 60)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        default=True,
        help="Exit after publishing once (default: True)",
    )
    parser.add_argument(
        "--no-once",
        action="store_false",
        dest="once",
        help="Keep running and republish on every GNSS message",
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = GnssToInitialPose(timeout_sec=args.timeout, once=args.once)

    try:
        import time
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.done and node.once:
                break
            if (time.monotonic() - start) >= args.timeout:
                node.get_logger().warn(
                    "Timeout waiting for /sensing/gnss/pose_with_covariance"
                )
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if node.done else 1


if __name__ == "__main__":
    sys.exit(main())
