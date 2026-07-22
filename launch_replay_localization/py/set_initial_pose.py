#!/usr/bin/env python3
"""
Set initial pose for localization via ROS 2 topic
Supports both command-line arguments and YAML file input
"""
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import SetBool
import sys
import yaml
import os
import argparse


def unix_sec_to_time_msg(unix_sec: float) -> Time:
    sec = int(unix_sec)
    nanosec = int(round((unix_sec - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    t = Time()
    t.sec = sec
    t.nanosec = nanosec
    return t


class InitialPoseSetter(Node):
    def __init__(
        self,
        pose_data,
        skip_initial_localization=False,
        stamp_unix_sec=None,
    ):
        super().__init__('initial_pose_setter')

        # Determine which topic(s) to publish to
        if skip_initial_localization:
            # Directly to EKF localizer (skips pose estimation)
            self.topic_names = ['/initialpose3d']
            self.get_logger().info('Publishing directly to EKF localizer (skipping pose estimation)')
        else:
            # To pose initializer and/or unified_localization (localization_node subscribes to /localization_node/initialpose)
            self.topic_names = ['/initialpose', '/localization_node/initialpose']
            self.get_logger().info('Publishing to /initialpose and /localization_node/initialpose (pose initializer / unified_localization)')

        self._pub_list = [
            self.create_publisher(PoseWithCovarianceStamped, name, 10)
            for name in self.topic_names
        ]
        self.publisher = self._pub_list[0]  # for compatibility

        # Wait for publisher to be ready
        self.get_logger().info('Waiting for subscribers...')
        import time
        time.sleep(1.0)

        # Create and publish initial pose
        msg = PoseWithCovarianceStamped()
        if stamp_unix_sec is not None:
            msg.header.stamp = unix_sec_to_time_msg(float(stamp_unix_sec))
            self.get_logger().info(
                f'Using explicit header.stamp from --stamp-unix-sec: {stamp_unix_sec}'
            )
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Extract pose from data
        pose = pose_data['pose']['pose']
        msg.pose.pose.position.x = float(pose['position']['x'])
        msg.pose.pose.position.y = float(pose['position']['y'])
        msg.pose.pose.position.z = float(pose['position']['z'])

        # Save for map tile prefetch
        self._initial_x = msg.pose.pose.position.x
        self._initial_y = msg.pose.pose.position.y

        msg.pose.pose.orientation.x = float(pose['orientation']['x'])
        msg.pose.pose.orientation.y = float(pose['orientation']['y'])
        msg.pose.pose.orientation.z = float(pose['orientation']['z'])
        msg.pose.pose.orientation.w = float(pose['orientation']['w'])

        # Set covariance from YAML or use defaults
        if 'covariance' in pose_data['pose']:
            covariance = pose_data['pose']['covariance']
            # Ensure covariance has 36 elements
            if len(covariance) < 36:
                covariance.extend([0.0] * (36 - len(covariance)))
            msg.pose.covariance = covariance[:36]
        else:
            # Default covariance values
            msg.pose.covariance = [0.0] * 36
            msg.pose.covariance[0] = 0.25   # x variance
            msg.pose.covariance[7] = 0.25   # y variance
            msg.pose.covariance[14] = 0.01  # z variance
            msg.pose.covariance[21] = 0.01  # rotation x variance
            msg.pose.covariance[28] = 0.01  # rotation y variance
            msg.pose.covariance[35] = 0.06  # rotation z variance

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.get_logger().info(f'Publishing initial pose: x={x:.2f}, y={y:.2f}, z={z:.2f}')
        # Publish once only: repeating the same initial pose caused pose_initializer to run
        # multiple full cycles (EKF/NDT deactivate → ndt_align → activate) per message,
        # which was disruptive during bag replay.
        for pub in self._pub_list:
            pub.publish(msg)

        self.get_logger().info(f'Initial pose published successfully to {", ".join(self.topic_names)}')

        # If skipping initial localization, activate EKF localizer and NDT scan matcher
        if skip_initial_localization:
            # Stop pose_initializer node to prevent it from deactivating EKF/NDT
            self.stop_pose_initializer()
            self.activate_ekf_localizer()
            self.activate_ndt_scan_matcher()

    def activate_ekf_localizer(self):
        """Activate EKF localizer by calling trigger_node service"""
        import time
        time.sleep(2.0)  # Wait for the initial pose to be processed by EKF localizer

        service_name = '/localization/pose_twist_fusion_filter/trigger_node'
        self.get_logger().info(f'Calling service {service_name} to activate EKF localizer...')

        client = self.create_client(SetBool, service_name)

        # Wait for service to be available (retry up to 10 seconds)
        max_wait_time = 10.0
        wait_interval = 0.5
        waited_time = 0.0

        while waited_time < max_wait_time:
            if client.wait_for_service(timeout_sec=wait_interval):
                break
            waited_time += wait_interval
            self.get_logger().info(f'Waiting for service {service_name}... ({waited_time:.1f}s)')

        if not client.service_is_ready():
            self.get_logger().error(f'Service {service_name} is not available after {max_wait_time}s')
            return

        # Call service to activate EKF localizer
        request = SetBool.Request()
        request.data = True  # True means activate

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('EKF localizer activated successfully')
                else:
                    self.get_logger().warn('EKF localizer activation returned success=False')
            except Exception as e:
                self.get_logger().error(f'Failed to activate EKF localizer: {e}')
        else:
            self.get_logger().error('Service call timed out')

    def wait_for_pointcloud_map(self, timeout_sec=60.0):
        """Wait until pcd_loader_service is available, then prefetch tiles around initial pose."""
        from autoware_map_msgs.srv import GetDifferentialPointCloudMap
        self.get_logger().info('Waiting for pcd_loader_service to be available...')
        client = self.create_client(GetDifferentialPointCloudMap, '/map/get_differential_pointcloud_map')
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(f'pcd_loader_service not available within {timeout_sec}s. Proceeding anyway.')
            self.destroy_client(client)
            return

        self.get_logger().info('pcd_loader_service is available. Prefetching map tiles around initial pose...')

        # Request tiles around the initial pose position
        request = GetDifferentialPointCloudMap.Request()
        request.area.center_x = self._initial_x
        request.area.center_y = self._initial_y
        request.area.radius = 100.0
        request.cached_ids = []

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.done():
            try:
                response = future.result()
                n_tiles = len(response.new_pointcloud_with_ids)
                self.get_logger().info(f'Map tiles prefetched: {n_tiles} tile(s) received around initial pose.')
            except Exception as e:
                self.get_logger().warn(f'Map tile prefetch failed: {e}. Proceeding anyway.')
        else:
            self.get_logger().warn('Map tile prefetch timed out. Proceeding anyway.')

        self.destroy_client(client)

    def activate_ndt_scan_matcher(self):
        """Activate NDT scan matcher by calling trigger_node service"""
        import time
        time.sleep(0.5)  # Wait a bit after EKF activation

        # Ensure pointcloud map is loaded before activating NDT
        self.wait_for_pointcloud_map(timeout_sec=60.0)

        service_name = '/localization/pose_estimator/trigger_node'
        self.get_logger().info(f'Calling service {service_name} to activate NDT scan matcher...')

        client = self.create_client(SetBool, service_name)

        # Wait for service to be available (retry up to 10 seconds)
        max_wait_time = 10.0
        wait_interval = 0.5
        waited_time = 0.0

        while waited_time < max_wait_time:
            if client.wait_for_service(timeout_sec=wait_interval):
                break
            waited_time += wait_interval
            self.get_logger().info(f'Waiting for service {service_name}... ({waited_time:.1f}s)')

        if not client.service_is_ready():
            self.get_logger().error(f'Service {service_name} is not available after {max_wait_time}s')
            return

        # Call service to activate NDT scan matcher
        request = SetBool.Request()
        request.data = True  # True means activate

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('NDT scan matcher activated successfully')
                else:
                    self.get_logger().warn('NDT scan matcher activation returned success=False')
            except Exception as e:
                self.get_logger().error(f'Failed to activate NDT scan matcher: {e}')
        else:
            self.get_logger().error('Service call timed out')

    def stop_pose_initializer(self):
        """Stop pose_initializer node to prevent it from deactivating EKF/NDT"""
        import subprocess
        import time

        self.get_logger().info('Stopping pose_initializer node...')

        # Find and kill pose_initializer_node process
        try:
            # Find the process
            result = subprocess.run(
                ['pgrep', '-f', 'pose_initializer_node'],
                capture_output=True,
                text=True
            )

            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid:
                        try:
                            subprocess.run(['kill', '-SIGTERM', pid], check=True)
                            self.get_logger().info(f'Stopped pose_initializer_node (PID: {pid})')
                        except subprocess.CalledProcessError as e:
                            self.get_logger().warn(f'Failed to stop pose_initializer_node (PID: {pid}): {e}')

                # Wait a bit for the process to terminate
                time.sleep(1.0)
                self.get_logger().info('pose_initializer node stopped successfully')
            else:
                self.get_logger().warn('pose_initializer_node process not found (may already be stopped)')
        except Exception as e:
            self.get_logger().error(f'Failed to stop pose_initializer node: {e}')


def load_from_yaml(yaml_file):
    """Load initial pose from YAML file"""
    if not os.path.exists(yaml_file):
        print(f"Error: YAML file not found: {yaml_file}")
        sys.exit(1)

    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    if 'pose' not in data:
        print(f"Error: Invalid YAML format. 'pose' key not found in {yaml_file}")
        sys.exit(1)

    return data


def load_from_args(args):
    """Load initial pose from command-line arguments"""
    if len(args) != 7:
        print("Error: Expected 7 arguments (x y z qx qy qz qw)")
        print("Usage: set_initial_pose.py <x> <y> <z> <qx> <qy> <qz> <qw>")
        sys.exit(1)

    x, y, z, qx, qy, qz, qw = map(float, args)

    return {
        'pose': {
            'pose': {
                'position': {'x': x, 'y': y, 'z': z},
                'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}
            }
        }
    }


def main():
    parser = argparse.ArgumentParser(
        description='Set initial pose for localization via ROS 2 topic',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Start pose estimation (default behavior)
  set_initial_pose.py /path/to/initial_pose.yaml
  set_initial_pose.py -2535.33 129.72 1511.95 -0.0028 -0.0044 0.9407 0.3391

  # Skip pose estimation and directly set EKF initial pose
  # Option 1: Use command-line argument
  set_initial_pose.py --skip-initial-localization /path/to/initial_pose.yaml
  set_initial_pose.py --skip-initial-localization -2535.33 129.72 1511.95 -0.0028 -0.0044 0.9407 0.3391

  # Option 2: Specify in YAML file (add skip_initial_localization: true)
  # YAML format:
  #   pose:
  #     pose:
  #       position: {x: -2535.33, y: 129.72, z: 1511.95}
  #       orientation: {x: -0.0028, y: -0.0044, z: 0.9407, w: 0.3391}
  #   skip_initial_localization: true
        """
    )
    parser.add_argument(
        '--skip-initial-localization',
        action='store_true',
        help='Skip pose estimation and directly set initial pose to EKF localizer (/initialpose3d). Can also be specified in YAML file as skip_initial_localization: true'
    )
    parser.add_argument(
        '--stamp-unix-sec',
        type=float,
        default=None,
        help='initial pose の header.stamp に使う UNIX 時刻（省略時は get_clock().now()）',
    )
    parser.add_argument(
        'pose_args',
        nargs='+',
        help='Either a YAML file path or 7 pose values (x y z qx qy qz qw)'
    )

    args = parser.parse_args()

    # Check if first argument is a YAML file
    skip_initial_localization = args.skip_initial_localization
    if args.pose_args[0].endswith('.yaml') or args.pose_args[0].endswith('.yml'):
        pose_data = load_from_yaml(args.pose_args[0])
        # Check if skip_initial_localization is specified in YAML file
        # (command-line argument takes precedence)
        if not skip_initial_localization and 'skip_initial_localization' in pose_data:
            skip_initial_localization = bool(pose_data['skip_initial_localization'])
    else:
        # Parse as command-line arguments
        if len(args.pose_args) != 7:
            parser.error("Expected 7 arguments (x y z qx qy qz qw) when not using YAML file")
        pose_data = load_from_args(args.pose_args)

    rclpy.init()
    node = InitialPoseSetter(
        pose_data,
        skip_initial_localization=skip_initial_localization,
        stamp_unix_sec=args.stamp_unix_sec,
    )

    # Keep node alive briefly to ensure operations complete
    import time
    try:
        if skip_initial_localization:
            # Wait a bit to ensure EKF and NDT are activated, then exit
            # (pose_initializer is stopped, so no need to keep running)
            node.get_logger().info('Initial pose set and EKF/NDT activated. Exiting...')
            time.sleep(2.0)  # Brief wait to ensure operations complete
        else:
            # Brief wait for normal operation
            time.sleep(1.0)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}", file=sys.stderr)

    # Exit with success code even if interrupted by Ctrl+C
    return 0


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code if exit_code is not None else 0)
