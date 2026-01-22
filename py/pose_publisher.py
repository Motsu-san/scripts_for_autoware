import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'visualization_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'  # 必要に応じてframe_idを変更
        msg.pose.position.x = 38895.447894861456
        msg.pose.position.y = 27731.505803787728
        msg.pose.position.z = 164.06027221679688
        msg.pose.orientation.x = -0.016512730833171115
        msg.pose.orientation.y = -0.00133482169809349
        msg.pose.orientation.z = -0.12212173865762065
        msg.pose.orientation.w = 0.9923768583147762
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
