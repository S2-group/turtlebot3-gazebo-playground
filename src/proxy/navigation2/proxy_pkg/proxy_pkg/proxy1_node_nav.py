import rclpy
from rclpy.node import Node
from custom_msg.msg import OdometryReducePose
from nav_msgs.msg import Odometry

class Proxy1Node(Node):
    def __init__(self):
        super().__init__('proxy1')
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.odom_reduce_publisher = self.create_publisher(
            OdometryReducePose, 'odom_reduce_nav', 10)

    def odom_callback(self, msg):
        reduced_msg = OdometryReducePose()
        # reduced_msg = Odometry()
        # Populate reduced_msg based on msg, without the pose data
        # ...
        reduced_msg.header = msg.header
        reduced_msg.child_frame_id = msg.child_frame_id
        reduced_msg.twist = msg.twist
        # reduced_msg.pose = msg.pose

        self.odom_reduce_publisher.publish(reduced_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Proxy1Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
