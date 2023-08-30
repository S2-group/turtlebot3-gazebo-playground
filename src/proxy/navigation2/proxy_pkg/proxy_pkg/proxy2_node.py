import rclpy
from rclpy.node import Node
from custom_msg.msg import OdometryReduce
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance

class Proxy2Node(Node):
    def __init__(self):
        super().__init__('proxy2')
        self.odom_reduce_subscriber = self.create_subscription(
            OdometryReduce, 'odom_reduce', self.odom_reduce_callback, 10)
        self.odom_new_publisher = self.create_publisher(
            Odometry, 'odom_new', 10)

    def odom_reduce_callback(self, msg):
        new_msg = Odometry()
        # Populate new_msg based on msg and add any additional data needed
        # ...
        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id
        new_msg.pose = msg.pose
        # initial new_msg.pose with PoseWithCovariance()
        new_msg.twist = TwistWithCovariance()

        self.odom_new_publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Proxy2Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
