#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace this with your custom message type(s)
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import TransformStamped
import tf2_msgs.msg
from sensor_msgs.msg import LaserScan
import time
from pympler.asizeof import asizeof


class MessageStatsNode(Node):
    def __init__(self):
        super().__init__('message_stats_node')
        self.topic_list = ['/map', '/odom', '/scan', '/tf']  # Add more topic names if needed
        self.message_counts = {topic: 0 for topic in self.topic_list}
        self.message_sizes = {topic: 0 for topic in self.topic_list}

        self.subscription_handlers = []
        self.setup_subscriptions()

    def setup_subscriptions(self):
        self.subscription_handlers.append(
            self.create_subscription(OccupancyGrid, '/map', lambda msg: self.callback(msg, '/map'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(Odometry, '/odom', lambda msg: self.callback(msg, '/odom'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(LaserScan, '/scan', lambda msg: self.callback(msg, '/scan'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(tf2_msgs.msg.TFMessage, '/tf', lambda msg: self.callback(msg, '/tf'), 10)
        )

    def callback(self, msg, topic_name):
        self.message_counts[topic_name] += 1
        self.message_sizes[topic_name] += asizeof(msg)

        # Print timestamp and message data
        # if topic_name == '/tf':
        #     self.get_logger().info(f"Timestamp: {time.time()}, Topic: {topic_name}, Message: {msg.transform.translation}")
        # self.get_logger().info(f"Timestamp: {time.time()}, Topic: {topic_name}")

    def print_stats(self):
        self.get_logger().info("Message statistics:")
        for topic in self.topic_list:
            count = self.message_counts[topic]
            size = self.message_sizes[topic]
            self.get_logger().info(f"Timestamp: {time.time()}, Topic: {topic}, Count: {count}, Total Size: {size} bytes, {size / 1000000} (MB)")


def main(args=None):
    start_time = time.time()
    print("Start time: ", start_time)
    rclpy.init(args=args)
    node = MessageStatsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.print_stats()
    end_time = time.time()
    print("End time: ", end_time)
    print("duration: ", end_time - start_time)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
