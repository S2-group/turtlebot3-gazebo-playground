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
from lifecycle_msgs.msg import TransitionEvent
import lifecycle_msgs.msg
import nav2_msgs.msg
import bond.msg
import rosgraph_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import diagnostic_msgs.msg


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
        self.subscription_handlers.append(
            self.create_subscription(TransitionEvent, '/amcl/transition_event', lambda msg: self.callback(msg, '/amcl/transition_event'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(TransitionEvent, '/behavior_server/transition_event', lambda msg: self.callback(msg, '/behavior_server/transition_event'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(nav2_msgs.msg.BehaviorTreeLog, '/behavior_tree_log', lambda msg: self.callback(msg, '/behavior_tree_log'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(bond.msg.Status, '/bond', lambda msg: self.callback(msg, '/bond'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(TransitionEvent, '/bt_navigator/transition_event', lambda msg: self.callback(msg, '/bt_navigator/transition_event'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(rosgraph_msgs.msg.Clock, '/clock', lambda msg: self.callback(msg, '/clock'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(geometry_msgs.msg.Twist, '/cmd_vel', lambda msg: self.callback(msg, '/cmd_vel'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(geometry_msgs.msg.Twist, '/cmd_vel_nav', lambda msg: self.callback(msg, '/cmd_vel_nav'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(visualization_msgs.msg.MarkerArray, '/constraint_list', lambda msg: self.callback(msg, '/constraint_list'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(TransitionEvent, '/controller_server/transition_event', lambda msg: self.callback(msg, '/controller_server/transition_event'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(sensor_msgs.msg.PointCloud2, '/cost_cloud', lambda msg: self.callback(msg, '/cost_cloud'), 10)
        )
        self.subscription_handlers.append(
            self.create_subscription(diagnostic_msgs.msg, '/diagnostics', lambda msg: self.callback(msg, '/diagnostics'), 10)
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
        total_count = 0
        total_size = 0
        for topic in self.topic_list:
            count = self.message_counts[topic]
            total_count += count
            size = self.message_sizes[topic]
            size = size / 1000000
            total_size += size
            self.get_logger().info(f"Timestamp: {time.time()}, Topic: {topic}, Count: {count}, Size:  {size} (MB)")
        self.get_logger().info(f"Total Count: {total_count}, Total Size: {total_size} (MB)")


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
