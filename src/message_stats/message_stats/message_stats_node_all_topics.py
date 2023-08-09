#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pympler.asizeof import asizeof
import time
# import ros_msg String
from std_msgs.msg import String

def import_msg_type(module, msg_type):
    msg_module = __import__(module, fromlist=[msg_type])
    # import pdb; pdb.set_trace()
    msg_type_class = getattr(msg_module, msg_type)
    return msg_type_class

class MessageStatsNodeAllTopics(Node):
    def __init__(self):
        super().__init__('message_stats_node')
        self.message_counts = {}
        self.message_sizes = {}
        self.setup_subscriptions()
        self.set_publisher('/message_stats')

    def setup_subscriptions(self):
        # Get list of all topics and their message types
        topic_list_output = self._get_command_output(['ros2', 'topic', 'list'])
        topics = topic_list_output.decode().splitlines()
        # print("Topics: ", topics)
        for topic in topics:
            # Extract the message type of the topic
            topic_info_output = self._get_command_output(['ros2', 'topic', 'info', topic])
            topic_info = topic_info_output.decode().splitlines()
            message_type = None
            sub_count = None
            for line in topic_info:
                if line.startswith("Type: "):
                    message_type = line[len("Type: "):]
                if line.startswith("Subscription count: "):
                    sub_count = int(line[len("Subscription count: "):])


            if sub_count and sub_count> 0 and message_type:
                # Check if the message type is in the correct format
                if '/' not in message_type:
                    print(f"Invalid message type format for topic '{topic}'. Skipping...")
                    continue

                # Convert message type to Python import format
                # import pdb; pdb.set_trace()
                # module, msg_type = message_type.split('/')
                # split the message type with the last '/' to handle nested messages
                module, msg_type = message_type.rsplit('/', 1)
                module_import = module.replace('/', '.')
                msg_type_class = import_msg_type(module_import, msg_type)

                # Subscribe to the topic
                self.message_counts[topic] = 0
                self.message_sizes[topic] = 0
                self.create_subscription(msg_type_class, topic, lambda msg, topic=topic: self.callback(msg, topic), 10)
        print("Finished setting up subscriptions, topics: ", self.message_counts.keys())

    def set_publisher(self, output_topic):
        # Create a publisher on the specified output topic
        self.publisher = self.create_publisher(
            String,  # Change message type as needed
            output_topic,
            10  # Set your desired queue size
        )

    def _get_command_output(self, command):
        import subprocess
        try:
            return subprocess.check_output(command)
        except subprocess.CalledProcessError:
            print(f"Failed to run command: {' '.join(command)}")
            return b''

    def callback(self, msg, topic_name):
        self.message_counts[topic_name] += 1
        self.message_sizes[topic_name] += asizeof(msg)

    def print_stats(self):
        # self.get_logger().info("Message statistics:")
        # set get_logger() to print to file
        
        total_count = 0
        total_size = 0
        for topic, count in self.message_counts.items():
            size = self.message_sizes[topic] / 1000000  # Convert bytes to MB
            # format size to 2 decimal places
            size = "{:.2f}".format(size)
            total_count += count
            total_size += size
            print(f"Topic: {topic}, Count: {count}, Size: {size} MB")
        print(f"Total Count: {total_count}, Total Size: {total_size} MB")

def main(args=None):
    rclpy.init(args=args)
    # rclpy.logging.get_logger('rclpy').set_level(rclpy.logging.LoggingSeverity.ERROR)
    node = MessageStatsNodeAllTopics()
    start_time = time.time()
    print("Start time: ", start_time)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.print_stats()
    end_time = time.time()
    print("End time: ", end_time)
    print("Duration: ", end_time - start_time)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
