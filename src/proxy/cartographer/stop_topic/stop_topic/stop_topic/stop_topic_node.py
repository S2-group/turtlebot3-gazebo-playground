import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # Example message types
# from your_package.msg import ConstraintList  # Example message types
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class StopTopicProxy(Node):
    def __init__(self, original_topic):
        super().__init__('stop_topic_proxy')
        self.subscription = self.create_subscription(
            MarkerArray,  # Change message type as needed
            original_topic,
            self.input_callback,
            10  # Set your desired queue size
        )
        self.publisher = None  # Will be set later

    def input_callback(self, msg):
        # Stop publishing the message, effectively "blocking" the topic
        pass  # Do nothing with the received message

    def set_publisher(self, output_topic):
        # Create a publisher on the specified output topic
        self.publisher = self.create_publisher(
            MarkerArray,  # Change message type as needed
            output_topic,
            10  # Set your desired queue size
        )

def main(args=None):
    rclpy.init(args=args)

    original_topic = '/constraint_list'  # Set the original topic to stop
    output_topic = '/constraint_list_stopped'      # Set the output topic for the proxy

    stop_topic_proxy = StopTopicProxy(original_topic)
    stop_topic_proxy.set_publisher(output_topic)

    rclpy.spin(stop_topic_proxy)

    stop_topic_proxy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

