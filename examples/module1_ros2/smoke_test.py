
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SmokeTestSubscriber(Node):

    def __init__(self):
        super().__init__('smoke_test_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.received_message = False

    def listener_callback(self, msg):
        self.get_logger().info('Smoke Test - I heard: "%s"' % msg.data)
        self.received_message = True

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Node('minimal_publisher_for_smoke_test') # Temporary publisher for smoke test
    publisher = minimal_publisher.create_publisher(String, 'topic', 10)

    smoke_test_subscriber = SmokeTestSubscriber()

    print("Starting ROS2 smoke test...")

    # Give some time for nodes to set up
    time.sleep(2)

    msg = String()
    msg.data = 'Smoke Test Message'
    publisher.publish(msg)
    minimal_publisher.get_logger().info('Publishing smoke test message: "%s"' % msg.data)

    # Spin for a short period to allow message to be received
    rclpy.spin_once(smoke_test_subscriber, timeout_sec=3)

    if smoke_test_subscriber.received_message:
        print("Smoke test PASSED: Message successfully received.")
    else:
        print("Smoke test FAILED: No message received.")

    # Clean up
    minimal_publisher.destroy_node()
    smoke_test_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
