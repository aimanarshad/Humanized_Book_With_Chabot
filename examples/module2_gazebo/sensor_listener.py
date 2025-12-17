import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class SensorListener(Node):

    def __init__(self):
        super().__init__('sensor_listener')
        self.depth_image_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10)
        self.depth_image_subscription  # prevent unused variable warning
        self.get_logger().info('Sensor Listener Node has been started.')

    def depth_image_callback(self, msg):
        self.get_logger().info(f'Received depth image data: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        # Process image data here, e.g., save to file, analyze
        # For simplicity, we just log the timestamp
        pass

def main(args=None):
    rclpy.init(args=args)
    sensor_listener = SensorListener()
    rclpy.spin(sensor_listener)
    sensor_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()