
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SensorLoggerNode(Node):

    def __init__(self):
        super().__init__('sensor_logger_node')
        self.subscription = self.create_subscription(
            Image,
            '/humanoid/depth_camera/depth/image_raw',
            self.depth_image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info('Sensor Logger Node started, subscribing to /humanoid/depth_camera/depth/image_raw')

    def depth_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Log some information about the depth image
            self.get_logger().info(f'Received depth image: timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, ' +
                                   f'width={cv_image.shape[1]}, height={cv_image.shape[0]}, ' +
                                   f'min_depth={cv_image.min():.2f}, max_depth={cv_image.max():.2f}')
            # Optionally, save the image or process it further
            # cv2.imwrite(f'depth_image_{msg.header.stamp.sec}.png', cv_image)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

def main(args=None):
    rclpy.init(args=args)

    sensor_logger_node = SensorLoggerNode()

    rclpy.spin(sensor_logger_node)

    sensor_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
