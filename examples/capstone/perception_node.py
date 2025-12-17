import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')
        self.declare_parameter('image_topic', '/humanoid/depth_camera/image_raw') # Or other camera topic
        self.declare_parameter('perception_output_topic', '/perception_output')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.perception_output_topic = self.get_parameter('perception_output_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(String, self.perception_output_topic, 10)
        self.get_logger().info(f"Perception Node initialized, subscribing to {self.image_topic}, publishing to {self.perception_output_topic}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # --- TODO: Implement actual perception logic here ---
        # This is a placeholder. In a real scenario, you'd use computer vision techniques
        # to detect objects, estimate poses, etc.
        # For example, a simple color-based detection for a "red ball"
        perception_result = ""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range for red color in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Assuming the largest contour is our target object
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2

            # Simple depth estimation (if depth image available and calibrated)
            # This requires a depth image subscription, which is not handled by this node currently
            # For a true Capstone, combine with depth_image_callback from Module 2 example
            depth_info = "" # Placeholder for actual depth

            perception_result = f"Detected red ball at (approx. px): ({center_x}, {center_y}) {depth_info}"
        else:
            perception_result = "No red ball detected."

        self.get_logger().info(f"Publishing perception: {perception_result}")

        msg = String()
        msg.data = perception_result
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
