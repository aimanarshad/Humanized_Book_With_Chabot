import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String # For simplified perception results
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
from torchvision import transforms
import numpy as np
import argparse
import os

# Assuming SimpleVisionModel is defined in train_vision_model.py or a shared utility
# For this example, we'll re-define a simple version for self-containment.
# In a real project, you'd import it.
class SimpleVisionModel(nn.Module):
    def __init__(self, num_classes=2):
        super(SimpleVisionModel, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=3, padding=1)
        self.relu1 = nn.ReLU()
        self.pool1 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=3, padding=1)
        self.relu2 = nn.ReLU()
        self.pool2 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.flatten = nn.Flatten()
        # This linear layer size must match the actual output of the conv/pool layers
        # For this skeleton, we assume a typical image size like 640x480 which becomes 160x120 after pooling
        self.fc = nn.Linear(32 * (640 // 4) * (480 // 4), num_classes) # Adjust based on actual input size

    def forward(self, x):
        x = self.pool1(self.relu1(self.conv1(x)))
        x = self.pool2(self.relu2(self.conv2(x)))
        x = self.flatten(x)
        x = self.fc(x)
        return x


class InferenceNode(Node):

    def __init__(self):
        super().__init__('inference_node')
        self.declare_parameter('model_path', 'trained_model.pth')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('perception_topic', '/perception_results')
        self.declare_parameter('num_classes', 2)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.perception_topic = self.get_parameter('perception_topic').get_parameter_value().string_value
        self.num_classes = self.get_parameter('num_classes').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.get_logger().info(f"Loading model from: {self.model_path}")
        self.model = SimpleVisionModel(num_classes=self.num_classes).to(self.device)
        if os.path.exists(self.model_path):
            self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
            self.model.eval()
            self.get_logger().info("Model loaded successfully.")
        else:
            self.get_logger().warn("Model not found at specified path. Initializing with random weights.")

        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((480, 640)), # Must match training input size
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, self.perception_topic, 10)
        self.get_logger().info(f"Inference node initialized, subscribing to {self.image_topic}, publishing to {self.perception_topic}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Preprocess image
        input_tensor = self.transform(cv_image).unsqueeze(0).to(self.device)

        # Run inference
        with torch.no_grad():
            outputs = self.model(input_tensor)
            _, predicted = torch.max(outputs.data, 1)

        # Post-process results (simplified for this example)
        perception_result = f"Detected class: {predicted.item()}"
        self.get_logger().info(f"Publishing: {perception_result}")

        # Publish perception result
        msg = String()
        msg.data = perception_result
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    inference_node = InferenceNode()

    rclpy.spin(inference_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
