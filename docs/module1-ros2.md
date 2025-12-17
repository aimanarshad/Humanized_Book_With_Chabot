---
title: Module 1 — The Robotic Nervous System (ROS 2)
---

# Module 1 — The Robotic Nervous System (ROS 2)

## Module Overview

This module introduces the Robot Operating System 2 (ROS 2), a flexible framework for writing robot software. You will learn the fundamental architectural concepts of ROS 2 and gain practical experience in controlling humanoid robots using `rclpy`, the Python client library for ROS 2.

## Learning Objectives

Upon successful completion of this module, you will be able to:

1.  **Understand ROS 2 Core Concepts**: Define and explain ROS 2 nodes, topics, services, actions, and the Unified Robot Description Format (URDF).
2.  **Implement Basic ROS 2 Communication**: Create `rclpy` publisher and subscriber nodes to facilitate inter-node communication.
3.  **Develop Simple Robot Controllers**: Write `rclpy`-based control logic for a basic humanoid robot and load its URDF model.
4.  **Verify ROS 2 Setup**: Utilize smoke tests within a Docker environment to ensure proper ROS 2 installation and functionality.

## Theory: Nodes, Topics, Services, Actions, URDF

### Nodes

A **node** is an executable process that performs computation. In ROS 2, the system is designed as a distributed network of many independent nodes, each responsible for a specific task (e.g., controlling a motor, processing sensor data, planning a movement).

### Topics

**Topics** are the primary mechanism for asynchronous, many-to-many, publish-subscribe communication in ROS 2. Nodes publish messages to topics, and other nodes can subscribe to these topics to receive the messages. This is ideal for continuous data streams like sensor readings or joint states.

### Services

**Services** provide a synchronous request/reply communication mechanism between nodes. A client node sends a request to a service server, which processes the request and sends back a response. Services are suitable for operations that involve a single request and a single response, such as querying a sensor for a specific value or triggering a one-time action.

### Actions

**Actions** are a higher-level, asynchronous communication method used for long-running, goal-oriented tasks. They extend services by providing feedback during execution and allowing for preemption (canceling a goal). Actions are composed of a goal, feedback, and a result, making them suitable for complex robot behaviors like navigating to a location or manipulating an object.

### URDF (Unified Robot Description Format)

**URDF** is an XML file format used in ROS to describe all elements of a robot. This includes its kinematic and dynamic properties, visual appearance, and collision properties. URDF files are essential for visualizing robots in simulation tools like RViz and Gazebo, and for path planning and inverse kinematics calculations.

## Example: Talker.py and Listener.py (ROS 2 Python)

This example demonstrates basic ROS 2 topic communication using `rclpy`.

### `talker.py` (Publisher Node)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### `listener.py` (Subscriber Node)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Instructions

To run the talker and listener example, you would typically use `ros2 run` commands in separate terminals, or a ROS 2 launch file.

1.  **Open two terminals.**
2.  **In the first terminal, run the publisher:**
    ```bash
    ros2 run module1_ros2 talker
    ```
3.  **In the second terminal, run the subscriber:**
    ```bash
    ros2 run module1_ros2 listener
    ```

*(Note: Ensure your ROS 2 environment is sourced before running these commands.)*

## Minimal Humanoid URDF

This simple URDF describes a basic humanoid robot with a base link and a single arm. (The full `minimal_humanoid.urdf` file will be provided in `examples/module1_ros2/urdf/`.)

```xml
<?xml version="1.0"?>
<robot name="minimal_humanoid">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.35"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

</robot>
```

## Smoke Test Command and Assertion Description

To verify the basic functionality of your ROS 2 setup and the example nodes, a smoke test can be implemented. This test will launch a minimal ROS 2 node and assert that messages are being published on a specific topic.

**Command to run smoke test (conceptual):**

```bash
# This command would typically be part of a larger test script or CI pipeline
ros2 launch module1_ros2 minimal_launch.py # Example launch file
# ... then assert topic presence/content using ros2 topic echo or a dedicated test node
```

**Assertion Description:**

The smoke test asserts that `ros2 topic echo /topic` shows messages being published by `talker.py` and that `listener.py` successfully receives and logs these messages, indicating correct ROS 2 inter-node communication.
