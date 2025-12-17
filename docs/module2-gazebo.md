---
sidebar_position: 2
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Overview of Digital Twins and Simulation-First Robotics

Digital twins are virtual models designed to accurately reflect a physical object, process, or system. In robotics, a digital twin allows developers to design, test, and refine robot behaviors in a simulated environment before deploying them to physical hardware. This simulation-first approach significantly reduces development time, cost, and risks associated with real-world testing.

### Benefits:
- **Cost-Effective:** Reduces the need for expensive physical prototypes.
- **Faster Iteration:** Allows rapid testing and modification of designs.
- **Safety:** Enables testing in hazardous scenarios without risk to humans or equipment.
- **Reproducibility:** Ensures consistent testing environments for reliable results.

## SDF vs URDF Explanation

**URDF (Unified Robot Description Format):**
- XML format for describing the kinematic and dynamic properties of a robot.
- Primarily used for defining the robot's visual and collision properties, joints, and links.
- Ideal for single-robot descriptions in ROS (Robot Operating System).
- Lacks support for environmental elements (e.g., world, sensors not attached to the robot, light sources).

**SDF (Simulation Description Format):**
- XML format used by Gazebo for describing robots, environments, and other objects in a simulation.
- More comprehensive than URDF, supporting complex environments, dynamic objects, and various sensor types.
- Can define gravity, light, physics properties, and multiple robots within a single world.
- Gazebo internally converts URDF to SDF for simulation, but using native SDF for complex worlds is often preferred.

## Build Minimal Gazebo World

A Gazebo world is defined in an `.sdf` file. Here's a minimal example of a world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="minimal_world">
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0 0.5 0 1</ambient>
            <diffuse>0 0.5 0 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Add Depth/LiDAR/IMU Sensors

Gazebo allows for easy integration of various sensor types through plugins. These sensors publish data to ROS 2 topics, which can then be consumed by other ROS 2 nodes.

### Example: Depth Camera Sensor Plugin

To add a depth camera, you typically include a `sensor` tag within a `link` of your robot's SDF.

```xml
<link name="camera_link">
  <visual name="visual">
    <geometry>
      <box>
        <size>0.05 0.05 0.05</size>
      </box>
    </geometry>
  </visual>
  <collision name="collision">
    <geometry>
      <box>
        <size>0.05 0.05 0.05</size>
      </box>
    </geometry>
  </collision>
  <sensor name="depth_camera" type="depth">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <namespace>depth_camera</namespace>
        <argument>--ros-args -r __ns:=/camera</argument>
        <argument>--ros-args -r image:=image_raw</argument>
        <argument>--ros-args -r depth:=depth_raw</argument>
        <argument>--ros-args -r camer-info:=camer-info</argument>
      </ros>
      <cameraName>depth_camera</cameraName>
      <frameName>camera_depth_frame</frameName>
    </plugin>
  </sensor>
</link>
```

### Other Sensors:
- **LiDAR (Ray Sensor):** Uses `ray` type with `libgazebo_ros_laser.so` plugin.
- **IMU (Inertial Measurement Unit):** Uses `imu` type with `libgazebo_ros_imu_sensor.so` plugin.

## Unity for High-Fidelity Rendering (HRI)

While Gazebo excels in physics simulation and ROS 2 integration, Unity can be used for high-fidelity Human-Robot Interaction (HRI) visuals and advanced rendering.

**Integration Approaches:**
1.  **ROS#-Unity Bridge:** A package that allows Unity to communicate with ROS 2 systems, subscribing to sensor data from Gazebo (via ROS 2) and publishing control commands.
2.  **Separate Visualization:** Use Unity solely for rendering a visually appealing environment and robot model, taking pose and other state information from Gazebo.
3.  **Hybrid Simulation:** In some advanced setups, Unity can even handle certain physics simulations (e.g., soft body dynamics) while Gazebo handles others.

**Key Considerations:**
-   **Data Synchronization:** Efficiently transfer data (e.g., joint states, sensor readings, object poses) between Gazebo (via ROS 2) and Unity.
-   **Coordinate Systems:** Be mindful of differences in coordinate systems (e.g., ROS/Gazebo typically Z-up, Unity typically Y-up).
-   **Performance:** High-fidelity rendering can be computationally intensive, requiring optimization.

## Sensor Data Collection Example (ROS 2 Python)

Once sensors are configured in Gazebo to publish to ROS 2 topics, you can create a simple ROS 2 Python node to subscribe to and log this data.

```python
# examples/module2_gazebo/sensor_listener.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu # Example message types

class SensorListener(Node):

    def __init__(self):
        super().__init__('sensor_listener')
        self.depth_image_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw', # Replace with your actual depth topic
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
```

## Launch Instructions

To launch a Gazebo world with a robot and sensors, you typically use a shell script or a ROS 2 launch file.

### Using a Shell Script (`launch_sim.sh`)

```bash
#!/bin/bash
# examples/module2_gazebo/launch_sim.sh

# Source ROS 2 environment
source /opt/ros/humble/setup.bash # Replace humble with your ROS 2 distribution

# Export Gazebo model path if you have custom models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models

# Launch Gazebo with your world file
gazebo --verbose examples/module2_gazebo/world.sdf -s libgazebo_ros_factory.so -s libgazebo_ros_init.so
```
Make sure to make the script executable: `chmod +x launch_sim.sh`

### Running the Sensor Listener Node

In a separate terminal, after launching Gazebo:

```bash
# Source ROS 2 environment (if not already sourced in this terminal)
source /opt/ros/humble/setup.bash

# Run your sensor listener node
ros2 run examples_module2_gazebo sensor_listener
```
*(Note: You'll need to create a `setup.py` and `package.xml` for the `examples_module2_gazebo` package for `ros2 run` to work correctly. For a quick test, you can run the Python script directly.)*

## Minimal Humanoid URDF

While SDF is used for the world, a robot's description can often start as a URDF for simplicity and then be converted to SDF for Gazebo. Here's a very minimal URDF for a humanoid:

```xml
<?xml version="1.0"?>
<robot name="minimal_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
    </collision>
  </link>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>
</robot>
```

## Smoke Test Command and Assertion Description

**Gazebo Launch Smoke Test:**

**Command:**
```bash
bash examples/module2_gazebo/launch_sim.sh
```

**Assertion:**
-   Gazebo launches successfully, displaying the `minimal_world` with a green ground plane.
-   No critical errors are reported in the terminal output of `launch_sim.sh`.
-   The simulated robot (if added to `world.sdf`) appears in the Gazebo environment.

**Sensor Listener Smoke Test:**

**Command (in a new terminal after Gazebo launch):**
```bash
ros2 run examples_module2_gazebo sensor_listener
```

**Assertion:**
-   The `sensor_listener` node starts without errors.
-   Messages confirming "Received depth image data" are logged, indicating the node is successfully subscribing to the camera topic and receiving data. (Requires a depth camera sensor to be active in the Gazebo simulation and publishing to `/camera/depth/image_raw`).
