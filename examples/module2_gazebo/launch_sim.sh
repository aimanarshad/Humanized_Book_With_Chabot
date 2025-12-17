#!/bin/bash
# examples/module2_gazebo/launch_sim.sh

# Source ROS 2 environment
source /opt/ros/humble/setup.bash # Replace humble with your ROS 2 distribution

# Export Gazebo model path if you have custom models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models

# Launch Gazebo with your world file and robot model
gazebo --verbose examples/module2_gazebo/world.sdf -s libgazebo_ros_factory.so -s libgazebo_ros_init.so & \
ros2 run gazebo_ros spawn_entity.py -entity minimal_humanoid -file examples/module2_gazebo/robot.sdf -x 0 -y 0 -z 0.5