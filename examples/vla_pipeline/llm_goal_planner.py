import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from example_interfaces.action import FollowJointTrajectory # Placeholder for a robot action
import json

# Placeholder for a simple LLM inference function
# In a real application, this would involve calling a large language model API
def call_llm_for_action_plan(text_instruction):
    # Simulate LLM response for a simple instruction
    if "move forward" in text_instruction.lower():
        return {"action": "move", "direction": "forward", "distance": 0.5}
    elif "turn left" in text_instruction.lower():
        return {"action": "turn", "direction": "left", "angle": 90}
    elif "pick up" in text_instruction.lower():
        return {"action": "gripper", "command": "grasp"}
    else:
        return {"action": "unknown", "instruction": text_instruction}

class LLMGoalPlannerNode(Node):

    def __init__(self):
        super().__init__('llm_goal_planner_node')
        self.declare_parameter('whisper_topic', '/whisper_output')
        self.declare_parameter('action_plan_topic', '/robot_action_plan')

        self.whisper_topic = self.get_parameter('whisper_topic').get_parameter_value().string_value
        self.action_plan_topic = self.get_parameter('action_plan_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            String,
            self.whisper_topic,
            self.whisper_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, self.action_plan_topic, 10)

        self.get_logger().info(f"LLM Goal Planner Node initialized, subscribing to {self.whisper_topic}, publishing to {self.action_plan_topic}")

    def whisper_callback(self, msg):
        text_instruction = msg.data
        self.get_logger().info(f"Received text instruction from Whisper: \"{text_instruction}\"")

        # Call LLM to generate action plan
        action_plan = call_llm_for_action_plan(text_instruction)
        self.get_logger().info(f"Generated action plan: {action_plan}")

        # Publish action plan (as a JSON string for simplicity)
        action_plan_msg = String()
        action_plan_msg.data = json.dumps(action_plan)
        self.publisher_.publish(action_plan_msg)

def main(args=None):
    rclpy.init(args=args)

    llm_goal_planner_node = LLMGoalPlannerNode()

    rclpy.spin(llm_goal_planner_node)

    llm_goal_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
