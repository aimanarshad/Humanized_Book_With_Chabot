import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# Placeholder for a simple LLM inference function (similar to vla_pipeline/llm_goal_planner.py)
def call_llm_for_action_plan(text_instruction, perceived_objects):
    # Simulate LLM response for a simple instruction combined with perception
    self.get_logger().info(f"LLM input: instruction='{text_instruction}', objects='{perceived_objects}'")

    if "move forward" in text_instruction.lower():
        return {"action": "move", "direction": "forward", "distance": 0.5}
    elif "turn left" in text_instruction.lower():
        return {"action": "turn", "direction": "left", "angle": 90}
    elif "pick up" in text_instruction.lower() and "red ball" in perceived_objects.lower():
        return {"action": "gripper", "command": "grasp", "target": "red ball"}
    elif "go to" in text_instruction.lower() and "red ball" in perceived_objects.lower():
         return {"action": "move_to_object", "target": "red ball", "location": "({center_x}, {center_y})"} # Placeholder for actual coordinates
    else:
        return {"action": "unknown", "instruction": text_instruction, "perception": perceived_objects}

class LLMPlannerNode(Node):

    def __init__(self):
        super().__init__('llm_planner_node')
        self.declare_parameter('voice_command_topic', '/voice_command')
        self.declare_parameter('perception_output_topic', '/perception_output')
        self.declare_parameter('action_plan_topic', '/robot_action_plan')

        self.voice_command_topic = self.get_parameter('voice_command_topic').get_parameter_value().string_value
        self.perception_output_topic = self.get_parameter('perception_output_topic').get_parameter_value().string_value
        self.action_plan_topic = self.get_parameter('action_plan_topic').get_parameter_value().string_value

        self.latest_voice_command = ""
        self.latest_perception_output = ""

        self.voice_command_subscription = self.create_subscription(
            String,
            self.voice_command_topic,
            self.voice_command_callback,
            10)
        self.perception_subscription = self.create_subscription(
            String,
            self.perception_output_topic,
            self.perception_callback,
            10)
        self.publisher_ = self.create_publisher(String, self.action_plan_topic, 10)

        self.get_logger().info(f"LLM Planner Node initialized, subscribing to {self.voice_command_topic} and {self.perception_output_topic}, publishing to {self.action_plan_topic}")

    def voice_command_callback(self, msg):
        self.latest_voice_command = msg.data
        self.get_logger().info(f"Received voice command: \"{self.latest_voice_command}\"")
        self.generate_and_publish_plan() # Trigger plan generation on new command

    def perception_callback(self, msg):
        self.latest_perception_output = msg.data
        self.get_logger().info(f"Received perception output: \"{self.latest_perception_output}\"")
        # Optional: Trigger plan generation if perception changes are critical, or wait for new voice command
        # self.generate_and_publish_plan()

    def generate_and_publish_plan(self):
        if not self.latest_voice_command:
            self.get_logger().warn("No voice command received yet, skipping plan generation.")
            return

        self.get_logger().info("Generating action plan based on latest command and perception...")
        action_plan = call_llm_for_action_plan(self.latest_voice_command, self.latest_perception_output)
        self.get_logger().info(f"Generated action plan: {action_plan}")

        action_plan_msg = String()
        action_plan_msg.data = json.dumps(action_plan)
        self.publisher_.publish(action_plan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
