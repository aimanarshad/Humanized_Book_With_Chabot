import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String

# Replace with your actual robot action definition (e.g., from custom_interfaces/action/RobotMove.action)
# For this example, we'll use a dummy action type and simulate its execution.
from example_interfaces.action import FollowJointTrajectory # Placeholder for a robot action
import json
import time

class RobotActionServerNode(Node):

    def __init__(self):
        super().__init__('robot_action_server_node')
        self.declare_parameter('action_plan_topic', '/robot_action_plan')

        self.action_plan_topic = self.get_parameter('action_plan_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            String,
            self.action_plan_topic,
            self.action_plan_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize an Action Server (replace FollowJointTrajectory with your actual action type)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory, # Replace with the actual ROS2 action type for your robot
            'robot_action_execution', # Replace with the actual action name
            self._execute_callback)

        self.get_logger().info(f"Robot Action Server Node initialized, subscribing to {self.action_plan_topic}, providing action 'robot_action_execution'")

    def action_plan_callback(self, msg):
        action_plan_str = msg.data
        self.get_logger().info(f"Received action plan from LLM Planner: {action_plan_str}")

        try:
            action_plan = json.loads(action_plan_str)
            # In a real scenario, you would create a proper ROS2 Action Goal
            # based on the parsed action_plan and send it to the action server.
            # For this skeleton, we will simulate dispatching the goal.

            # Dummy goal creation (replace with actual goal for FollowJointTrajectory or custom action)
            goal_msg = FollowJointTrajectory.Goal() # Or your custom action type
            # Populate goal_msg based on action_plan. For example:
            # if action_plan["action"] == "move":
            #     goal_msg.trajectory.points.append(JointTrajectoryPoint(positions=[action_plan["distance"], 0.0, 0.0]))

            self.get_logger().info(f"Simulating dispatching goal for action: {action_plan['action']}")
            # In a full action client/server setup, you'd have an action client here
            # and send the goal. For this integrated demo, we'll just log.
            # For now, let's directly call the execute callback to simulate it.
            # NOTE: This is a simplification; typically the ActionClient sends a goal to the ActionServer.
            # For this integrated demo, we'll just log the intent and let the 'execute_callback' simulate.
            self.get_logger().info(f"Action plan \'{action_plan}\' would trigger a robot action.")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse action plan JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing action plan: {e}")


    def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing action server goal...')

        # --- TODO: Implement actual robot control logic here ---
        # Based on the goal received, control the robot.
        # This would involve sending commands to robot hardware interfaces.

        # Simulate action execution with feedback
        feedback_msg = FollowJointTrajectory.Feedback() # Or your custom action Feedback type
        feedback_msg.actual.positions = [0.1, 0.2, 0.3] # Example feedback
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.actual.positions))

        # Simulate work being done
        time.sleep(3) # Simulate robot movement time

        # Check if goal is done (e.g., reached target position or completed grasp)
        goal_handle.succeed()

        result = FollowJointTrajectory.Result() # Or your custom action Result type
        result.error_code = 0 # Example result: 0 for success
        self.get_logger().info('Returning result: {0}'.format(result.error_code))
        return result

def main(args=None):
    rclpy.init(args=args)

    robot_action_server_node = RobotActionServerNode()

    rclpy.spin(robot_action_server_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
