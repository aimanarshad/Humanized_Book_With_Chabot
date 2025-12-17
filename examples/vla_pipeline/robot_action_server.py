import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String

from example_interfaces.action import FollowJointTrajectory # Replace with your specific robot action


class RobotActionServer(Node):

    def __init__(self):
        super().__init__('robot_action_server')
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
            'robot_arm_follow_joint_trajectory', # Replace with the actual action name
            self._execute_callback)

        self.get_logger().info(f"Robot Action Server initialized, subscribing to {self.action_plan_topic}, providing action 'robot_arm_follow_joint_trajectory'")

    def action_plan_callback(self, msg):
        action_plan_str = msg.data
        self.get_logger().info(f"Received action plan from LLM Planner: {action_plan_str}")

        # In a real scenario, parse the action_plan_str (e.g., JSON) into a structured plan
        # and then send it as a goal to the action server.
        # For this skeleton, we'll simulate processing and directly execute a dummy action.

        # Example: Triggering a dummy action goal based on the received plan
        # This part would typically involve creating a goal message for FollowJointTrajectory
        # and sending it to self._action_server.
        # For now, we'll just log that a plan was received.
        self.get_logger().info("Simulating execution of received action plan.")

        # Create a dummy goal for the action server (replace with actual goal creation)
        # goal_msg = FollowJointTrajectory.Goal()
        # # Populate goal_msg based on action_plan_str
        # self._action_server.publish_feedback(goal_msg) # This is not how feedback is published, just an example


    def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # --- TODO: Implement actual robot control logic here ---
        # Based on the goal received, control the robot.
        # This would involve sending commands to robot hardware interfaces.

        # Simulate action execution
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.actual.positions = [0.1, 0.2, 0.3] # Example feedback
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.actual.positions))

        # Simulate work being done
        import time
        time.sleep(2) # Simulate robot movement time

        # Check if goal is done (e.g., reached target position)
        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = 0 # Example result
        self.get_logger().info('Returning result: {0}'.format(result.error_code))
        return result

def main(args=None):
    rclpy.init(args=args)

    robot_action_server = RobotActionServer()

    rclpy.spin(robot_action_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
