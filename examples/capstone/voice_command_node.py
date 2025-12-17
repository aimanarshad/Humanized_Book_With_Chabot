import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher_ = self.create_publisher(String, '/voice_command', 10)
        self.get_logger().info('Voice Command Node initialized. Ready to publish commands.')
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: "{command}"')

    def get_user_input(self):
        self.get_logger().info("Enter voice commands (e.g., 'move forward', 'turn left', 'pick up'):")
        while rclpy.ok():
            try:
                command = input("")
                if command.lower() == 'exit':
                    break
                self.publish_command(command)
            except EOFError:
                self.get_logger().info("EOF received, exiting input thread.")
                break
            except Exception as e:
                self.get_logger().error(f"Error in input thread: {e}")
                time.sleep(1) # Prevent busy-waiting on errors

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
