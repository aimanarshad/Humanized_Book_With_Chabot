
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleController(Node):

    def __init__(self):
        super().__init__('simple_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        # Simple example: move forward for 5 seconds, then stop
        if self.i < 10: # 10 * 0.5s = 5 seconds
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.get_logger().info('Moving forward')
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Stopping')
            # For a real robot, you might want to stop publishing after a while
            # or wait for new commands.
            # For this simple example, we'll loop movement for demonstration.
            if self.i > 20: # Reset after 10 seconds of stopping
                self.i = 0

        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    simple_controller = SimpleController()

    rclpy.spin(simple_controller)

    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
