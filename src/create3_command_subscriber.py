import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Create3CommandSubscriber(Node):
    def __init__(self):
        super().__init__('create3_command_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def command_callback(self, msg):
        command_data = msg.data.split(',')
        action = command_data[0].strip()
        parameters = command_data[1].strip() if len(command_data) > 1 else ''
        twist = Twist()

        if action == 'Move Forward':
            twist.linear.x = float(parameters) if parameters else 0.1
        elif action == 'Turn Left':
            twist.angular.z = float(parameters) if parameters else 0.1
        elif action == 'Turn Right':
            twist.angular.z = -float(parameters) if parameters else -0.1
        elif action == 'Stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info(f"Executed command: {action} with parameters: {parameters}")

def main(args=None):
    rclpy.init(args=args)
    node = Create3CommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
