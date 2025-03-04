import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from airtable_interface import get_commands

class AirtableCommandPublisher(Node):
    def __init__(self):
        super().__init__('airtable_command_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)  # Fetch commands every 5 seconds

    def timer_callback(self):
        commands = get_commands()
        for command in commands:
            action = command['fields'].get('Action', '')
            parameters = command['fields'].get('Parameters', '')
            msg = String()
            msg.data = f"{action},{parameters}"
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = AirtableCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
