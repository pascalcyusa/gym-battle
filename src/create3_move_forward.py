# Script to move the iRobot Create3 in a straight line
# Author: Jean Pascal Cyusa Shyaka

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class WalkRoutePublisher(Node):
    def __init__(self):
        super().__init__("walk_route_publisher")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(
            0.5, self.publish_velocity
        )  # Publish every 0.5 sec

    def publish_velocity(self):
        msg = Twist()

        msg.linear.x = 0.5  # Move forward at 0.5 m/s

        msg.angular.z = 0.0  # No turning

        self.publisher.publish(msg)

        self.get_logger().info("🚶‍♂️ Moving Forward!")


def main(args=None):
    rclpy.init(args=args)

    node = WalkRoutePublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
