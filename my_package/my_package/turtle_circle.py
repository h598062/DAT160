#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class myTurtleCircle(Node):
    def __init__(self) -> None:
        super().__init__("turtle_circle")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer = self.create_timer(0.5, self.set_cmd_vel)

    def set_cmd_vel(self):
        msg = Twist()
        msg.angular.z = 1.0
        msg.linear.x = 1.0
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = myTurtleCircle()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

