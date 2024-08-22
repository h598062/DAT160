#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class mySubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("my_subscriber")
        self.sub = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
        print("Created")

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = mySubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
