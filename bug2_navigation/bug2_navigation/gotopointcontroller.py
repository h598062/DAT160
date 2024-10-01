import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class GoToPointController(Node):
    def __init__(self):
        super().__init__("go_to_point")

        self.scan = self.create_subscription(Odometry, "/odom", self.clbk_odom, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.position = 0
        self.yaw = 0

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def timer_callback(self):
        vel_msg = Twist()
        fw_speed = 0.0
        turn_speed = 0.0

        vel_msg.linear.x = fw_speed
        vel_msg.angular.z = turn_speed
        self.cmd_vel_pub.publish(vel_msg)


def constrain(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    controller = GoToPointController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        controller.cmd_vel_pub.publish(msg)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
