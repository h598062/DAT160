import rclpy
import numpy as np
import math
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class GoToPointController(Node):
    def __init__(self):
        super().__init__("go_to_point")
        self.get_logger().set_level(LoggingSeverity.INFO)

        self.scan = self.create_subscription(Odometry, "/odom", self.clbk_odom, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.position = None
        self.yaw = 0.0  # 0.0 is to the bots right in the simulation. yaw is in radians

        self.target = [-4.0, 1.0]  # set a target to the left in the simulation
        self.targetYaw = 0.0

        self.timer = None
        # Do not enable timer until we have recieved a message
        self.first_message = True

    def set_timer(self):
        if self.timer is None:
            self.get_logger().info("Got first odometer message, starting...")
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
        else:
            self.get_logger().warning(
                "Attempted to set the timer when it already exists"
            )

    def clbk_odom(self, msg: Odometry):
        if self.first_message:
            self.first_message = False
            self.set_timer()
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        # self.getTargetYaw()

    def timer_callback(self):
        vel_msg = Twist()
        fw_speed = 0.0
        turn_speed = 0.0

        dx = self.target[0] - self.position.x
        dy = self.target[1] - self.position.y
        targetVec = np.array([dx, dy])
        targetAngle = math.atan2(dy, dx)
        angleBetween = targetAngle - self.yaw
        # normalize angle between -pi and pi
        angleBetweenNormalized = math.atan2(
            math.sin(angleBetween), math.cos(angleBetween)
        )

        turn_speed = translate(angleBetweenNormalized, -math.pi, math.pi, -1.0, 1.0)
        if turn_speed <= 0.1:
            dst = math.sqrt((dx * dx) + (dy * dy))
            fw_speed = translate(dst, 0.0, 10.0, 0.2, 1.0)

        self.get_logger().info(
            f"fw_speed: {fw_speed} - turn_speed: {turn_speed} - pos: (x: {self.position.x:.3f} y: {self.position.y:.3f} z: {self.position.z:.3f} - yaw: {self.yaw:.3f} - tv: [{targetVec[0]:.2f}, {targetVec[1]:.2f}] - ta: {targetAngle:.2f} - dan: {angleBetweenNormalized:.2f}"
        )

        if self.reachedTarget():
            self.get_logger().info("Reached target!!!")
            fw_speed = 0.0
            turn_speed = 0.0

        vel_msg.linear.x = fw_speed
        vel_msg.angular.z = turn_speed
        self.cmd_vel_pub.publish(vel_msg)

    def reachedTarget(self):
        x = self.position.x
        y = self.position.y
        tx = self.target[0]
        ty = self.target[1]

        return abs(tx - x) <= 0.2 and abs(ty - y) <= 0.2

    def getTargetYaw(self):  # not in use, calculated in time func instead
        dx = self.target[0] - self.position.x
        dy = self.target[1] - self.position.y
        targetVec = np.array([dx, dy])
        targetAngle = math.atan2(dy, dx)
        angleBetween = targetAngle - self.yaw
        # normalize angle between -pi and pi
        angleBetweenNormalized = math.atan2(
            math.sin(angleBetween), math.cos(angleBetween)
        )
        self.targetYaw = targetAngle

        self.get_logger().info(
            f"targetVec: [{targetVec[0]:.2f}, {targetVec[1]:.2f}] - targetAngle: {targetAngle:.2f} - angleBetweenNormalized: {angleBetweenNormalized:.2f}"
        )


def translate(
    value: float, min: float, max: float, result_min: float, result_max: float
) -> float:
    """
    Translates a value from a min/max range into another min/max range
    """
    orig_span = max - min
    result_span = result_max - result_min
    val_scaled = (value - min) / orig_span
    return result_min + (val_scaled * result_span)


def constrain(
    value,
    min_val,
    max_val,
):
    """
    constrains a value between min_val and max_val
    the arguments must work with max() and min() functions
    """
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
