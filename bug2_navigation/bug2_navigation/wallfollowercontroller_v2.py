import rclpy
import math
import numpy as np
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


class WallfollowerController_v2(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.get_logger().set_level(LoggingSeverity.INFO)

        self.scan = self.create_subscription(
            LaserScan, "/scan", self.clbk_laser, qos_profile_sensor_data
        )
        self.odometer = self.create_subscription(
            Odometry, "/odom", self.clbk_odom, qos_profile_sensor_data
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = None
        # Do not enable timer until we have recieved a message
        self.first_message = True

        self.position = 0
        self.yaw = 0

        # Modes: drive, turn, stop
        self.mode = "drive"

        # if a desired direction cannot be calculated, use this
        self.default_direction = "left"

        self.laser = []
        self.laser_increment = 0.0

        self.get_logger().info("Started the wallfollower controller")

    def set_timer(self):
        if self.timer is None:
            self.get_logger().info("Got first laserscan message, starting...")
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
        else:
            self.get_logger().warning(
                "Attempted to set the timer when it already exists"
            )

    def clbk_laser(self, msg: LaserScan):
        if self.first_message:
            self.first_message = False
            self.set_timer()
            self.laser_increment = math.degrees(msg.angle_increment)
        self.laser = fix_laser_array(msg.ranges)
        self.get_logger().debug(
            f"Laser Scan: {msg.ranges}\nFixed laser array: {self.laser}"
        )

    def timer_callback(self):
        msg = Twist()
        fw_speed = 0.0
        turn_speed = 0.0

        drive_min_dst = 0.3
        turn_min_dst = 0.5

        # formatted_list = [f"{x:.2f}" for x in self.laser[80:101:1]]
        # self.get_logger().info(f"Front 40: {formatted_list}")

        # når en vegg kommer for nærme foran (30 grader, 90+-15)
        # Finn nærmeste vinkel som er lengre unna enn en viss avstand
        # bruk laser_increment til å finne ønsket vinkel (fra current yaw)
        # roter til vi matcher denne vinkel
        # fortsett kjøring
        # hvis vinkel ikke finnes, stopp

        if self.mode == "drive":
            fw_speed = 0.5
            fw_dst_avg = float(np.average(self.laser[85:96]))
            # self.get_logger().info(f"fw_dst_avg: {fw_dst_avg}")
            fw_dst_avg = constrain(fw_dst_avg, 0.0, 2.0)
            # self.get_logger().info(f"fw_dst_avg: {fw_dst_avg}")
            if fw_dst_avg >= drive_min_dst:
                fw_speed = translate(fw_dst_avg, drive_min_dst, 2.0, 0.05, fw_speed)
                # self.get_logger().info(f"fw_speed: {fw_speed}")
            else:
                fw_speed = 0.0
                self.changeMode("turn")

        self.get_logger().info(f"fw_speed: {fw_speed} - turn_speed: {turn_speed}")
        msg.linear.x = fw_speed
        msg.angular.z = turn_speed
        self.cmd_vel_pub.publish(msg)

    def changeMode(self, mode: str):
        if mode == "drive":
            self.mode = mode
        elif mode == "turn":
            self.mode = mode
        elif mode == "stop":
            self.mode = mode
        else:
            self.get_logger().error(f"Got invalid mode: {mode}")
            return
        self.get_logger().info(f"Switching to mode: {mode}")

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


def fix_laser_array(array):
    """
    expects an array with 360 elements.
      0-180 == left side
    360-180 == right side
    extracts values from 90->0->270
    this will then be the front half circle of the bot
    new list starts at 0 on the rightmost part, and ends at 180 on leftmost part
    90 is straight forwards
    """
    newlist = array[270:360]
    newlist.extend(array[0:91])
    return newlist


def main(args=None):
    rclpy.init(args=args)
    controller = WallfollowerController_v2()
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
