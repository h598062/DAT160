from time import sleep
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

        self.lidar = []
        self.angle_increment = 0.0

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
            self.angle_increment = msg.angle_increment

        self.lidar = msg.ranges

    def timer_callback(self):
        msg = Twist()
        fw_speed = 0.0
        turn_speed = 0.0
        # make a copy so it doesnt change during calculation
        lidar = np.copy(self.lidar)

        front_dist, left_dist, right_dist = self.get_wall_distances(lidar)

        # Decide on the direction and speed
        fw_speed, turn_speed = self.decide_direction(front_dist, left_dist, right_dist)

        self.get_logger().info(
            f"fw_speed: {fw_speed} - turn_speed: {turn_speed} - fd: {front_dist:.2f} - ld: {left_dist:.2f} - rd: {right_dist:.2f}"
        )
        msg.linear.x = fw_speed
        msg.angular.z = turn_speed
        self.cmd_vel_pub.publish(msg)

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

    def get_wall_distances(self, lasers):
        # Get mean distance for each region to detect walls
        front_distances = np.append(lasers[330:361], lasers[0:31])
        left_distances = lasers[30:91]
        right_distances = lasers[270:331]

        # Calculate the average distance in each region (you can also use min distance)
        front_mean = np.mean(front_distances)
        left_mean = np.mean(left_distances)
        right_mean = np.mean(right_distances)

        return front_mean, left_mean, right_mean

    def decide_direction(
        self,
        front_dist: float,
        left_dist: float,
        right_dist: float,
        distance_threshold: float = 0.5,
    ):
        fw_speed = 0.2  # default forward speed
        turn_speed = 0.0  # default no turning

        if front_dist < distance_threshold:
            # Wall in front, decide based on left and right distances
            if left_dist > right_dist:
                turn_speed = 0.9  # Turn left
            else:
                turn_speed = -0.9  # Turn right
            fw_speed = 0.0  # Stop moving forward when turning
        else:
            # No wall in front, move forward and adjust for side walls
            if left_dist < distance_threshold:
                # Too close to the left wall, turn slightly right
                turn_speed = -0.5
            elif right_dist < distance_threshold:
                # Too close to the right wall, turn slightly left
                turn_speed = 0.5
            else:
                # No nearby walls, continue straight
                turn_speed = 0.0

        return fw_speed, turn_speed


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
    controller = WallfollowerController_v2()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        controller.cmd_vel_pub.publish(msg)
        sleep(1)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
