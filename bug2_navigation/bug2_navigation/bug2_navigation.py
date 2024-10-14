import math
from time import sleep
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from bug2_interfaces.srv import Bug2Goto
from tf_transformations import euler_from_quaternion


class Bug2_navigator(Node):
    def __init__(self):
        super().__init__("bug2navigatorcontroller")
        self.wallfollow = self.create_client(SetBool, "wall_follower_service")
        self.gotopoint = self.create_client(Bug2Goto, "go_to_point_service")
        while not self.wallfollow.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.wallfollow_req = SetBool.Request()
        while not self.gotopoint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.gotopoint_req = Bug2Goto.Request()

        self.regions = {
            "fright": 0.0,
            "front": 0.0,
            "fleft": 0.0,
        }
        self.position = Point()
        self.yaw = 0.0
        self.closest = 10.0

        self.target = Point()
        self.target.x = -6.0
        self.target.y = 3.4

        self.mode = 1
        self.mode_dict = {"wallfollow": 0, "gotopoint": 1}

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.clbk_laser, 10
        )
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.clbk_odom, 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sendto_gotopoint(True, self.target)

    def timer_callback(self):
        if self.mode == self.mode_dict["gotopoint"]:
            if self.regions["front"] < 1.0 and (
                self.regions["fright"] < 1.0 or self.regions["fleft"] < 1.0
            ):
                self.get_logger().info("Found a wall, entering wall follower mode")
                self.closest = getDistance(self.position, self.target)
                self.change_mode(self.mode_dict["wallfollow"])
        elif self.mode == self.mode_dict["wallfollow"]:
            dst = getDistance(self.position, self.target)
            if dst < self.closest:
                self.closest = dst

    def change_mode(self, mode: int):
        if mode != self.mode:
            if mode == self.mode_dict["wallfollow"]:
                self.mode = self.mode_dict["gotopoint"]
                self.sendto_wallfollower(True)
                self.sendto_gotopoint(False, self.target)
            elif mode == self.mode_dict["gotopoint"]:
                self.mode = self.mode_dict["wallfollow"]
                self.sendto_gotopoint(True, self.target)
                self.sendto_wallfollower(False)
        else:
            self.get_logger().warning(
                f"Tried to switch to the same mode, or invalid mode: {mode}"
            )

    def clbk_laser(self, msg):
        self.regions = {
            "fright": min(min(msg.ranges[320:339]), 1.0),
            "front": min(min(min(msg.ranges[0:9]), min(msg.ranges[350:359])), 1.0),
            "fleft": min(min(msg.ranges[20:39]), 1.0),
        }

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

    def sendto_wallfollower(self, start_wallfollower: bool) -> SetBool.Response:
        self.get_logger().info(
            f"Sending message to wallfollower controller: start: {start_wallfollower}"
        )
        self.wallfollow_req.data = start_wallfollower
        self.wallfollow_future = self.wallfollow.call_async(self.wallfollow_req)
        rclpy.spin_until_future_complete(self, self.wallfollow_future, timeout_sec=3.0)
        print("exited sendto wallfollow")
        return self.wallfollow_future.result()

    def sendto_gotopoint(
        self, start_gotopoint: bool, target: Point
    ) -> Bug2Goto.Response:
        self.get_logger().info(
            f"Sending message to go-to-point controller: start: {start_gotopoint} - target: {target}"
        )
        self.gotopoint_req.move_switch = start_gotopoint
        self.gotopoint_req.target_position = target
        self.gotopoint_future = self.gotopoint.call_async(self.gotopoint_req)
        rclpy.spin_until_future_complete(self, self.gotopoint_future, timeout_sec=3.0)
        print("exited sendto gotopoint")
        return self.gotopoint_future.result()


def getDistance(start: Point, end: Point) -> float:
    return math.sqrt(pow(end.y - start.y, 2) + pow(end.x - start.x, 2))


def main(args=None):
    rclpy.init(args=args)
    controller = Bug2_navigator()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        sleep(1)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
