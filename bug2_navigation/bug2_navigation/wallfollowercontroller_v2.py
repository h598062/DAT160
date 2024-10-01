import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


class WallfollowerController(Node):
    def __init__(self):
        super().__init__("wall_follower")

        self.scan = self.create_subscription(
            LaserScan, "/scan", self.clbk_laser, qos_profile_sensor_data
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.position = 0
        self.yaw = 0

        # Front lidars
        self.lidar_front_left = 100
        self.lidar_front_middle = 100
        self.lidar_front_right = 100
        # Left side lidars
        self.lidar_left_f = 100
        self.lidar_left_m = 100
        self.lidar_left_b = 100
        # Right side lidars
        self.lidar_right_f = 100
        self.lidar_right_m = 100
        self.lidar_right_b = 100

        self.mode = 0
        self.tracking_wall = ""
        self.wall_dissappeared = False

        print("Starting mode 0, looking for a wall...")

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_laser(self, msg):
        # Read the front lidars
        self.lidar_front_left = msg.ranges[12]  # 12 deg to the left
        self.lidar_front_middle = msg.ranges[0]  # direct infront
        self.lidar_front_right = msg.ranges[348]  # 12 deg to the right (360 - 12)
        # Read the left lidars
        self.lidar_left_f = msg.ranges[74]  # 16 deg infront of 90
        self.lidar_left_m = msg.ranges[90]  # 90 deg left
        self.lidar_left_b = msg.ranges[106]  # 16 deg behind 90
        # Read the right lidars
        self.lidar_right_f = msg.ranges[286]  # 16 deg infront of 270
        self.lidar_right_m = msg.ranges[270]  # 270 deg right
        self.lidar_right_b = msg.ranges[254]  # 16 deg behind 270

    def timer_callback(self):
        vel_msg = Twist()
        fw_speed = 0.0
        turn_speed = 0.0
        fr = self.lidar_front_right
        fm = self.lidar_front_middle
        fl = self.lidar_front_left
        rf = self.lidar_right_f
        rm = self.lidar_right_m
        rb = self.lidar_right_b
        lf = self.lidar_left_f
        lm = self.lidar_left_m
        lb = self.lidar_left_b

        mode1_dst = 0.6
        mode2_dst = 0.6
        mode3_dst = 0.7
        # print(
        #     f"fl: {fl:.2f} - fm: {fm:.2f} - fr: {fr:.2f} - rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f} - lf: {lf:.2f} - lm: {lm:.2f} - lb: {lb:.2f}"
        # )

        if self.mode == 0:
            # print(
            #     f"fl: {fl:.2f} - fm: {fm:.2f} - fr: {fr:.2f} - rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f} - lf: {lf:.2f} - lm: {lm:.2f} - lb: {lb:.2f}"
            # )
            dst = 0.5
            fw_speed = 0.2
            turn_speed = 0.0
            if fm < dst:
                self.mode = 2
                self.tracking_wall = "right"
                print("Found wall, activated mode 2: Turning")
        if self.mode == 2:
            fw_speed = 0.005
            if self.tracking_wall == "right":
                print(f"fm: {fm:.2f} - rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f}")
                turn_speed = 1.0
                if (
                    fm >= mode2_dst
                    and rm <= rb
                    and rm <= rf
                    and abs(rb - rf) < 0.1
                    and rm < mode2_dst
                ):
                    turn_speed = 0.0
                    fw_speed = 0.0
                    self.mode = 1
                    print("Finished turning, activated mode 1: follow wall")
            elif self.tracking_wall == "left":
                # print(f"lf: {lf:.2f} - lm: {lm:.2f} - lb: {lb:.2f}")
                turn_speed = -1.0
                if (
                    fm >= mode2_dst
                    and lm <= lb
                    and lm <= lf
                    and abs(lb - lf) < 0.1
                    and lm < mode2_dst
                ):
                    turn_speed = 0.0
                    fw_speed = 0.0
                    self.mode = 1
                    print("Finished turning, activated mode 1: follow wall")
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(vel_msg)
                raise Exception(
                    "ERROR: i am in mode 2 (Turning) but i am not tracking a wall"
                )

        if self.mode == 1:
            # print(
            #     f"fl: {fl:.2f} - fm: {fm:.2f} - fr: {fr:.2f} - rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f} - lf: {lf:.2f} - lm: {lm:.2f} - lb: {lb:.2f}"
            # )
            fw_speed = 0.2
            if rm <= rb and rm <= rf:
                turn_speed = 0.0
            else:
                if rf > rm + mode1_dst:
                    fw_speed = 0.05
                    if self.wall_dissappeared:
                        if rm > rb + mode1_dst:
                            print("Wall dissappeared, going mode 3")
                            self.mode = 3
                            fw_speed = 0.0
                    else:
                        print("wall disappeared maybe")
                        self.wall_dissappeared = True
                        turn_speed = 0.0
                else:
                    turn_speed = constrain((rb - rf) * 2, -1.0, 1.0)

            if fm < mode1_dst:
                turn_speed = 0.0
                fw_speed = 0.0
                print("Found new wall, going mode 2")
                self.mode = 2

        if self.mode == 3:
            print(f"fm: {fm:.2f} - rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f}")
            turn_speed = -0.4
            fw_speed = 0.2
            if rf <= mode3_dst:
                turn_speed = -0.1
            if rm <= mode3_dst and rf <= mode3_dst + 0.2 and rb <= mode3_dst + 0.2:
                print("re-found the wall, continuing mode 1")
                fw_speed = 0.0
                self.mode = 1
            elif fm < mode2_dst:
                print("found a wall in front, going mode 2")
                self.mode = 2

        vel_msg.linear.x = fw_speed
        vel_msg.angular.z = turn_speed
        self.cmd_vel_pub.publish(vel_msg)

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


def constrain(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    controller = WallfollowerController()
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
