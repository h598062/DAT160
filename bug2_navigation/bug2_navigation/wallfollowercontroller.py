import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallfollowerController(Node):
    def __init__(self):
        super().__init__("bug2_navigation")

        self.scan = self.create_subscription(LaserScan, "/scan", self.clbk_laser, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

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
        # print(
        #     f"fl: {fl:.2f} - fm: {fm:.2f} - fr: {fr:.2f} - rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f} - lf: {lf:.2f} - lm: {lm:.2f} - lb: {lb:.2f}"
        # )

        if self.mode == 0:
            # print(
            #     f"fl: {fl:.2f} - fm: {fm:.2f} - fr: {fr:.2f} - rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f} - lf: {lf:.2f} - lm: {lm:.2f} - lb: {lb:.2f}"
            # )
            dst = 0.6
            fw_speed = 0.2
            turn_speed = 0.0
            if fm < dst:
                self.mode = 2
                self.tracking_wall = "right"
                print("Found wall, activated mode 2: Turning")
        if self.mode == 2:
            dst = 0.6
            fw_speed = 0.005
            if self.tracking_wall == "right":
                print(f"rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f}")
                turn_speed = 1.0
                if rm <= rb and rm <= rf and abs(rb - rf) < 0.1 and rm < dst:
                    turn_speed = 0.0
                    fw_speed = 0.0
                    self.mode = 1
                    print("Activated mode 1")
            elif self.tracking_wall == "left":
                print(f"lf: {lf:.2f} - lm: {lm:.2f} - lb: {lb:.2f}")
                turn_speed = -1.0
                if lm <= lb and lm <= lf and abs(lb - lf) < 0.1 and lm < dst:
                    turn_speed = 0.0
                    fw_speed = 0.0
                    self.mode = 1
                    print("Activated mode 1")
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(vel_msg)
                raise Exception(
                    "ERROR: i am in mode 2 (Turning) but i am not tracking a wall"
                )

        if self.mode == 1:
            print(
                f"fl: {fl:.2f} - fm: {fm:.2f} - fr: {fr:.2f} - rf: {rf:.2f} - rm: {rm:.2f} - rb: {rb:.2f} - lf: {lf:.2f} - lm: {lm:.2f} - lb: {lb:.2f}"
            )
            dst = 0.5
            fw_speed = 0.2
            if rm <= rb and rm <= rf:
                turn_speed = 0.0
            else:
                if rf > rm + dst:
                    fw_speed = 0.05
                    if self.wall_dissappeared:
                        if rm > rb:
                            print("Wall dissappeared, going mode 3")
                            self.mode = 3
                            turn_speed = 0.0
                            fw_speed = 0.0
                    else:
                        print("wall disappeared maybe")
                else:
                    turn_speed = constrain((rb - rf) * 2, -1.0, 1.0)

            if fm < 0.6:
                turn_speed = 0.0
                fw_speed = 0.0
                print("Found new wall, going mode 2")
                self.mode = 2

        if self.mode == 3:
            print("Mode 3 not implemented")

        vel_msg.linear.x = fw_speed
        vel_msg.angular.z = turn_speed
        self.cmd_vel_pub.publish(vel_msg)


def constrain(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)

    controller = WallfollowerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
