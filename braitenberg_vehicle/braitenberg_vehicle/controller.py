import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# TODO: Import the message types that you need for your publishers and subscribers here:

class BraitenbergController(Node):
    def __init__(self):
        # TODO: Initialze a ros node
        super().__init__("braitenberg_vehicle")
        # TODO: Create a subscriber to the /scan topic using as a callback
        # function the existing function self.clbk_laser and queue size of 10
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.clbk_laser, 10)

        # TODO: Create a publisher to the /cmd_vel topic with a queue size of 10
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Default values for the lidar variables as a placeholder until the
        # actual sensor values are recieved through from the ros topic
        self.lidar_left_front = 100
        self.lidar_right_front = 100

        # Creates a timer definition -> during rclpy.spin() the function
        # self.timer_callback() will be executed every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Callback function for the Turtlebots Lidar topic /scan
    def clbk_laser(self, msg):
        self.lidar_left_front = msg.ranges[12]
        self.lidar_right_front = msg.ranges[348]

    def timer_callback(self):
        # Creates a message from type Twist
        vel_msg = Twist()
        # Defines the speed at which the robot will move forward (value between 0 and 1)
        vel_msg.linear.x = 0.7
        # Defines the speed at which the robot will turn around its own axis (value between -1 and 1)
        vel_msg.angular.z = 0.0

        # print(f"left: {self.lidar_left_front}  -  right: {self.lidar_right_front}")

        # TODO: Set vel_msg.linear.x and vel_msg.angular.z depending on the
        # values from self.lidar_left_front and self.lidar_right_front
        # print("lidar left front: ", self.lidar_left_front)

        avstand = 1.5
        left = self.lidar_left_front
        right = self.lidar_right_front
        if left < avstand and right < avstand and (right-0.5 < left < right+0.5) and (left-0.5 < right < left+0.5):
            # Stopp
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        else:
            # legg inn en maks grense for utregning av sving
            if left > avstand:
                left = avstand
            if right > avstand:
                right = avstand
            # endre sving vinkel til å være differansen mellom sensorene * 2
            vel_msg.angular.z = (left - right) * 2.0
            
        # TODO: Publish vel_msg using the previously defined publisher
        self.cmd_vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = BraitenbergController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
