#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class BlobTracker(Node):
    def __init__(self):
        super().__init__("blob_tracker_node")
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # TODO: Make a subscriber to camera topic '/camera/image_raw' with message type 'Image'
        # and callback function self.image_callback
        self.cam_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        # TODO: Create SimpleBlobDetector_Params object and define filter parameters
        params = cv2.SimpleBlobDetector_Params()

        # Filter by Area.
        params.filterByArea = False
        params.minArea = 15
        params.maxArea = 6000

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # TODO: Create a SimpleBlobDetector object with the parameter object defined above and store it as a property of the class
        self.detector = cv2.SimpleBlobDetector_create(params)

    def image_callback(self, data):
        # This converts the image from ROS Image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # TODO: Implement blob detection on the camera image 'cv_image'.
        # Consider changing color space, thresholding and the filter parameters for the blob detection.
        # Remember that based on your thresholding method, you might have to invert the binary image with the ~ operator before passing it to the detector.
        #
        # Return the result of the blob detection into the variable 'keypoints':
        # keypoints =

        if keypoints:
            # Assuming the biggest blob is the desired one
            # You can change this logic to track a different blob, but choosing the biggest one worked fine for me
            keypoint_to_track = max(keypoints, key=lambda k: k.size)

            # DO NOT MODIFY AFTER THIS LINE
            # Perform visual servoing
            self.visual_servoing(keypoint_to_track, cv_image)

            # Draw detected blobs
            blobs = cv2.drawKeypoints(
                cv_image,
                [keypoint_to_track],
                np.zeros((1, 1)),
                (0, 255, 0),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
            )
        else:
            # No blobs found, stop the robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

            # Only display original image if no blobs found
            blobs = cv_image

        cv2.imshow("Blob Tracker", blobs)
        cv2.waitKey(1)

    def visual_servoing(self, keypoint, cv_image):
        # DO NOT MODIFY THIS FUNCTION
        # Visual servoing logic to control the robot
        x, y = keypoint.pt
        blob_size = keypoint.size
        twist = Twist()
        # Calculate the linear velocity based on the size of the blob
        desired_size = 80  # desired size
        size_error = desired_size - blob_size
        twist.linear.x = 0.015 * size_error  # Proportional control
        twist.linear.x = np.clip(twist.linear.x, -0.75, 0.75)  # Limit the speed
        # Calculate angular velocity to keep the blob centered horizontally
        error_x = x - (cv_image.shape[1] / 2)
        twist.angular.z = -error_x / 650.0  # Proportional control
        # Limit the angular speed
        twist.angular.z = np.clip(twist.angular.z, -1.5, 1.5)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    blob_tracker = BlobTracker()
    rclpy.spin(blob_tracker)
    blob_tracker.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
