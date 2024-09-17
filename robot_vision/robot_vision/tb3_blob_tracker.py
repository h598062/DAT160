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
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 30000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.8

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.6
        params.maxConvexity = 1.0

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.5

        params.filterByColor = True
        params.blobColor = 255

        # TODO: Create a SimpleBlobDetector object with the parameter object defined above and store it as a property of the class
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.mousex = 0
        self.mousey = 0

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
        lab_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
        # hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # cv2.imshow("LAB image", lab_img)
        # cv2.imshow("HSV image", hsv_img)

        threshold_mask = cv2.inRange(lab_img, np.array([0,150,140]), np.array([255,255,255]))
        cv2.imshow("Threshold mask", threshold_mask)

        result_image = cv2.bitwise_and(cv_image, cv_image, mask=threshold_mask)
        cv2.imshow("Result image", result_image)


        # Function to display LAB values on mouse movement
        def update_mouse_pos(event, x, y, flags, param):
            if event == cv2.EVENT_MOUSEMOVE:  # Check if the mouse is moving
                self.mousex = x
                self.mousey = y
        def create_text_img(lab_image):
            temp_image = lab_image.copy()
            l_value, a_value, b_value = lab_image[self.mousey, self.mousex]
            # Prepare the text to display LAB values
            text = f"L: {l_value}, A: {a_value}, B: {b_value}"
    
            # Draw the text on the image at the bottom-left corner
            cv2.putText(temp_image, text, (10, temp_image.shape[0] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
            # Show the updated image with LAB values
            cv2.imshow("Image with LAB values", temp_image)
        cv2.namedWindow("Image with LAB values")
        cv2.setMouseCallback("Image with LAB values", update_mouse_pos)
        create_text_img(lab_img)


        cv2.waitKey(1)


        keypoints = self.detector.detect(threshold_mask)

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
