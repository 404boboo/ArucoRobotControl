#!/usr/bin/env python3

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # ROS2 package to convert between ROS and OpenCV Images
import cv2  # Python OpenCV library
import cv2.aruco as aruco  # ArUco library in OpenCV



class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Subscription to the image_raw topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',  # Camera image topic
            self.listener_callback,
            10
        )
        self.get_logger().info("Camera node started and subscribing to 'image_raw'.")

        # ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

    def listener_callback(self, image_data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")

            # Convert the image to grayscale for marker detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            # If markers are detected, draw them on the image
            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                self.get_logger().info(f"Detected marker IDs: {ids.flatten().tolist()}")

            # Display the image with detected markers
            cv2.imshow("ArUco Marker Detection", cv_image)

            # Add a delay to refresh the image
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    # Create the camera node
    camera_node = CameraNode()

    # Spin the node to process callbacks
    rclpy.spin(camera_node)

    # Shutdown the node
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()