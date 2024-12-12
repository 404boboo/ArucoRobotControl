#!/usr/bin/env python3

import rclpy  # ROS 2 Python Client Library
from rclpy.node import Node  # Base class for ROS nodes
from sensor_msgs.msg import Image  # Image message type
from geometry_msgs.msg import Twist  # Twist message for robot control
from cv_bridge import CvBridge  # Convert between ROS Image and OpenCV
import cv2  # OpenCV library
import cv2.aruco as aruco  # ArUco marker detection


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Subscription to the image_raw topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Camera node started and subscribing to 'image_raw'.")

        # ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        # Define mapping from marker IDs to commands
        self.id_to_command = {
            14: self.create_twist_command(0.5, 0.0),  # Move forward
            20: self.create_twist_command(0.0, 0.5),  # Rotate in place
            30: self.create_twist_command(-0.5, 0.0), # Move backward
            40: self.create_twist_command(0.0, -0.5), # Rotate in the opposite direction
        }

    def create_twist_command(self, linear_x, angular_z):
        """
        Helper function to create a Twist message.
        """
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        return cmd

    def listener_callback(self, image_data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")

            # Convert the image to grayscale for marker detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            # If markers are detected
            if ids is not None:
                # Draw detected markers on the image
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                self.get_logger().info(f"Detected marker IDs: {ids.flatten().tolist()}")

                # Check detected IDs against the mapping
                for marker_id in ids.flatten():
                    if marker_id in self.id_to_command:
                        command = self.id_to_command[marker_id]
                        self.cmd_publisher.publish(command)
                        self.get_logger().info(f"Published command for ID {marker_id}")
                    else:
                        self.get_logger().warning(f"No command mapped for ID {marker_id}")

            # Display the image with detected markers
            cv2.imshow("ArUco Marker Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Create and run the CameraNode
    camera_node = CameraNode()

    try:
        # Spin the node
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the node
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
