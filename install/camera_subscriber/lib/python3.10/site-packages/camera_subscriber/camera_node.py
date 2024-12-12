#!/usr/bin/env python3

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library


def listener_callback(image_data):
    # Convert ROS Image message to OpenCV image
    cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
    # Display image
    cv2.imshow("camera", cv_image)
    # Stop to show the image
    cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    # Create the node
    node = Node('camera_node')
    # Log information into the console
    node.get_logger().info('Hello node')
    # Create the subscriber. This subscriber will receive an Image
    # from the image_raw topic. The queue size is 10 messages.
    subscription = node.create_subscription(Image,'image_raw',listener_callback,10)
    # Spin the node so the callback function is called.
    rclpy.spin(node)
    # Spin the node so the callback function is called.
    rclpy.shutdown()

if __name__ == '__main__':
    main()