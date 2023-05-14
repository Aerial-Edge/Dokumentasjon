#!/usr/bin/etv python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
from cv_bridge import CvBridge
import cvzone
from cvzone.FPS import FPS
from cvzone.ColorModule import ColorFinder
import numpy as np
import time
import math

class ObjectDetection(Node):
    """
    A class to detect objects in an image and publish their locations as ROS messages.

    This class is a Node in ROS that subscribes to image data, processes it to find objects,
    and publishes the locations of these objects.
    """

    def __init__(self):
        """
        Initializes ObjectDetection with a subscriber, bridge, FPS reader, and publisher.
        """
        super().__init__('object_detection')
        self.subscription = self.create_subscription(Image, 'image_data', self.process_image, 10)
        self.bridge = CvBridge()
        self.fpsreader = FPS()
        self.distance_and_position_publisher = self.create_publisher(Int32MultiArray, 'distance_and_pos', 10)

    def is_circle(self, cnt, threshold=0.7):
        """
        Determines if a contour is a circle.

        Args:
            cnt: The contour to evaluate.
            threshold: The circularity threshold. Defaults to 0.7.

        Returns:
            bool: True if the contour is a circle, False otherwise.
        """
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            return False
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        return circularity >= threshold

    def process_image(self, msg):
        """
        Processes image data and publishes object locations.

        This function converts the ROS image message to an OpenCV format,
        finds objects in the image, and publishes their locations.

        Args:
            msg: The ROS image message to process.
        """
        start_time = time.time()
        opencv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        

        h, w, _ = opencv_image.shape

        myColorFinder: ColorFinder = ColorFinder(False)
        hsvValsBlue = {'hmin': 104, 'smin': 128, 'vmin': 0, 'hmax': 120, 'smax': 255, 'vmax': 152} #blue
        hsvValsGreen = {'hmin': 75, 'smin': 34, 'vmin': 22, 'hmax': 103, 'smax': 175, 'vmax': 255} #green
        hsvValsRed = {'hmin': 0, 'smin': 120, 'vmin': 120, 'hmax': 20, 'smax': 255, 'vmax': 255} #red

        fps, img = self.fpsreader.update(opencv_image)

        _, maskBlue = myColorFinder.update(img, hsvValsBlue)
        _, maskGreen = myColorFinder.update(img, hsvValsGreen)
        _, maskRed = myColorFinder.update(img, hsvValsRed)


        _, contoursBlue = cvzone.findContours(img, maskBlue)
        _, contoursGreen = cvzone.findContours(img, maskGreen)
        _, contoursRed = cvzone.findContours(img, maskRed)

        # Filter contours that are circles
        circular_contours_blue = [cnt for cnt in contoursBlue if self.is_circle(cnt['cnt'])]
        circular_contours_green = [cnt for cnt in contoursGreen if self.is_circle(cnt['cnt'])]
        circular_contours_red = [cnt for cnt in contoursRed if self.is_circle(cnt['cnt'])]

        # Process and display depth, x, and y position for each ball
        for color, circular_contours_list in zip(['blue', 'green', 'red'],
                                                [circular_contours_blue, circular_contours_green, circular_contours_red]):
            if circular_contours_list:

                cnt = circular_contours_list[0]
                #data = cnt['center'][0], h - cnt['center'][1], int(cnt['area'])
                x, y = cnt['center']

                f = 889
                W = 6.5
                w = cnt['bbox'][3]
                d = (W * f) / w
                self.get_logger().info(f"{color}: {d}")
                self.get_logger().info(f" fps: {fps}")
                self.get_logger().info(f" x: {x} y: {y}")

                self.publish_dist_and_pos(x , y, d)
        
        computaiton_time = time.time() - start_time
        self.get_logger().info("Time to compute: " + str(computaiton_time))

    def publish_dist_and_pos(self, x, y, distance):
        """
        Publishes the distance and position of an object.

        This function creates and publishes a ROS message containing the x, y position and distance of an object.

        Args:
            x: The x position of the object.
            y: The y position of the object.
            distance: The distance to the object.
        """
        msg = Int32MultiArray()
        msg.data = [int(x), int(y), int(distance)]
        self.distance_and_position_publisher.publish(msg)

def main(args=None):
    """
    Main function which initializes the ROS client library, creates an ObjectDetection node, and spins.

    Args:
        args: Arguments passed to rclpy.init. Defaults to None.
    """
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()