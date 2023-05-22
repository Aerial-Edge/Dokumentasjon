#!/usr/bin/etv python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from picamera2 import Picamera2
from libcamera import controls

class CameraCapture(Node):
    """
    A class to capture frames from a camera and publish them as ROS messages.

    This class is a Node in ROS that uses the Picamera2 library for capturing video data,
    and CvBridge for converting between ROS and OpenCV image formats.
    """

    def __init__(self):
        """
        Initializes CameraCapture with a publisher, timer, video capture, and bridge.
        """
        super().__init__('camera_capture')
        self.publisher_ = self.create_publisher(Image, 'image_data', 10)
        self.timer = self.create_timer(1/30, self.publish_image_data)
        self.opencv_video = Picamera2()
        self.opencv_video.configure(self.opencv_video.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
        self.opencv_video.set_controls({"AwbEnable": True})
        self.opencv_video.start()
        self.bridge = CvBridge()

    def publish_image_data(self):
        """
        Publishes image data.

        Captures a frame from the video capture device, converts the image to a ROS message, and publishes the message.
        """
        frame = self.opencv_video.capture_array()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    """
    Main function which initializes the ROS client library, creates a CameraCapture node, and spins.

    Args:
        args: Arguments passed to rclpy.init. Defaults to None.
    """
    rclpy.init(args=args)
    node = CameraCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()