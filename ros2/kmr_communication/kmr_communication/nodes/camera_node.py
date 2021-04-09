#!/usr/bin/env python3

import sys
import rclpy
import argparse
from std_msgs.msg import String, Float64
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from ..script.camera_opencv import Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.name = 'camera_node'
        self.declare_parameter('id')
        self.id = self.get_parameter('id').value
        self.camera = Camera()
        self.timer = self.create_timer(1/30, self.callback)
        self.bridge = CvBridge()
        
        # Make a listener for relevant topics
        """ nothing as of now """

        # Publishers
        self.publish_image = self.create_publisher(Image, 'image', 10)

        self.get_logger().info('Node is ready')

    def callback(self):
        frame = self.camera.capture()

        msg = self.bridge.cv2_to_imgmsg(frame)

        self.publish_image.publish(msg)

    def tear_down(self):
        try:
            self.destroy_node()
            rclpy.shutdown()
            print(cl_green("Successfully tore down camera node"))
        except:
            print(cl_red('Error: ') + "rclpy shutdown failed")


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.tear_down()
        
if __name__ == '__main__':
    main()