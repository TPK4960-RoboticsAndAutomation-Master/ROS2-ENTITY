#!/usr/bin/env python3

import sys
import rclpy
import argparse
from std_msgs.msg import String, Float64
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import os

def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.name = 'camera_node'
        self.declare_parameter('id')
        self.id = self.get_parameter('id').value
        self.declare_parameter('udp/ip')
        self.ip = self.get_parameter('udp/ip').value

        sub_start_camera = self.create_subscription(String, 'start_camera', self.start_camera, 10)

    def start_camera(self, data):
        print("Starting camera")
        os.system("bash kmr_communication/kmr_communication/script/startcamera.sh " + self.ip)
    
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