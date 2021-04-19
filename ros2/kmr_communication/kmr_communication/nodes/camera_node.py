#!/usr/bin/env python3

from os import error
import sys
import rclpy
import argparse
from std_msgs.msg import String, Float64
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import subprocess

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
        self.proc = None

        sub_camera = self.create_subscription(String, 'handle_camera', self.handle_camera, 10)

    def handle_camera(self, data):
        if data.data.lower() == "start":
            print(cl_green("Starting camera"))
            self.proc = subprocess.Popen(["/bin/bash", "kmr_communication/kmr_communication/script/startcamera.sh", self.ip])

        elif data.data.lower() == "stop":
            try:
                self.proc.terminate()
                print(cl_green("Stopping camera"))  
            except AttributeError:
                print(cl_red("Camera was never started, therefore never stopped"))  
    
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