#!/usr/bin/env python3

from os import error, uname
import sys
from typing import Callable
import rclpy
import argparse
from std_msgs.msg import String, Float64
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import subprocess

if uname()[4] == "aarch64":
    import picamera
    import socket
    import time

def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'


class CameraNode(Node):
    def __init__(self, robot):
        super().__init__('camera_node')
        self.name = 'camera_node'
        self.robot = robot
        self.status = 0
        self.declare_parameter('id')
        self.id = self.get_parameter('id').value
        self.declare_parameter('udp/ip')
        self.ip = self.get_parameter('udp/ip').value
        self.proc = None
        self.isRPI = uname()[4] == "aarch64"

        if self.isRPI:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscribers
        sub_camera = self.create_subscription(String, 'handle_camera_' + str(self.id), self.handle_camera, 10)
        sub_status_check = self.create_subscription(String, 'status_check', self.publish_status, 10)

        # Publishers
        self.camera_status_publisher = self.create_publisher(String, 'camera_status', 10)

    def handle_camera(self, data):
        if data.data.lower() == "start" and self.status == 0:
            print(cl_green("Starting camera"))
            #self.proc = subprocess.Popen(["/bin/bash", "kmr_communication/kmr_communication/script/startcamera.sh", self.ip])
            if self.isRPI:
                server, port = self.ip.split(":")
                print("I AM RASPBERRY PI")
                print(server, port)

                while True:
                    try:
                        self.client_socket.connect((server, int(port)))
                        print("IM IN THE LOOP")
                        break
                    except socket.error:
                        print("Connection Failed, Retrying..")
                        time.sleep(1)

                print("CONNECTED!!!!!!!!")
                self.connection = self.client_socket.makefile('wb')
                with picamera.PiCamera() as camera:
                    camera.resolution = (640, 480)
                    camera.framerate = 24
                    camera.start_recording(self.connection , format='h264')
                    print("CAMERA STARTED!!!!!!!!")
            self.status = 1
        elif data.data.lower() == "stop":
            try:
                self.status = 0
                #self.proc.terminate()
                if self.isRPI:
                    with picamera.PiCamera() as camera:
                        camera.stop_recording()
                        self.connection.close()
                        self.client_socket.close()
                print(cl_green("Stopping camera"))  
            except AttributeError:
                print(cl_red("Camera was never started, therefore never stopped")) 

        self.publish_status()


    def publish_status(self):
        msg = String()
        msg.data = self.id + ":" + self.robot + ":camera:" + str(self.status) + ":" + str(self.ip) #ip = ip:port
        self.camera_status_publisher.publish(msg)
    
    def tear_down(self):
        try:
            self.destroy_node()
            rclpy.shutdown()
            print(cl_green("Successfully tore down camera node"))
        except:
            print(cl_red('Error: ') + "rclpy shutdown failed")


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    while True:
        rclpy.init(args=argv)
        camera_node = CameraNode(args.robot)
        rclpy.spin(camera_node)
        
if __name__ == '__main__':
    main()