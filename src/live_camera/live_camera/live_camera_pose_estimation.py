import os
import cv2
import time
import numpy as np
from tqdm import tqdm
from cycler import cycle
from sympy import homogeneous_order

from matplotlib import pyplot as plt

import pytransform3d.camera as pc
import pytransform3d.rotations as pr
import pytransform3d.trajectories as ptr
import pytransform3d.transformations as pt

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

class VO(Node):
    def __init__(self):
        super().__init__("vo_node")
        self.subscription_ = self.create_subscription(Image, "camera", 
                                self.camera_callback, qos_profile_sensor_data)
        self.br = CvBridge()

    def camera_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Camera", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    vo = VO()
    try:
        rclpy.spin(vo)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
