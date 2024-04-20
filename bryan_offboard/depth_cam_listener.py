import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
from bryan_msgs.msg import DepthCamera

class Listner(Node):

    def __init__(self):
        super().__init__('depth_camera_listener')

        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
        )

        self.subscription = self.create_subscription(DepthCamera, 'depth_camera', self.listener_callback, 10)

    def listener_callback(self, msg):
        print(msg.spotted)
        print('estimated forward distance: ', msg.dx)
        print('estimated lateral distance: ', msg.dy)
        print()

def main(args=None):
    rclpy.init(args=args)

    listener = Listner()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()