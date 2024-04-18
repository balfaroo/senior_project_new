import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
from bryan_msgs.msg import BottomCamera

class BottomCameraNode(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('bottom_facing_camera')

        self.detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.publisher = self.create_publisher(BottomCamera, 'bottom_camera', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.cap = cv2.VideoCapture(0)



    def timer_callback(self):
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        msg = BottomCamera()
        msg.dx = 0.0
        msg.dy = 0.0
        if detections:
            msg.found = True
        else:
            msg.found = False
        self.publisher.publish(msg)
        print('published message')
        


    def publish_camera_msg(self, frame):  # assume frame already in grayscale
        detections = self.detector.detect(frame)
        msg = BottomCamera()
        
        

        if detections:
            d = detections[0]
            center = d.center
            msg.dx = 0.0 # for now
            msg.dy = 0.0
            msg.found = True
        else:
            msg.dx = 0.0
            msg.dy = 0.0
            msg.found = False

        self.publisher.publish(msg)
        print('published message')

def main(args = None):
    rclpy.init(args = args)

    # camera = 0
    # cap = cv2.VideoCapture(camera)
    node = BottomCameraNode()
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()
    # while True:
    #     try:
    #         rclpy.spin_once(node)
    #         ret, frame = cap.read()
    #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #         BottomCameraNode.publish_camera_msg(gray)
    #     except Exception:
    #             try:
    #                 BottomCameraNode.destroy_node()
    #             except:
    #                 pass
    #             try:
    #                 rclpy.shutdown()
    #             except:
    #                 pass
            
    #             sys.exit(0)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
            




            