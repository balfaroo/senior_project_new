'''implementation of a listener node for messages from the bottom camera that allows for testing'''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
from bryan_msgs.msg import BottomCamera
from px4_msgs.msg import VehicleLocalPosition

class Listner(Node):

    def __init__(self):
        super().__init__('bottom_facing_camera_listener')

        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
        )

        self.subscription = self.create_subscription(BottomCamera, 'bottom_camera', self.listener_callback, 10)
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

    def listener_callback(self, msg):
        ''' call back function for bottom camera '''
        if msg.found:
            dx,dy = self.get_april_horiz_distance(msg.cx, msg.cy)
            print(dx, dy)

    def get_april_horiz_distance(self, cx, cy):
        ''' function to get object distance, implemented with a fully downward-facing camera '''
        h_cam = 0.083
        l_cam = 0.084
        h_fov = 41
        v_fov = 66

        ncols = 720
        nrows = 1280


        v_ang_perpx = v_fov/nrows
        h_ang_perpx =  h_fov/ncols
        
        h_of = 0.158

        z = abs(self.vehicle_local_position.z)
        z_leg = z - h_of
        z_cam = z_leg+h_cam

        alpha_h = (cx-ncols/2)*h_ang_perpx
        alpha_v = -(cy-nrows/2)*v_ang_perpx
        print(z)
        print(cx, cy)
        print('alpha h ', alpha_h)
        print('alpha v ', alpha_v)
        
        if z_leg: # for now just checking dx and dy. If determining increments is desired, this can be done by changing
            # the if statement fo if z_leg < <Threshold>

            dx = z_cam*np.tan(np.radians(alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
            dy = z_cam*np.tan(np.radians(alpha_h))
            
        else: # clipping to only go down by 10 cm increments

            z_leg = 0.1
            z_cam = z_leg+h_cam

            dx = z_cam*np.tan(np.radians(alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
            dy = z_cam*np.tan(np.radians(alpha_h))-l_cam

        return dx, dy

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

def main(args=None):
    rclpy.init(args=args)

    listener = Listner()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
