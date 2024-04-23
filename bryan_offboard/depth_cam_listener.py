import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
from bryan_msgs.msg import DepthCamera
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class Listener(Node):

    def __init__(self):
        super().__init__('depth_camera_listener')

        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
        )

        self.subscription = self.create_subscription(DepthCamera, 'depth_camera', self.listener_callback, 10)
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position=vehicle_local_position

    def body_to_local(self, x, y):
        hding = self.vehicle_local_position.heading # + self.yaw_inst maybe don't need to add yaw since you haven't rotated yet
        return np.cos(hding)*x-np.sin(hding)*y, np.sin(hding)*x+np.cos(hding)*y

    def listener_callback(self, msg):
        if msg.spotted:
            # print('estimated forward distance: ', msg.dx)
            # print('estimated lateral distance: ', msg.dy)
            # print('estimated yaw: ', msg.yaw)
            # print()]
            dx = msg.dx/1000
            dy = msg.dy/1000
            yaw = msg.yaw
            print('yaw: ', yaw + np.radians(msg.yaw), 'dx: ', dx, 'dy: ', dy)
            dx, dy = self.body_to_local(0.2, -np.sign(dy)*min(0.2,abs(dy))) # dy needs to be small, for now will run it so that it is
            # print('would send dx, ', dx, ' would send dy, ', dy, ' and would yaw by', yaw)

            

def main(args=None):
    rclpy.init(args=args)

    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()