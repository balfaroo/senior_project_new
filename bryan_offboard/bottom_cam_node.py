import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
from bryan_msgs.msg import BottomCamera
from px4_msgs.msg import VehicleLocalPosition

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

        self.msg = BottomCamera()
        self.publisher = self.create_publisher(BottomCamera, 'bottom_camera', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        # self.cap = cv2.VideoCapture(0)

        # self.vehicle_local_position_subscriber = self.create_subscription(
        #     VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        # self.vehicle_local_position = VehicleLocalPosition()


    def update_msg(self, msg):
        self.msg = msg

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def timer_callback(self):
        # ret, frame = self.cap.read()
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        # detections = self.detector.detect(gray)
        # #cv2.imshow('frame',frame)
        # msg = BottomCamera()
        # msg.cx = -1
        # msg.cy = -1
        # if detections:
        #     msg.found = True
        #     msg.cx = int(detections[0].center[0])
        #     msg.cy =int(detections[0].center[1])
        #     #print('found apriltag')
        #     # _, _ = self.get_april_horiz_distance(msg.cx, msg.cy)
        # else:
        #     msg.found = False
        self.publisher.publish(self.msg)
        #print('published message')
        

    # def get_april_horiz_distance(self, cx, cy):
    #     h_cam = 0.063
    #     l_cam = 0.063
    #     h_fov = 41
    #     v_fov = 66

    #     ncols = 720
    #     nrows = 1280

    #     # v_ang_perpx = frame.shape[0]/v_fov
    #     # h_ang_perpx = frame.shape[1]/h_fov

    #     v_ang_perpx = v_fov/nrows
    #     h_ang_perpx =  h_fov/ncols
        
    #     h_of = 0.158

    #     z = abs(self.vehicle_local_position.z)
    #     z_leg = z - h_of
    #     z_cam = z_leg+h_cam

    #     alpha_h = (cx-ncols/2)*h_ang_perpx
    #     alpha_v = -(cy-nrows/2)*v_ang_perpx

    #     print('alpha h ', alpha_h)
    #     print('alpha v ', alpha_v)
        
    #     #if z_leg: # for now just checking dx and dy

    #     dx = z_cam*np.tan(np.radians(45+alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
    #     dy = z_cam*np.tan(np.radians(45+alpha_h))-l_cam
        
    #     # else: # clipping to only go down by 10 cm increments

    #     #     z_leg = 0.1
    #     #     z_cam = z_leg+h_cam

    #     #     dx = z_cam*np.tan(np.radians(45)+alpha_v)-l_cam
    #     #     dy = z_cam*np.tan(np.radians(45)+alpha_h)-l_cam

    #         # do the calculations as if we were only 10 cm in the air

    #     print('dx ', dx, ' dy ', dy)
            

    #     return dx, dy


    # def publish_camera_msg(self, frame):  # assume frame already in grayscale
    #     detections = self.detector.detect(frame)
    #     msg = BottomCamera()
        
        

    #     if detections:
    #         d = detections[0]
    #         center = d.center
    #         msg.cx = int(center[0]) # for now
    #         msg.cy = int(center[1])
    #         msg.found = True
    #         _, _ = self.get_april_horiz_distance(msg.cx, msg.cy)
    #     else:
    #         msg.cx = -1
    #         msg.cy = -1
    #         msg.found = False

    #     self.publisher.publish(msg)
    #     print('published message')

def get_april_horiz_distance(node, cx, cy):
    h_cam = 0.063
    l_cam = 0.063
    h_fov = 41
    v_fov = 66

    ncols = 720
    nrows = 1280

    # v_ang_perpx = frame.shape[0]/v_fov
    # h_ang_perpx = frame.shape[1]/h_fov

    v_ang_perpx = v_fov/nrows
    h_ang_perpx =  h_fov/ncols
    
    h_of = 0.158

    z = abs(node.vehicle_local_position.z)
    z_leg = z - h_of
    z_cam = z_leg+h_cam

    alpha_h = (cx-ncols/2)*h_ang_perpx
    alpha_v = -(cy-nrows/2)*v_ang_perpx

    print('alpha h ', alpha_h)
    print('alpha v ', alpha_v)
    
    #if z_leg: # for now just checking dx and dy

    dx = z_cam*np.tan(np.radians(45+alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
    dy = z_cam*np.tan(np.radians(45+alpha_h))-l_cam
    
    # else: # clipping to only go down by 10 cm increments

    #     z_leg = 0.1
    #     z_cam = z_leg+h_cam

    #     dx = z_cam*np.tan(np.radians(45)+alpha_v)-l_cam
    #     dy = z_cam*np.tan(np.radians(45)+alpha_h)-l_cam

        # do the calculations as if we were only 10 cm in the air

    print('dx ', dx, ' dy ', dy)
        

    return dx, dy

def main(args = None):
    rclpy.init(args = args)

    camera = 0
    cap = cv2.VideoCapture(camera)
    node = BottomCameraNode()
    detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
    
    msg = BottomCamera()
    while True:
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)
        # cv2.imsh ow('frame',frame)
        
        msg.cx = -1
        msg.cy = -1
        if detections:
            cv2.imwrite('image.png', frame)
            msg.found = True
            msg.cx = int(detections[0].center[0])
            msg.cy =int(detections[0].center[1])
            print(msg.cx, msg.cy)
            
            #print('found apriltag')
            # _, _ = self.get_april_horiz_distance(msg.cx, msg.cy)
        else:
            msg.found = False
        node.update_msg(msg)
        rclpy.spin_once(node)
        
        

    # rclpy.spin(node)
    # node.destroy_node
    # rclpy.shutdown()
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
            




            