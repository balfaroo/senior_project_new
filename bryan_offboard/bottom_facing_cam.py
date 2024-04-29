'''implementation of a node for publishing messages from the bottom camera'''
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

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)


        self.publisher = self.create_publisher(BottomCamera, 'bottom_camera', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.cap = cv2.VideoCapture(0) # initialize the camera


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position


    def body_to_local(self, x, y):
        ''' function for converting body dx and dy commands into the PX4 local coordinate system, which is defined by
        an x-axis aligned with North, y-axis aligned with East, and a heading angle increasing from North to East'''       
        hding = self.vehicle_local_position.heading
        return np.cos(hding)*x-np.sin(hding)*y, np.sin(hding)*x+np.cos(hding)*y

    def timer_callback(self):
        '''timer callback function that publishes messages to bottom camera topic'''
        ret, frame = self.cap.read()
        
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE) # rotation needed because of way camera was mounted
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # perform detection on gray scale
        detections = self.detector.detect(gray)
        msg = BottomCamera()
        msg.cx = -1
        msg.cy = -1
        msg.land = False
        if detections:
            cv2.imwrite('image.png', gray) # saves a reference image so you can see what the object looks like from drone POV
            msg.found = True
            msg.cx = int(detections[0].center[0])
            msg.cy =int(detections[0].center[1]) # center of apriltag
            msg.tx, msg.ty, msg.land = self.get_april_horiz_distance(msg.cx, msg.cy) # old fields that would tell communicate
            # the drone needed to move to, have since moved to computing those destinations through the offboard node
            msg.tx, msg.ty = self.body_to_local(msg.tx, msg.ty)
            msg.tx = self.vehicle_local_position.x + msg.tx
            msg.ty = self.vehicle_local_position.y + msg.ty
        else:
            msg.found = False
        self.publisher.publish(msg)


        
    def get_april_horiz_distance(self, cx, cy):
        ''' Function for getting distance from bottom camera to object. Mainly redundant and only used for testing
        of the node, since computation of this distance has shifted to the offboard control node'''
        h_cam = 0.083 
        l_cam = 0.084
        h_fov = 41 
        v_fov = 66 

        ncols = 720 
        nrows = 1280

        v_ang_perpx = v_fov/nrows
        h_ang_perpx =  h_fov/ncols
        
        h_of = 0.158 
        h_obj = 0.04682  

        z = abs(self.vehicle_local_position.z)
        print(z)
        z_leg = z - h_of

        # "effective" altitude of legs - modeling the object as if the ground were on a plane aligned with the top of the object
        z_leg_eff = z_leg - h_obj 
        z_cam = z_leg_eff+h_cam

        alpha_h = (cx-ncols/2)*h_ang_perpx # referred to as beta in paper
        alpha_v = -(cy-nrows/2)*v_ang_perpx # referred to as alpha in paper
        
        if z_leg_eff: # for now just checking dx and dy

            dx = z_cam*np.tan(np.radians(alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
            dy = z_cam*np.tan(np.radians(alpha_h))
            if abs(dx) < 0.03 and abs(dy) < 0.03: # checking if we are aligned with object before decreasing altitude
                return dx, dy, True #boolean represents if we can just go straight down now
            else:
                return dx, dy, False
            
        else: # clipping to only go down by 10 cm increments

            z_leg_eff = 0.1
            z_cam = z_leg_eff+h_cam

            dx = z_cam*np.tan(np.radians(45+alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
            dy = z_cam*np.tan(np.radians(alpha_h))
            
            return dx, dy, False

            # do the calculations as if we were only 10 cm in the air
    
def main(args = None):
    rclpy.init(args = args)
    node = BottomCameraNode()
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
            




            