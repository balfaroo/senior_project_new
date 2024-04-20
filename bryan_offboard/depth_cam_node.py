import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
from bryan_msgs.msg import BottomCamera
from px4_msgs.msg import VehicleLocalPosition

class DepthCameraNode(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('depth_camera')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.publisher = self.create_publisher(BottomCamera, 'depth_camera', 10) ### CHANGE BOTTOMCAMERA TO DEPTHCAMERA, NEED TO CREATE A MESSAGE TYPE
        self.timer = self.create_timer(0.5, self.timer_callback)


        # depth cam setup
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.monoLeft = pipeline.create(dai.node.MonoCamera)
        self.monoRight = pipeline.create(dai.node.MonoCamera)
        self.stereo = pipeline.create(dai.node.StereoDepth)
        self.spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

        self.camRgb = pipeline.create(dai.node.ColorCamera)
        self.xoutRgb = pipeline.create(dai.node.XLinkOut)
        self.xoutRgb.setStreamName("rgb")

        self.xoutDepth = pipeline.create(dai.node.XLinkOut)
        self.xoutSpatialData = pipeline.create(dai.node.XLinkOut)
        self.xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

        self.xoutDepth.setStreamName("depth")
        self.xoutSpatialData.setStreamName("spatialData")
        self.xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

        # Properties
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoLeft.setCamera("left")
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoRight.setCamera("right")


        self.camRgb.setPreviewSize(640, 480)
        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(True)


        self.camRgb.setPreviewSize(640, 480)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        self.camRgb.preview.link(xoutRgb.input)
        # Config
        self.topLeft = dai.Point2f(0.4, 0.4)
        self.bottomRight = dai.Point2f(0.6, 0.6)

        self.detector = Detector(
                families="tag36h11",
                nthreads=1,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
            )

        self.config = dai.SpatialLocationCalculatorConfigData()
        self.config.depthThresholds.lowerThreshold = 100
        self.config.depthThresholds.upperThreshold = 10000
        self.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        self.config.roi = dai.Rect(topLeft, bottomRight)

        self.spatialLocationCalculator.inputConfig.setWaitForMessage(False)
        self.spatialLocationCalculator.initialConfig.addROI(config)

        # Linking
        self.monoLeft.out.link(stereo.left)
        self.monoRight.out.link(stereo.right)

        self.spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
        self.stereo.depth.link(spatialLocationCalculator.inputDepth)

        self.spatialLocationCalculator.out.link(xoutSpatialData.input)
        self.xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

        # self.vehicle_local_position_subscriber = self.create_subscription(
        #     VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        # self.vehicle_local_position = VehicleLocalPosition()

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def timer_callback(self):
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        detections = self.detector.detect(gray)
        #cv2.imshow('frame',frame)
        msg = BottomCamera()
        msg.cx = -1
        msg.cy = -1
        if detections:
            msg.found = True
            msg.cx = int(detections[0].center[0])
            msg.cy =int(detections[0].center[1])
            #print('found apriltag')
            # _, _ = self.get_april_horiz_distance(msg.cx, msg.cy)
        else:
            msg.found = False
        self.publisher.publish(msg)
        #print('published message')
        

    def get_april_horiz_distance(self, cx, cy):
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

        z = abs(self.vehicle_local_position.z)
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
            




            