import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
from bryan_msgs.msg import DepthCamera
import depthai as dai

def get_depth_coords(x_rgb, y_rgb, rgb_width = 640, rgb_height = 480, depth_width = 640, depth_height = 400):
    RGB_FOV_HORIZONTAL = 50 # deg
    RGB_FOV_VERITCAL = 38

    DEPTH_FOV_HORIZONTAL = 80
    DEPTH_FOV_VERTICAL = 55

    # center of rgb
    half_x = rgb_width/2
    half_y = rgb_height/2

    rgb_hz_angpp = RGB_FOV_HORIZONTAL/rgb_width
    rgb_vt_angpp = RGB_FOV_VERITCAL/rgb_height

    angle_horiz = int((x_rgb - half_x)*(rgb_hz_angpp))
    angle_vert = int((y_rgb-half_y)*(rgb_vt_angpp))

    ## center of depth
    half_x = depth_width/2
    half_y = depth_height/2

    angle_per_depth_px_horizontal = DEPTH_FOV_HORIZONTAL/depth_width
    angle_per_depth_px_vertical = DEPTH_FOV_VERTICAL/depth_height
    depth_x = int(half_x + angle_horiz/angle_per_depth_px_horizontal)
    depth_y = int(half_y + angle_vert/angle_per_depth_px_vertical)

    return depth_x, depth_y



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


        self.publisher = self.create_publisher(DepthCamera, 'depth_camera', 10) 
        self.timer = self.create_timer(4.0, self.timer_callback)


        # depth cam setup
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.spatialLocationCalculator = self.pipeline.create(dai.node.SpatialLocationCalculator)

        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRgb.setStreamName("rgb")

        self.xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        self.xoutSpatialData = self.pipeline.create(dai.node.XLinkOut)
        self.xinSpatialCalcConfig = self.pipeline.create(dai.node.XLinkIn)

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

        self.camRgb.preview.link(self.xoutRgb.input)
        # Config
        self.topLeft = dai.Point2f(0.4, 0.4) # carry overs from sample code
        self.bottomRight = dai.Point2f(0.6, 0.6)
        self.newConfig = False

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
        self.config.roi = dai.Rect(self.topLeft, self.bottomRight)

        self.spatialLocationCalculator.inputConfig.setWaitForMessage(False)
        self.spatialLocationCalculator.initialConfig.addROI(self.config)

        # Linking
        self.monoLeft.out.link(self.stereo.left)
        self.monoRight.out.link(self.stereo.right)

        self.spatialLocationCalculator.passthroughDepth.link(self.xoutDepth.input)
        self.stereo.depth.link(self.spatialLocationCalculator.inputDepth)

        self.spatialLocationCalculator.out.link(self.xoutSpatialData.input)
        self.xinSpatialCalcConfig.out.link(self.spatialLocationCalculator.inputConfig)
        self.cfg = dai.SpatialLocationCalculatorConfig()
        self.cfg.addROI(self.config)

        

        print('initialization finished')

        # self.vehicle_local_position_subscriber = self.create_subscription(
        #     VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        # self.vehicle_local_position = VehicleLocalPosition()

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def timer_callback(self):
        msg = DepthCamera()
        msg.dx = 0.0
        msg.dy = 0.0
        msg.yaw = 0.0
        msg.spotted = False
        with dai.Device(self.pipeline) as device:
            depthQueue = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
            spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=1, blocking=False)
            spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")
            qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

            inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

            depthFrame = inDepth.getFrame() # depthFrame values are in millimeters
            # print('i think its stuck here')
            inRgb = qRgb.get()
            inRgb = inRgb.getCvFrame()  

            depth_downscaled = depthFrame[::4]
            if np.all(depth_downscaled == 0):
                min_depth = 0  # Set a default minimum depth value when all elements are zero
            else:
                min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
            max_depth = np.percentile(depth_downscaled, 99)
            depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

            spatialData = spatialCalcQueue.get().getSpatialLocations()
            gray = cv2.cvtColor(inRgb, cv2.COLOR_RGB2GRAY) 
            detections = self.detector.detect(gray)

            if detections:
                msg.spotted = True
                d = detections[0]
                center = d.center
                cx = center[0]
                cy = center[1]
                dx, dy = get_depth_coords(cx, cy)

                self.topLeft = dai.Point2f(dx - 5, dy-5)  ## getting depth of middle of frame if apriltag is there
                self.bottomRight = dai.Point2f(dx+5, dy+5)

                self.config.roi = dai.Rect(self.topLeft, self.bottomRight)
                self.config.calculationAlgorithm = self.calculationAlgorithm
                self.cfg = dai.SpatialLocationCalculatorConfig()
                self.cfg.addROI(self.config)
                spatialCalcConfigInQueue.send(self.cfg)
                # self.newConfig = False

                # self.newConfig = True
                for depthData in spatialData:
                    roi = depthData.config.roi
                    roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
                    xmin = int(roi.topLeft().x)
                    ymin = int(roi.topLeft().y)
                    xmax = int(roi.bottomRight().x)
                    ymax = int(roi.bottomRight().y)


                    
                    xmin = -5
                    xmax = 5
                    ymin = -5
                    ymax = 5

                    depthMin = depthData.depthMin
                    depthMax = depthData.depthMax

                    msg.dx = depthData.spatialCoordinates.z
                    msg.dy = depthData.spatialCoordinates.x

                    if self.newConfig:
                        pass
                        self.config.roi = dai.Rect(self.topLeft, self.bottomRight)
                        self.config.calculationAlgorithm = self.calculationAlgorithm
                        self.cfg = dai.SpatialLocationCalculatorConfig()
                        self.cfg.addROI(self.config)
                        spatialCalcConfigInQueue.send(self.cfg)
                        self.newConfig = False
        
        self.publisher.publish(msg)
        print('msg sent')
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
    node = DepthCameraNode()
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
            




            