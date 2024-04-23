import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
from bryan_msgs.msg import DepthCamera
import depthai as dai
import time

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
        self.msg = DepthCamera()
        self.msg.dx = 0.0
        self.msg.dy = 0.0
        self.msg.yaw = 0.0
        self.msg.spotted = False
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def set_message(self, msg):
        self.msg = msg

    def timer_callback(self):
        self.publisher.publish(self.msg)


def main(args = None):
    rclpy.init(args = args)

    node = DepthCameraNode()
    msg = DepthCamera()
    stepSize = 0.05

    newConfig = False

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")

    xoutDepth = pipeline.create(dai.node.XLinkOut)
    xoutSpatialData = pipeline.create(dai.node.XLinkOut)
    xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

    xoutDepth.setStreamName("depth")
    xoutSpatialData.setStreamName("spatialData")
    xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

    # Properties
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")


    camRgb.setPreviewSize(640, 480)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)


    camRgb.setPreviewSize(640, 480)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    camRgb.preview.link(xoutRgb.input)
    # Config
    topLeft = dai.Point2f(0.4, 0.4)
    bottomRight = dai.Point2f(0.6, 0.6)

    cx = 331.85198975
    cy = 247.21032715
    fx = 509.9508667
    fy = 509.9508667
    L_paper = 150.0 ## mm

    L_paper/=1000

    detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

    config = dai.SpatialLocationCalculatorConfigData()
    config.depthThresholds.lowerThreshold = 100
    config.depthThresholds.upperThreshold = 10000
    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
    config.roi = dai.Rect(topLeft, bottomRight)

    spatialLocationCalculator.inputConfig.setWaitForMessage(False)
    spatialLocationCalculator.initialConfig.addROI(config)

    # Linking
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
    stereo.depth.link(spatialLocationCalculator.inputDepth)

    spatialLocationCalculator.out.link(xoutSpatialData.input)
    xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:


        # Output queue will be used to get the depth frames from the outputs defined above
        depthQueue = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
        spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=1, blocking=False)
        spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")
        qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False) #maxsize changed to 1 from 4

        color = (255, 255, 255)

        print("streaming")

        while True:
            # print('looped')
            inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

            depthFrame = inDepth.getFrame() # depthFrame values are in millimeters
            # print('i think its stuck here')
            inRgb = qRgb.get()
            inRgb = inRgb.getCvFrame()  
            # print('got past')
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
            detections = detector.detect(gray, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=L_paper)
            for d in detections:
                if d.tag_id != 0:
                    continue
                center = d.center
                cx = center[0]
                cy = center[1]
                dx, dy = get_depth_coords(cx, cy)
                # print('cx pcnt dif ', abs(cx-inRgb.shape[1]/2)/(inRgb.shape[1]/2))
                #print(center)
                close = True #abs(cx-inRgb.shape[1]/2)/(inRgb.shape[1]/2) <= 0.1 and abs(cy-inRgb.shape[0]/2)/(inRgb.shape[0]/2) <= 0.1
                # print(depthFrameColor.shape)
                if close:
                    #print('close')
                    newConfig = True
                    for depthData in spatialData:
                        roi = depthData.config.roi
                        roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
                        xmin = int(roi.topLeft().x)
                        ymin = int(roi.topLeft().y)
                        xmax = int(roi.bottomRight().x)
                        ymax = int(roi.bottomRight().y)

                        topLeft = dai.Point2f(-5, -5)
                        bottomRight = dai.Point2f(5, 5)

                        topLeft = dai.Point2f(dx - 5, dy-5)  ## getting depth of middle of frame if apriltag is there
                        bottomRight = dai.Point2f(dx+5, dy+5)
                        xmin = -5
                        xmax = 5
                        ymin = -5
                        ymax = 5

                        depthMin = depthData.depthMin
                        depthMax = depthData.depthMax
                        msg.dx = depthData.spatialCoordinates.z
                        msg.dy = depthData.spatialCoordinates.x
                        msg.spotted = True
                        print(msg.dx, msg.dy)

                        pose_R = d.pose_R
                        try:
                            msg.yaw = -np.degrees(np.arcsin(pose_R[0][2])) ## this the one chief]
                        except:
                            msg.yaw = 0.0
                    # Show the frame

                    if newConfig:
                        config.roi = dai.Rect(topLeft, bottomRight)
                        config.calculationAlgorithm = calculationAlgorithm
                        cfg = dai.SpatialLocationCalculatorConfig()
                        cfg.addROI(config)
                        spatialCalcConfigInQueue.send(cfg)
                        newConfig = False
            node.set_message(msg)
            rclpy.spin_once(node)
            msg.spotted = False
            msg.yaw = 0.0
            #time.sleep(0.3)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
            




            