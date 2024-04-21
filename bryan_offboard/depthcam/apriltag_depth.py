#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
from pupil_apriltags import Detector

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

    print("Use WASD keys to move ROI!")

    while True:
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
        cv2.namedWindow("rgb", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("rgb", inRgb)
        if detections:
            d = detections[0]
            center = d.center
            cx = center[0]
            cy = center[1]
            dx, dy = get_depth_coords(cx, cy)
            # print('cx pcnt dif ', abs(cx-inRgb.shape[1]/2)/(inRgb.shape[1]/2))
            #print(center)
            close = True #abs(cx-inRgb.shape[1]/2)/(inRgb.shape[1]/2) <= 0.1 and abs(cy-inRgb.shape[0]/2)/(inRgb.shape[0]/2) <= 0.1
            print(depthFrameColor.shape)
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

                    

                    # fontType = cv2.FONT_HERSHEY_TRIPLEX
                    # cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1)
                    # cv2.putText(depthFrameColor, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, color)
                    # cv2.putText(depthFrameColor, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, color)
                    # cv2.putText(depthFrameColor, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, color)
                    print("x distance: ", int(depthData.spatialCoordinates.x), "mm")
                    print("y distance: ", int(depthData.spatialCoordinates.y), "mm")
                    print("z distance: ", int(depthData.spatialCoordinates.z), "mm")
                # Show the frame
                

                key = cv2.waitKey(1)
                if key == ord('q'):
                    break
                elif key == ord('w'):
                    if topLeft.y - stepSize >= 0:
                        topLeft.y -= stepSize
                        bottomRight.y -= stepSize
                        newConfig = True
                elif key == ord('a'):
                    if topLeft.x - stepSize >= 0:
                        topLeft.x -= stepSize
                        bottomRight.x -= stepSize
                        newConfig = True
                elif key == ord('s'):
                    if bottomRight.y + stepSize <= 1:
                        topLeft.y += stepSize
                        bottomRight.y += stepSize
                        newConfig = True
                elif key == ord('d'):
                    if bottomRight.x + stepSize <= 1:
                        topLeft.x += stepSize
                        bottomRight.x += stepSize
                        newConfig = True
                elif key == ord('1'):
                    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEAN
                    print('Switching calculation algorithm to MEAN!')
                    newConfig = True
                elif key == ord('2'):
                    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN
                    print('Switching calculation algorithm to MIN!')
                    newConfig = True
                elif key == ord('3'):
                    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MAX
                    print('Switching calculation algorithm to MAX!')
                    newConfig = True
                elif key == ord('4'):
                    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MODE
                    print('Switching calculation algorithm to MODE!')
                    newConfig = True
                elif key == ord('5'):
                    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
                    print('Switching calculation algorithm to MEDIAN!')
                    newConfig = True

                if newConfig:
                    config.roi = dai.Rect(topLeft, bottomRight)
                    config.calculationAlgorithm = calculationAlgorithm
                    cfg = dai.SpatialLocationCalculatorConfig()
                    cfg.addROI(config)
                    spatialCalcConfigInQueue.send(cfg)
                    newConfig = False
        key = cv2.waitKey(1)
        if key == ord('q'):
            break    