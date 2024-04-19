import cv2
import depthai as dai
import numpy as np
import sys

### Start of depth setup

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)

xout.setStreamName("disparity")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)

### End of depth setup/start of rgb setup

camRgb = pipeline.create(dai.node.ColorCamera)
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
camRgb.setPreviewSize(640, 480)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
camRgb.preview.link(xoutRgb.input)

with dai.Device(pipeline) as device:

    # Output queue will be used to get the disparity frames from the outputs defined above
    qDepth = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
    qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

    ## depth capture
    # inDisparity = qDepth.get()  # blocking call, will wait until a new data has arrived
    # frame = inDisparity.getFrame()
    # # Normalization for better visualization
    # frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

    # #cv2.imshow("disparity", frame)

    # # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
    # frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)

    # ## rgb capture
    # inRgb = qDepth.get().getCvFrame()

    # depth_width = frame.shape[1]
    # depth_height = frame.shape[0]
    # rgb_width = inRgb.shape[1]
    # rgb_height = inRgb.shape[0]

    rgb_frames = []
    depth_frames = []

    print('starting video capture')

    while True:
        try:

            ## depth capture
            inDisparity = qDepth.get()  # blocking call, will wait until a new data has arrived
            frame = inDisparity.getFrame()
            # Normalization for better visualization
            frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

            #cv2.imshow("disparity", frame)

            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)

            ## rgb capture
            inRgb = qRgb.get().getCvFrame()
            rgb_frames.append(inRgb)
            depth_frames.append(frame)
        except KeyboardInterrupt:
            rgbSize = (rgb_frames[0].shape[1], rgb_frames[0].shape[0])
            rgbOut = cv2.VideoWriter('rgb.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 20, rgbSize)

            depthSize = (depth_frames[0].shape[1], depth_frames[0].shape[0])
            depthOut = cv2.VideoWriter('depth.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 20, depthSize)

            for i in range(len(rgb_frames)):
                rgbOut.write(rgb_frames[i])
            rgbOut.release()

            for i in range(len(depth_frames)):
                depthOut.write(depth_frames[i])
            depthOut.release()

            print('successfully captured both videos')
            sys.exit(0)




