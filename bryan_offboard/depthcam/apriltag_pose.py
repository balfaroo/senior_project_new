import cv2
import numpy as np
import depthai as dai
from pupil_apriltags import Detector, Detection
import datetime

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
#camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
#camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

# Properties
# very slow camRgb.setPreviewSize(1920, 1080)
camRgb.setPreviewSize(640, 480)
#camRgb.setPreviewKeepAspectRatio(True)
#camRgb.setPreviewSize(400, 400) #down from 480 p bc of lag
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

camRgb.preview.link(xoutRgb.input)

### hfov = 38 degress, vfov = 35.2 degrees for 400x400

detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

''' old parameters for a 1080 p stream 
cx = 995.555908
cy = 561.630981
fx = 1529.85254 ## focal lenghts most likely in mm, apparently on actual april robotics they use px
fy = 1529.85254
'''
# 640x480
cx = 331.85198975
cy = 247.21032715
fx = 509.9508667
fy = 509.9508667

# cx = 207.4074707
# cy = 204.50645447
# fx = 318.7192688
# fy = 318.7192688
L_paper = 150.0 ## mm


    # fx/=1000
    # fy/=1000
L_paper/=1000

with dai.Device(pipeline) as device:

    print('Connected cameras:', device.getConnectedCameraFeatures())
    # Print out usb speed
    print('Usb speed:', device.getUsbSpeed().name)
    # Bootloader version
    if device.getBootloaderVersion() is not None:
        print('Bootloader version:', device.getBootloaderVersion())
    # Device name
    print('Device name:', device.getDeviceName(), ' Product name:', device.getProductName())

    # Output queue will be used to get the rgb frames from the output defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False) #maxsize changed to 1 from 4

    while True:
        start = datetime.datetime.now()
        #inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
        #print('getting frame')
        #now = datetime.datetime.now()
        #print(now)
        inRgb = qRgb.get()
        inRgb = inRgb.getCvFrame()
        #color_img = np.asanyarray(inRgb)
        # Retrieve 'bgr' (opencv format) frame
        gray = cv2.cvtColor(inRgb, cv2.COLOR_RGB2GRAY) 
        detections = detector.detect(gray, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=L_paper)
        if detections:
                # d = detections[0]
                # center = d['center']
                # cX = int(center[0])
                # cY = int(center[1])
                # dx, dy = get_depth_coords(cX, cY, rgb_width, rgb_height, depth_width, depth_height)
                # dist = depth_image[dy, dx]*depth_scale
                # offboard_control.set_distance_to_april(dist)
                # offboard_control.set_detection(True)
            d = detections[0]
            center = d.center
            center_x = int(center[0])
            center_y = int(center[1])

            pose_R = d.pose_R
                #yaw = np.degrees(np.arctan(pose_R[0][1]/pose_R[0][0]))
                #yaw = np.degrees(np.arctan(pose_R[1][2]/pose_R[2][2]))
            try:
                yaw = -np.degrees(np.arcsin(pose_R[0][2])) ## this the one chief]
            except:
                yaw = 0.0
            print('yaw ,', yaw, ' degrees')


       #cv2.namedWindow("rgb", cv2.WINDOW_AUTOSIZE)

        #cv2.imshow("rgb", inRgb)

        if cv2.waitKey(1) == ord('q'):
            break
        #print(datetime.datetime.now() - start)

