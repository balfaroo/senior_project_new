import cv2
from pupil_apriltags import Detector
import datetime


detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)


horizontal_distance = 0.1 # move in 10 cm increments (clip)
yaw_delta = 15 # search for tag in increments of 15 degree
spun = False
next_height = -0.55 ## 10 cm increments
camera = 0
cap = cv2.VideoCapture(camera)
while True:
    now = datetime.datetime.now()
    #inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
    #print('getting frame')
    # now = datetime.datetime.now()
    # print(now)
    ret, frame = cap.read()
    #color_img = np.asanyarray(inRgb)
    # Retrieve 'bgr' (opencv format) frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    detections = detector.detect(gray)
    print(gray.shape)
    #detections = False
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
    print(datetime.datetime.now() - now)