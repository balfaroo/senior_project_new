#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

import depthai as dai
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
import datetime


''' need to rewrite for oak camera'''
def get_depth_coords(x_rgb, y_rgb, rgb_width, rgb_height, depth_width, depth_height):
    RGB_FOV_HORIZONTAL = 69 # deg
    RGB_FOV_VERITCAL = 42

    DEPTH_FOV_HORIZONTAL = 87
    DEPTH_FOV_VERTICAL = 58

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

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -0.65 # raised from -0.4 and 0.55
        self.april_spotted = False
        self.dist_to_april = 0.0
        self.forward_dist = 0.0
        self.forward_step_size = 0.1
        self.initial_heading = 0.0

        self.dx_inst = 0.0 # local x north, y east. body x forward, y right. need to convert with heading. these coordinates are in body fram
        self.dy_inst = 0.0
        self.yaw_inst = 0.0
        self.z_inst = 0.0

        self.x_local = 0.0 # vars for tracking positions
        self.y_local = 0.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
    

    # instructions provided in body frame
    def set_position_inst(self, x_b: float, y_b: float, yaw_b: float, z_b = -0.55):
        self.dx_inst = x_b
        self.dy_inst = y_b
        self.yaw_inst = np.radians(yaw_b) # yaw instruction will be provided in degrees
        self.z_inst = z_b
        #self.yaw_inst = np.mod(self.yaw_inst+np.pi, 2*np.pi) - np.pi

    def body_to_local(self):
        hding = self.vehicle_local_position.heading # + self.yaw_inst maybe don't need to add yaw since you haven't rotated yet
        return np.cos(hding)*self.dx_inst+np.sin(hding)*self.dy_inst, -np.sin(hding)*self.dx_inst+np.cos(hding)*self.dy_inst, np.mod(hding+self.yaw_inst+np.pi, 2*np.pi) - np.pi


    # def body_to_local_yaw(self, body_yaw: float): outdated
    #     yaw = self.initial_heading+body_yaw
    #     yaw = np.mod(yaw + np.pi, 2*np.pi) - np.pi
    #     return yaw
    
    def set_distance_to_april(self, dist: float):
        self.dist_to_april = dist
    
    def set_detection(self, detected: bool):
        self.april_spotted = detected



    # use body to local now
    # def convert_forward_body_to_local(self, forward_inst: float, new_yaw: float):
    #     #heading = self.vehicle_local_position.heading
    #     return forward_inst*np.cos(new_yaw), forward_inst*np.sin(new_yaw)


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        self.armed = True


    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)


    # for now this is fine since hardcoding a 90 degree turn, but would need to add an argument for yaw in future
    def publish_position_setpoint(self, x: float, y: float, z: float): # now using x and y in body frame, using y = 0
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = self.initial_heading
        #msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def move_as_commanded(self):
        local_dx, local_dy, local_yaw = self.body_to_local()
        msg = TrajectorySetpoint()
        msg.position = [local_dx + self.x_local, local_dy + self.y_local, self.z_inst] # could potentially be better to just do vehicle_local_position.x or .y
        msg.yaw = local_yaw
        self.x_local += local_dx
        self.y_local += local_dy
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[self.x_local, self.y_local, self.takeoff_height, np.degrees(local_yaw)]}")



    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()



        elif abs(self.vehicle_local_position.z - self.takeoff_height) > 0.02 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        elif abs(self.vehicle_local_position.z - self.takeoff_height) >= 0.02 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.offboard_setpoint_counter < 40:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.offboard_setpoint_counter += 1 # hover for 3 sec

        # elif self.dist_to_april == 0.0 or self.dist_to_april > 1.5 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.publish_position_setpoint(self.forward_step_size, 0.0, self.takeoff_height)
      
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.move_as_commanded()

        # elif self.dist_to_april <= 1.5 and self.dist_to_april != 0.0:
        #     self.land()
        #     exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
            self.initial_heading = self.offboard_setpoint_counter+self.initial_heading/(self.offboard_setpoint_counter+1) + self.vehicle_local_position.heading/(self.offboard_setpoint_counter+1)


def main(args=None) -> None:
    rgb_width = 640
    rgb_height = 480
    depth_width = 640
    depth_height = 480

    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()


    # Create pipeline
    pipeline = dai.Pipeline()

    # Define source and output
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    #camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    # Properties
    # very slow camRgb.setPreviewSize(1920, 1080)
    camRgb.setPreviewSize(400, 400)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    camRgb.preview.link(xoutRgb.input)

    detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

    # cx = 331.85198975
    # cy = 247.21032715
    # fx = 509.9508667
    # fy = 509.9508667

    cx = 207.4074707
    cy = 204.50645447
    fx = 318.7192688
    fy = 318.7192688

    L_paper = 150.0 ## mm


        # fx/=1000
        # fy/=1000
    L_paper/=1000

    horizontal_distance = 0.1 # move in 10 cm increments
    yaw_delta = 15 # search for tag in increments of 15 degree
    spun = False
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
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        while True:
            #inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
            try:
                #print('getting frame')
                # now = datetime.datetime.now()
                # print(now)
                inRgb = qRgb.get().getCvFrame()
                #color_img = np.asanyarray(inRgb)
                # Retrieve 'bgr' (opencv format) frame
                gray = cv2.cvtColor(inRgb, cv2.COLOR_BGR2GRAY) 
                detections = detector.detect(gray, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=L_paper)
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

                    pose_R = d.pose_R
                        #yaw = np.degrees(np.arctan(pose_R[0][1]/pose_R[0][0]))
                        #yaw = np.degrees(np.arctan(pose_R[1][2]/pose_R[2][2]))
                    yaw = -np.degrees(np.arcsin(pose_R[0][2])) ## this the one chief]

                    print('yawing to tag')

                    if abs(yaw) > abs(yaw_delta):
                        offboard_control.set_position_inst(0.0, 0.0, -np.sign(yaw)*yaw_delta) #just want to get drone lined up now
                    else:
                        offboard_control.set_position_inst(0.0, 0.0, -yaw)
                    #print('yaw ,', yaw, ' degrees')
                else:
                    offboard_control.set_position_inst(0.0, 0.0, yaw_delta)
                    print('yawing to find tag')
                
                
                rclpy.spin_once(offboard_control)
                spun = True

            except Exception as e:     
                print(e)           
                offboard_control.destroy_node()
                if spun:
                    rclpy.shutdown()
                sys.exit(0)
                

    ''' old, realsense version
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, rgb_width, rgb_height, rs.format.bgr8, 30)

    # Start streaming
    try:
        profile = pipeline.start(config)
        print('streaming depth')
    except:
        print('pipeline failed to start')
        offboard_control.destroy_node()
        sys.exit(0)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)
    #detector = apriltag("tag36h11")
    #armed = False

    detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    cx = 322.432983398438
    cy = 252.699798583984
    fx = 606.868835449219 ## focal lenghts most likely in mm, apparently on actual april robotics they use px
    fy = 606.979309082031
    L_paper = 150.0 ## mm

    # fx/=1000
    # fy/=1000
    L_paper/=1000
    spun = False
    horizontal_distance = 0.1 # move in 10 cm increments
    yaw_delta = 15 # search for tag in increments of 15 degrees
    while True:
        try:
            

            # rclpy.spin_once(offboard_control)
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY) 
            #detections = detector.detect(gray)
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
                yaw = -np.degrees(np.arcsin(pose_R[0][2])) ## this the one chief]

                dx, dy = get_depth_coords(center_x, center_y, rgb_width, rgb_height, depth_width, depth_height)
                dist = depth_image[dy, dx]*depth_scale
                print('estimaged distance to apriltag ', dist)
                if dist > 1.75 or dist == 0.0:
                    ### yaw needs to be negative for the drone to track
                    #offboard_control.set_position_inst(-horizontal_distance, 0.0, yaw) # did negative of the step size bc it was going backwards for some reason
                    if abs(yaw) > abs(yaw_delta):
                        offboard_control.set_position_inst(0.0, 0.0, -np.sign(yaw)*yaw_delta) #just want to get drone lined up now
                    else:
                        offboard_control.set_position_inst(0.0, 0.0, -yaw)
                        
                else:
                    offboard_control.set_position_inst(0.0, 0.0, 0.0, 0.0)
                
            

            else:
                # offboard_control.set_distance_to_april(0.0)
                # offboard_control.set_detection(False)
                offboard_control.set_position_inst(0.0, 0.0, yaw_delta)
            rclpy.spin_once(offboard_control)
            spun = True
        except Exception as e:
            print(e)
            offboard_control.destroy_node
            
            if spun:
                rclpy.shutdown()
                sys.exit(0)


            pipeline.stop()


            # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # #depth_colormap_dim = depth_colormap.shape
            # #color_colormap_dim = color_image.shape

            # gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY) 

            # detections = detector.detect(gray_image)

            # if len(detections) > 0 and not armed:
            #     print('first detection spotted')
            #     offboard_control.arm()
            #     armed = True
    '''



    #offboard_control.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)