#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from bryan_msgs.msg import BottomCamera
import depthai as dai
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
import datetime



def april_horiz_distance(cx, cy, frame):
    h_cam = 0.063
    l_cam = 0.063
    h_fov = 41
    v_fov = 66

    v_ang_perpx = frame.shape[0]/v_fov
    h_ang_perpx = frame.shape[1]/h_fov

    z 


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
        self.subscription = self.create_subscription(BottomCamera, 'bottom_camera', self.listener_callback, 10)
    


    def get_april_horiz_distance(self, cx, cy, frame):
        h_cam = 0.063
        l_cam = 0.063
        h_fov = 41
        v_fov = 66

        v_ang_perpx = frame.shape[0]/v_fov
        h_ang_perpx = frame.shape[1]/h_fov
        
        h_of = 0.158

        z = abs(self.vehicle_local_position.z)
        z_leg = z - h_of
        z_cam = z_leg+h_cam

        alpha_h = (cx-frame.shape[1]/2)*h_ang_perpx
        alpha_v = (cx-frame.shape[0]/2)*v_ang_perpx

        dx = z_cam*np.tan(np.radians(45)+alpha_v)-l_cam # alpha_v b/c x for the drone is forward/up in the picture 
        dy = z_cam*np.tan(np.radians(45)+alpha_h)-l_cam

        return dx, dy

    # instructions provided in body frame
    def set_position_inst(self, x_b: float, y_b: float, yaw_b: float, z_b = -0.65):
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
    def publish_position_setpoint(self, x: float, y: float, z: float, delta_yaw: float): # now using x and y in body frame, using y = 0
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = self.vehicle_local_position.heading + np.radians(delta_yaw)
        msg.yaw = np.mod(msg.yaw+np.pi, 2*np.pi)-np.pi
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

    def listener_callback(self, msg):
        if msg.found:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)
            print('detected apriltag!')
        else:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 15.0)



    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 15: ## raised delay to 1.5 s for heading to stabilitze
            self.engage_offboard_mode()
            self.arm()



        elif abs(self.vehicle_local_position.z - self.takeoff_height) > 0.02 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)

        # elif abs(self.vehicle_local_position.z - self.takeoff_height) >= 0.02 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.offboard_setpoint_counter < 40:
        #     self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        #     self.offboard_setpoint_counter += 1 # hover for 3 sec

        # elif self.dist_to_april == 0.0 or self.dist_to_april > 1.5 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.publish_position_setpoint(self.forward_step_size, 0.0, self.takeoff_height)
      
        # elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.move_as_commanded()

        # elif self.dist_to_april <= 1.5 and self.dist_to_april != 0.0:
        #     self.land()
        #     exit(0)

        if self.offboard_setpoint_counter < 16:
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
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


   

    # detector = Detector(
    #         families="tag36h11",
    #         nthreads=1,
    #         quad_decimate=1.0,
    #         quad_sigma=0.0,
    #         refine_edges=1,
    #         decode_sharpening=0.25,
    #         debug=0
    #     )


    # horizontal_distance = 0.1 # move in 10 cm increments (clip)
    # yaw_delta = 15 # search for tag in increments of 15 degree
    # spun = False
    # next_height = -0.55 ## 10 cm increments
    # camera = 0
    # cap = cv2.VideoCapture(camera)
    # while True:
    #     #inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
    #     try:
    #         #print('getting frame')
    #         # now = datetime.datetime.now()
    #         # print(now)
    #         ret, frame = cap.read()
    #         #color_img = np.asanyarray(inRgb)
    #         # Retrieve 'bgr' (opencv format) frame
    #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    #         detections = detector.detect(gray)
    #         #detections = False
    #         if detections:
    #                 # d = detections[0]
    #                 # center = d['center']
    #                 # cX = int(center[0])
    #                 # cY = int(center[1])
    #                 # dx, dy = get_depth_coords(cX, cY, rgb_width, rgb_height, depth_width, depth_height)
    #                 # dist = depth_image[dy, dx]*depth_scale
    #                 # offboard_control.set_distance_to_april(dist)
    #                 # offboard_control.set_detection(True)
    #             d = detections[0]
    #             center = d.center
    #             center_x = int(center[0])
    #             center_y = int(center[1])

    #             inst_x, inst_y = offboard_control.april_horiz_distance(center_x, center_y, frame)

    #             # pose_R = d.pose_R
    #             #     #yaw = np.degrees(np.arctan(pose_R[0][1]/pose_R[0][0]))
    #             #     #yaw = np.degrees(np.arctan(pose_R[1][2]/pose_R[2][2]))
    #             # yaw = -np.degrees(np.arcsin(pose_R[0][2])) ## this the one chief]

    #             print('moving to tag')

    #             inst_x = min(inst_x, horizontal_distance)
    #             inst_y = min(inst_y, horizontal_distance)
    #             offboard_control.set_position_inst(inst_x, inst_y, 0.0, next_height)
    #             next_height -= 10
    #             #print('yaw ,', yaw, ' degrees')
    #         else:
    #             offboard_control.set_position_inst(0.0, 0.0, yaw_delta)
    #             print('yawing to find tag')
            
            
    #         rclpy.spin_once(offboard_control)
    #         spun = True

    #     except Exception as e:     
    #         print(e)           
    #         offboard_control.destroy_node()
    #         if spun:
    #             rclpy.shutdown()
    #         sys.exit(0)
                




    #offboard_control.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)