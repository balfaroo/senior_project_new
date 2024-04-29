#!/usr/bin/env python3
''' ROS2 Node for flying to a nearby object that has an AprilTag on it'''
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

        self.subscription = self.create_subscription(BottomCamera, 'bottom_camera', self.listener_callback, 10)


        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -0.5 # raised from -0.4 and 0.55 increase back to 0.65 once drift issue figured out
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
    def set_position_inst(self, x_b: float, y_b: float, yaw_b: float, z_b = -0.65):
        self.dx_inst = x_b
        self.dy_inst = y_b
        self.yaw_inst = np.radians(yaw_b) # yaw instruction will be provided in degrees
        self.z_inst = z_b
        #self.yaw_inst = np.mod(self.yaw_inst+np.pi, 2*np.pi) - np.pi
    
    def set_distance_to_april(self, dist: float):
        self.dist_to_april = dist
    
    def set_detection(self, detected: bool):
        self.april_spotted = detected


    def body_to_local(self, x, y):
        ''' function for converting body dx and dy commands into the PX4 local coordinate system, which is defined by
        an x-axis aligned with North, y-axis aligned with East, and a heading angle increasing from North to East'''
        hding = self.vehicle_local_position.heading
        return np.cos(hding)*x-np.sin(hding)*y, np.sin(hding)*x+np.cos(hding)*y


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

    def publish_position_setpoint(self, x: float, y: float, z: float, delta_yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        self.x_local += x
        self.y_local += y
        # y = self.vehicle_local_position.y + y 
        msg.position = [self.x_local, self.y_local, z]
        msg.yaw = self.target_heading + np.radians(delta_yaw)
        msg.yaw = np.mod(msg.yaw+np.pi, 2*np.pi)-np.pi
        #msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

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

    
    def get_april_horiz_distance(self, cx, cy):
        '''function for getting the distance between the bottom camera and the object. This was written when we
        were still using the camera that was angled at 45 degrees'''
        h_cam = 0.063
        l_cam = 0.063
        h_fov = 41
        v_fov = 66

        ncols = 720
        nrows = 1280

        v_ang_perpx = v_fov/nrows
        h_ang_perpx =  h_fov/ncols
        
        h_of = 0.158

        z = abs(self.takeoff_height)
        z_leg = z - h_of
        z_cam = z_leg+h_cam

        alpha_h = (cx-ncols/2)*h_ang_perpx
        alpha_v = -(cy-nrows/2)*v_ang_perpx
)
        
        if z_leg<0.1: 

            dx = z_cam*np.tan(np.radians(45+alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
            dy = z_cam*np.tan(np.radians(alpha_h))-l_cam
            
        else: # clipping to only go down by 10 cm increments

            z_leg = 0.1
            z_cam = z_leg+h_cam

            dx = z_cam*np.tan(np.radians(45+alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
            dy = z_cam*np.tan(np.radians(alpha_h))-l_cam

    

        return dx, dy


    
    def listener_callback(self, msg):
        ''' callback function for bottom camera '''
        if abs(self.vehicle_local_position.z-self.takeoff_height) < 0.02: # only move once at the appropriate height
            if msg.found:
                dx, dy = self.get_april_horiz_distance(msg.cx, msg.cy)
                print('body dx, dy: ', dx, dy)
                self.takeoff_height += 0.1 # decrease altitude by 10 cm
                dx, dy, = self.body_to_local(dx, dy)
                if abs(self.vehicle_local_position.x - self.x_local) < 0.05 and abs(self.vehicle_local_position.y - self.y_local) < 0.05: # only set new points if previous ones have been reached
                   print('local dx, dy ', dx, dy)
                   self.publish_position_setpoint(dx, dy, self.takeoff_height, 0.0)
                print('detected apriltag!')
            else:
                self.target_heading += np.radians(5)
                self.target_heading = np.mod(self.target_heading + np.pi, 2*np.pi) - np.pi
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0) # last argument angle increment in degrees



    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 15: ## raised delay to 1.5 s for heading to stabilitze
            self.target_heading = self.vehicle_local_position.heading
            self.engage_offboard_mode()
            self.arm()


        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: 
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, self.target_heading)
        

        if self.offboard_setpoint_counter < 16:
            self.offboard_setpoint_counter += 1
            self.initial_heading = self.offboard_setpoint_counter+self.initial_heading/(self.offboard_setpoint_counter+1) + self.vehicle_local_position.heading/(self.offboard_setpoint_counter+1)


def main(args=None) -> None:

    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)