#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from bryan_msgs.msg import DepthCamera, BottomCamera
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

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -0.45 # raised from -0.4 and 0.55 increase back to 0.65 once drift issue figured out
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

        self.x_local_old = 0.0 # old positions
        self.y_local_old = 0.0

        self.target_heading = 0.0
        self.land = False

        self.pause_counter = 0 # pause to allow drone to geti

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscription_depth = self.create_subscription(DepthCamera, 'depth_camera', self.depth_listener_callback, 10)
        self.subscription_bottom =self.create_subscription(BottomCamera, 'bottom_camera', self.bottom_listener_callback, 10)

        self.depth_tracking = True
        self.bottom_spotted = False
        self.last_yaw_positive = True # if yaw ourself out of the way, need to go opposite way
        self.REF_YAW = np.radians(241.0)
    
    
    def set_distance_to_april(self, dist: float):
        self.dist_to_april = dist
    
    def set_detection(self, detected: bool):
        self.april_spotted = detected


    def body_to_local(self, x, y):
        hding = self.vehicle_local_position.heading # + self.yaw_inst maybe don't need to add yaw since you haven't rotated yet
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


    # for now this is fine since hardcoding a 90 degree turn, but would need to add an argument for yaw in future
    def publish_position_setpoint(self, x: float, y: float, z: float, delta_yaw: float): # now using x and y in body frame, using y = 0, x and y as dx and dy, yaw as delta
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        # y = self.vehicle_local_position.y + y 
        msg.position = [x, y, z]
        msg.yaw = delta_yaw #self.target_heading + np.radians(delta_yaw) ## based on this syntax, may actually end up not using delta_yaw  != 0.0 not doing a delta yaw anymore        
        msg.yaw = np.mod(msg.yaw+np.pi, 2*np.pi)-np.pi
        #msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        if x == 0.0 and y == 0.0:
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


    def depth_listener_callback(self, msg):

        if abs(self.vehicle_local_position.z-self.takeoff_height) < 0.02 and self.depth_tracking: # only move once at the appropriate height and if not tracking by the bottom camera
            if msg.spotted:
                dx = msg.dx/1000
                dy = msg.dy/1000
                self.get_logger().info(f"detected front dx, dy as {[dx, dy]}")
                yaw = np.radians(msg.yaw)
                if yaw > 0.0:
                    self.last_yaw_positive = True
                else:
                    self.last_yaw_positive = False

                if (dx>2.0 or (dx == 0.0 and dy ==0.0)):
                    dx, dy = self.body_to_local(0.1, -np.sign(dy)*min(0.1,abs(dy))) # dy needs to be small, for now will run it so that it is
                    self.x_local = self.vehicle_local_position.x+dx
                    self.y_local = self.vehicle_local_position.y+dy
                    self.target_heading = self.REF_YAW#self.vehicle_local_position.heading+yaw# need to see if this would work better
                    self.target_heading = np.mod(self.target_heading+np.pi, 2*np.pi)-np.pi
                    self.x_local_old = self.x_local
                    self.y_local_old = self.y_local
                    #self.publish_position_setpoint(self.x_local, self.y_local, self.takeoff_height, self.target_heading)

                elif dx<=2.0 and abs(dy < 0.05):
                    self.takeoff_height = 0.0
                    #self.publish_position_setpoint(self.x_local, self.y_local, self.takeoff_height, self.target_heading)
                    self.get_logger().info('LANDING BC OF PROXIMITY TO FRONT TAG')
                elif dx <= 2.0:
                    self.x_local = self.vehicle_local_position.x_local
                    self.y_local = self.vehicle_local_position.y+dy

            else:
                if self.last_yaw_positive:
                    self.target_heading = self.vehicle_local_position.heading - np.radians(5)
                else:
                    self.target_heading = self.vehicle_local_position.heading + np.radians(5)
                self.target_heading = self.REF_YAW #np.mod(self.target_heading+np.pi, 2*np.pi)-np.pi
                #self.publish_position_setpoint(self.x_local, self.y_local, self.takeoff_height, self.target_heading)                

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
        h_obj = 0.04682 # object height, m

        z = abs(self.takeoff_height)
        z_leg = z - h_of
        z_leg_eff = z_leg - h_obj # "effective" altitude of legs - modeling the object as if the ground were on a plane aligned with the top of the object
        z_cam = z_leg_eff+h_cam

        alpha_h = (cx-ncols/2)*h_ang_perpx
        alpha_v = -(cy-nrows/2)*v_ang_perpx

        # print('alpha h ', alpha_h)
        # print('alpha v ', alpha_v)
        
        if z_leg_eff: # for now just checking dx and dy

            dx = z_cam*np.tan(np.radians(45+alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
            dy = z_cam*np.tan(np.radians(alpha_h))
            return dx, dy, True #boolean represents if we can just go straight down now
            
        else: # clipping to only go down by 10 cm increments

            z_leg_eff = 0.05
            z_cam = z_leg_eff+h_cam

            dx = z_cam*np.tan(np.radians(45+alpha_v))-l_cam  # alpha_v b/c x for the drone is forward/up in the picture 
            dy = z_cam*np.tan(np.radians(alpha_h))
            
            return dx, dy, False

            # do the calculations as if we were only 10 cm in the air


    def bottom_listener_callback(self, msg):
        if abs(self.vehicle_local_position.z-self.takeoff_height) < 0.02: # only move once at the appropriate heigt
            if msg.found and not self.bottom_spotted:
                self.get_logger().info('SPOTTED OBJECT')
                self.depth_tracking = False
                self.bottom_spotted = True
                if abs(self.vehicle_local_position.x - self.x_local) < 0.05 and abs(self.vehicle_local_position.y - self.y_local) < 0.05 and abs(self.vehicle_local_position.z - self.takeoff_height) < 0.02:
                    self.x_local = msg.tx
                    self.y_local = msg.ty
                    self.land = msg.land
                if not msg.land:
                    self.takeoff_height += 0.1

            

                self.get_logger().info(f'target positions, {[msg.tx, msg.ty]}')

                # dx, dy, self.land = self.get_april_horiz_distance(msg.cx, msg.cy)
                # self.get_logger().info(f"detected bottom dx, dy as {[dx, dy]}")
                # if abs(self.vehicle_local_position.x-self.x_local) < 0.05 and abs(self.vehicle_local_position.y-self.y_local) < 0.05 and abs(self.target_heading - self.vehicle_local_position.heading) < np.radians(5):
                #     if not self.land:
                #         self.takeoff_height += 0.05 # decrease altitude by 5 cm
                #     dx, dy, = self.body_to_local(dx, dy)
                #     self.x_local= self.vehicle_local_position.x+dx
                #     self.y_local= self.vehicle_local_position.y+dy
                #     self.publish_position_setpoint(self.x_local, self.y_local, self.takeoff_height, self.target_heading)
                # if abs(self.vehicle_local_position.x - self.x_local) < 0.05 and abs(self.vehicle_local_position.y - self.y_local) < 0.05: # only set new points if previous ones have been reached
                #    print('local dx, dy ', dx, dy)
                #    self.publish_position_setpoint(dx, dy, self.takeoff_height, self.target_heading)
                print('detected apriltag!')
            else:
                self.target_heading = self.vehicle_local_position.heading + np.radians(5)
                self.target_heading = np.mod(self.target_heading + np.pi, 2*np.pi) - np.pi
                # self.publish_position_setpoint(self.x_local, self.y_local, self.takeoff_height, self.target_heading) # last argument angle increment in degrees


    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 15: ## raised delay to 1.5 s for heading to stabilitze
            self.target_heading = self.vehicle_local_position.heading
            self.REF_YAW = self.vehicle_local_position.heading
            self.engage_offboard_mode()
            self.get_logger().info('arming drone')
            self.arm()


        elif self.armed and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: # needed so that it stays hovering even when close
            if self.land and abs(self.vehicle_local_position.x - self.x_local) < 0.05 and abs(self.vehicle_local_position.y - self.y_local) < 0.05:
                self.takeoff_height = 0.0
                self.get_logger().info('LANDING BC OF PROXIMITY TO BOTTOM TAG')
            self.publish_position_setpoint(self.x_local, self.y_local, self.takeoff_height, self.target_heading)
        

        # elif abs(self.vehicle_local_position.z - self.takeoff_height) > 0.02 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, self.target_heading)

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