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
        self.armed = False
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -0.4 # raised from -0.4 and 0.55 increase back to 0.65 once drift issue figured out


        self.x_local = 0.0 # vars for tracking positions
        self.y_local = 0.0 
        self.land_height = 0.0

        self.target_heading = 0.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    
    
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


    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10: 
            self.target_heading = self.vehicle_local_position.heading
            self.land_height = self.vehicle_local_position.z
            self.offboard_setpoint_counter+=1
            self.engage_offboard_mode()
            self.get_logger().info('arming drone')
            self.arm()


        elif self.armed and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: # needed so that it stays hovering even when close
            self.publish_position_setpoint(self.x_local, self.y_local, self.takeoff_height, self.target_heading)
            if abs(self.vehicle_local_position.z - self.takeoff_height) < 0.05 and self.offboard_setpoint_counter < 21:
                self.offboard_setpoint_counter += 1
                self.get_logger().info('set servo to min')
            if abs(self.vehicle_local_position.z - self.land_height) < 0.05 and self.offboard_setpoint_counter < 61 and self.offboard_setpoint_counter >= 21:
                self.offboard_setpoint_counter += 1
            elif self.offboard_setpoint_counter < 61 and self.offboard_setpoint_counter >= 21 and abs(self.vehicle_local_position.z - self.takeoff_height) < 0.02:
                self.takeoff_height += 0.05
            
            


        if self.offboard_setpoint_counter == 20:
            self.offboard_setpoint_counter += 1    

        if self.offboard_setpoint_counter == 60:
            self.offboard_setpoint_counter += 1
            self.takeoff_height = -0.4  


        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


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