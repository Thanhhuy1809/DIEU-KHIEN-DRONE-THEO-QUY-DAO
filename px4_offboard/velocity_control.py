#!/usr/bin/env python3

from std_msgs.msg import String

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Twist, Vector3
from math import pi
import math
from std_msgs.msg import Bool


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile)
        
        
        self.trajectory_mode_sub = self.create_subscription(
            String,
            '/trajectory_mode',
            self.trajectory_mode_callback,
            qos_profile)
        
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)


        
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        
        # Trajectory mode variables
        self.trajectory_mode = "MANUAL"  # MANUAL, SINE, CIRCLE
        self.trajectory_start_time = 0.0
        self.trajectory_altitude_reached = False
        self.target_altitude = -30.0  # NED: -30m = 30m above ground
        self.current_altitude = 0.0
        
        # SINE trajectory parameters
        self.sine_amplitude = 5.0       # Amplitude in meters
        self.sine_frequency = 0.1       # Frequency in Hz (lower = slower oscillation)
        self.sine_forward_speed = 2.0   # Forward speed in m/s
        
        # CIRCLE trajectory parameters
        self.circle_radius = 10.0       # Radius in meters
        self.circle_angular_speed = 0.1 # Angular speed in rad/s


    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")
    
    def trajectory_mode_callback(self, msg):
        old_mode = self.trajectory_mode
        self.trajectory_mode = msg.data
        
        if old_mode != self.trajectory_mode:
            self.get_logger().info(f"Trajectory mode changed: {old_mode} → {self.trajectory_mode}")
            
            if self.trajectory_mode in ["SINE", "CIRCLE"]:
                self.trajectory_start_time = self.get_clock().now().nanoseconds / 1e9
                self.trajectory_altitude_reached = False
                self.get_logger().info(f"Starting {self.trajectory_mode} trajectory - Climbing to 30m first...")
            elif self.trajectory_mode == "MANUAL":
                self.velocity.x = 0.0
                self.velocity.y = 0.0
                self.velocity.z = 0.0
                self.yaw = 0.0
                self.get_logger().info("Manual control resumed")
    
    def local_position_callback(self, msg):
        self.current_altitude = msg.z  # NED: negative = up

  
    def arm_timer_callback(self):

        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True   


    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

 
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) 
        self.get_logger().info("Takeoff command send")

 
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    
        msg.command = command  
        msg.target_system = 1  
        msg.target_component = 1 
        msg.source_system = 1 
        msg.source_component = 1  
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
#-------------------------------------------------------------------
    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass


    #receives Twist commands from Teleop and converts NED -> FLU
    def offboard_velocity_callback(self, msg):
        # Only use manual velocity if in MANUAL mode
        if self.trajectory_mode == "MANUAL":
            #implements NED -> FLU Transformation
            # X (FLU) is -Y (NED)
            self.velocity.x = -msg.linear.y
            # Y (FLU) is X (NED)
            self.velocity.y = msg.linear.x
            # Z (FLU) is -Z (NED)
            self.velocity.z = -msg.linear.z
            # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
            self.yaw = msg.angular.z

    
    def attitude_callback(self, msg):
        orientation_q = msg.q

        
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
    
    def execute_trajectory(self):
        """Execute SINE or CIRCLE trajectory"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.trajectory_start_time
        
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # Step 1: Climb to 30m first
        if not self.trajectory_altitude_reached:
            if self.current_altitude > self.target_altitude + 0.5:  # NED: -30 + 0.5 = -29.5
                # Still climbing
                trajectory_msg.velocity[0] = 0.0
                trajectory_msg.velocity[1] = 0.0
                trajectory_msg.velocity[2] = -1.0  # NED: -1 = climb at 1 m/s
                
                if int(elapsed_time) % 2 == 0:  # Log every 2 seconds
                    self.get_logger().info(f"Climbing... Current alt: {-self.current_altitude:.1f}m / Target: 30m")
            else:
                # Reached target altitude
                self.trajectory_altitude_reached = True
                self.trajectory_start_time = current_time  # Reset time for trajectory
                self.get_logger().info(f"Reached 30m! Starting {self.trajectory_mode} pattern...")
        
        # Step 2: Execute trajectory pattern
        else:
            if self.trajectory_mode == "SINE":
                # SINE wave trajectory
                # x(t) = v_forward * t (forward motion)
                # y(t) = A * sin(2π * f * t) (sinusoidal lateral motion)
                # z = constant (maintain altitude)
                
                vx = self.sine_forward_speed
                vy = self.sine_amplitude * 2 * math.pi * self.sine_frequency * math.cos(
                    2 * math.pi * self.sine_frequency * elapsed_time
                )
                vz = 0.0  # Maintain altitude
                
                trajectory_msg.velocity[0] = vx
                trajectory_msg.velocity[1] = vy
                trajectory_msg.velocity[2] = vz
                
                if int(elapsed_time * 10) % 10 == 0:  # Log every 1 second
                    self.get_logger().info(f"SINE: t={elapsed_time:.1f}s vx={vx:.2f} vy={vy:.2f}")
            
            elif self.trajectory_mode == "CIRCLE":
                # CIRCLE trajectory
                # x(t) = R * cos(ω * t)
                # y(t) = R * sin(ω * t)
                # Velocity: vx = -R*ω*sin(ω*t), vy = R*ω*cos(ω*t)
                
                omega = self.circle_angular_speed
                R = self.circle_radius
                
                vx = -R * omega * math.sin(omega * elapsed_time)
                vy = R * omega * math.cos(omega * elapsed_time)
                vz = 0.0  # Maintain altitude
                
                trajectory_msg.velocity[0] = vx
                trajectory_msg.velocity[1] = vy
                trajectory_msg.velocity[2] = vz
                
                if int(elapsed_time * 10) % 10 == 0:  # Log every 1 second
                    angle_deg = (omega * elapsed_time * 180 / math.pi) % 360
                    self.get_logger().info(f"CIRCLE: t={elapsed_time:.1f}s angle={angle_deg:.0f}° vx={vx:.2f} vy={vy:.2f}")
        
        # Set NaN for unused fields
        trajectory_msg.position[0] = float('nan')
        trajectory_msg.position[1] = float('nan')
        trajectory_msg.position[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = 0.0
        
        self.publisher_trajectory.publish(trajectory_msg)
        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if(self.offboardMode == True):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)
            
           
            if self.trajectory_mode in ["SINE", "CIRCLE"]:
                self.execute_trajectory()
            else:
                
                cos_yaw = np.cos(self.trueYaw)
                sin_yaw = np.sin(self.trueYaw)
                velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
                velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

                
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                trajectory_msg.velocity[0] = velocity_world_x
                trajectory_msg.velocity[1] = velocity_world_y
                trajectory_msg.velocity[2] = self.velocity.z
                trajectory_msg.position[0] = float('nan')
                trajectory_msg.position[1] = float('nan')
                trajectory_msg.position[2] = float('nan')
                trajectory_msg.acceleration[0] = float('nan')
                trajectory_msg.acceleration[1] = float('nan')
                trajectory_msg.acceleration[2] = float('nan')
                trajectory_msg.yaw = float('nan')
                trajectory_msg.yawspeed = self.yaw

                self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()