#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Control Carla ego vehicle by using AckermannDrive messages
"""

import sys

import numpy
from simple_pid import PID  # pylint: disable=import-error,wrong-import-order

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from carla_ackermann_control import carla_control_physics as phys

from ackermann_msgs.msg import AckermannDrive  # pylint: disable=import-error,wrong-import-order
from std_msgs.msg import Header # pylint: disable=wrong-import-order
from carla_msgs.msg import CarlaEgoVehicleStatus  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleInfo  # pylint: disable=no-name-in-module,import-error
from carla_ackermann_msgs.msg import EgoVehicleControlInfo  # pylint: disable=no-name-in-module,import-error

ROS_VERSION = roscomp.get_ros_version()

if ROS_VERSION == 1:
    from carla_ackermann_control.cfg import EgoVehicleControlParameterConfig # pylint: disable=no-name-in-module,import-error,ungrouped-imports
    from dynamic_reconfigure.server import Server # pylint: disable=no-name-in-module,import-error
if ROS_VERSION == 2:
    from rcl_interfaces.msg import SetParametersResult


class CarlaAckermannControl(CompatibleNode):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor

        """
        super(CarlaAckermannControl, self).__init__("carla_ackermann_control")

        # PID controller
        # the controller has to run with the simulation time, not with real-time
        #
        # To prevent "float division by zero" within PID controller initialize it with
        # a previous point in time (the error happens because the time doesn't
        # change between initialization and first call, therefore dt is 0)
        sys.modules['simple_pid.PID']._current_time = (       # pylint: disable=protected-access
            lambda: self.get_time() - 0.1)

        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        # control info
        self.info = EgoVehicleControlInfo()

        # ackermann drive commands
        self.control_subscriber = self.new_subscription(
            AckermannDrive,
            "/carla/" + self.role_name + "/ackermann_cmd",
            self.ackermann_command_updated,
            qos_profile=10
        )

        # current status of the vehicle
        self.vehicle_status_subscriber = self.new_subscription(
            CarlaEgoVehicleStatus,
            "/carla/" + self.role_name + "/vehicle_status",
            self.vehicle_status_updated,
            qos_profile=10
        )

        # to send command to carla
        self.carla_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            qos_profile=1)

        # report controller info
        self.control_info_publisher = self.new_publisher(
            EgoVehicleControlInfo,
            "/carla/" + self.role_name + "/ackermann_control/control_info",
            qos_profile=1)

        self.prev_vel = 0
        self.prev_error = 0
        self.dt = 0.028
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        #self.info.current.time_sec = self.get_time()
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.

        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = True
        self.info.output.gear = 0
        self.info.restrictions.max_steering_angle = 1.22
        self.info.restrictions.max_speed = 60.0
        self.info.restrictions.max_decel = 8.0
        self.info.restrictions.max_accel = 3.0
        self.current_speed = 0
        self.throttle = 0
        self.ac_throttle = 0.0
        self.error = 0
        self.prev_steering = 0
        self.min_accel = 1.5
        self.current_accel = 0.
        self.engage_speed_reached = False
        self.initial = True    
        self.steering_factor = 0.45
        self.max_steer_angle = 0.7
        self.prev_target_speed = 0.0    

    def vehicle_status_updated(self, vehicle_status):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """

        # set target values
        self.vehicle_status = vehicle_status
        self.info.current.speed = self.vehicle_status.velocity 
        self.info.current.speed_abs = abs(self.vehicle_status.velocity)


    def ackermann_command_updated(self, ros_ackermann_drive):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        self.last_ackermann_msg_received_sec = self.get_time()
        # set target values
        self.set_target_steering_angle(ros_ackermann_drive.steering_angle)
        self.set_target_speed(ros_ackermann_drive.speed)
        self.set_target_accel(ros_ackermann_drive.acceleration)
        self.set_target_jerk(ros_ackermann_drive.jerk)

    def set_target_steering_angle(self, target_steering_angle):
        """
        set target sterring angle
        """
        self.info.target.steering_angle = -target_steering_angle
        if abs(self.info.target.steering_angle) > self.info.restrictions.max_steering_angle:
            self.logerr("Max steering angle reached, clipping value")
            self.info.target.steering_angle = numpy.clip(
                self.info.target.steering_angle,
                -self.info.restrictions.max_steering_angle,
                self.info.restrictions.max_steering_angle)

    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        if abs(target_speed) > self.info.restrictions.max_speed:
            self.logerr("Max speed reached, clipping value")
            self.info.target.speed = numpy.clip(
                target_speed, -self.info.restrictions.max_speed, self.info.restrictions.max_speed)
        else:
            self.info.target.speed = target_speed
        self.info.target.speed_abs = abs(self.info.target.speed)

    def set_target_accel(self, target_accel):
        """
        set target accel
        """
        epsilon = 0.00001
        # if speed is set to zero, then use max decel value
        if self.info.target.speed_abs < epsilon:
            self.info.target.accel = -self.info.restrictions.max_decel
        else:
            self.info.target.accel = numpy.clip(
                target_accel, -self.info.restrictions.max_decel, self.info.restrictions.max_accel)

    def set_target_jerk(self, target_jerk):
        """
        set target accel
        """
        self.info.target.jerk = target_jerk

    def update_controls(self): 

        def loop(timer_event=None):
            
            if (self.info.target.speed > 1.8 and self.initial == True):
                self.initial = False
                self.engage_speed_reached = True
            elif (self.initial == False and self.engage_speed_reached == True and self.info.current.speed < 0.01):
                self.initial = True 
                self.engage_speed_reached = False 


            sp_Kp = 50.0
            sp_Kd = 0.05
            sp_Ki = 0.25
            self.current_accel = (self.info.current.speed - self.prev_vel)/self.dt
            self.info.target.speed = self.info.target.speed
            self.info.current.speed = self.info.current.speed
            self.error = self.info.target.speed - self.info.current.speed
            self.throttle = sp_Kp*(self.error) + sp_Kd*(self.error - self.prev_error)/self.dt + sp_Ki*(self.error + self.prev_error)*self.dt
            if self.throttle > 0:
                self.info.output.throttle = numpy.clip(self.throttle/8.0, 0.0, 1.0)
                self.info.output.brake = 0.0
                self.info.output.steer = self.info.target.steering_angle
                self.info.output.reverse = False
                self.info.output.hand_brake = False 
            elif self.throttle < 0 and self.initial == True:
                self.info.output.throttle = 0.0
                self.info.output.brake = 0.0
                self.info.output.steer = self.info.target.steering_angle
                self.info.output.reverse = False
                self.info.output.hand_brake = False
            elif self.throttle < 0 and self.initial == False:
                self.info.output.throttle = 0.0
                self.info.output.brake = numpy.clip(-self.throttle/8.0, 0.0, 1.0)
                self.info.output.steer = self.info.target.steering_angle
                self.info.output.reverse = False
                self.info.output.hand_brake = False    
            if self.info.target.speed_abs < 0.00001:
                self.info.output.hand_brake = True
            self.prev_steering = self.info.output.steer
            self.prev_error = self.error
            self.prev_vel = self.info.current.speed
            
            self.carla_control_publisher.publish(self.info.output)  
        
        '''    
            if (self.info.target.speed > 1.8 and self.initial == True):
                self.initial = False
                self.engage_speed_reached = True
            elif (self.initial == False and self.engage_speed_reached == True and self.info.current.speed < 0.01):
                self.initial = True 
                self.engage_speed_reached = False   


            self.info.output.steer = (self.info.target.steering_angle / self.max_steer_angle)*self.steering_factor
            speed_diff = self.info.target.speed - self.info.current.speed
            if speed_diff > 0 and self.initial == True:         
                self.info.output.throttle = numpy.clip(self.info.target.speed/10, 0.4, 1.0)           
                self.info.output.brake = 0.0
                self.info.output.hand_brake = False
            elif speed_diff > 0 and self.initial == False:         
                self.info.output.throttle = numpy.clip(self.info.target.speed/10, 0.0, 0.8)        
                self.info.output.brake = 0.0
                self.info.output.hand_brake = False         
            elif speed_diff < 0.0:
                self.info.output.throttle = 0.0
                if self.info.target.speed <= 0.0:                
                    self.info.output.brake = 0.75
                elif  speed_diff > -0.15 and self.initial == False:
                    self.info.output.brake = 0.01
                    print("speed diff LOW: {}".format(speed_diff))     
                elif  speed_diff > -0.5 and speed_diff < -0.15 and self.initial == False:
                    self.info.output.brake = abs(speed_diff)*0.5
                    print("speed diff HIGH: {}".format(speed_diff))
                elif  speed_diff > -0.5 and self.initial == True:
                    self.info.output.brake = 0.0    
                else :
                    self.info.output.brake = 0.05
                    print("speed diff REGARDLESS: {}".format(speed_diff))

            self.carla_control_publisher.publish(self.info.output)          
        '''
            
        self.new_timer(0.028, loop)
        self.spin()


def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("carla_ackermann_control", args=args)

    try:
        controller = CarlaAckermannControl()
        controller.update_controls()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
