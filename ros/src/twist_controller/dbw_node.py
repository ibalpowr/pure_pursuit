#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # create three instances of rospy.Publisher
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # subscribe three ros topics
        rospy.Subscriber('/current_velocity', TwistStamped, 
                            self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, 
                            self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, 
                            self.dbw_enabled_cb, queue_size=1)

        # in kilogram
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # apr14sat2018gary---
        # create an instance of twist controller
        #self.controller = Controller(decel_limit=decel_limit,
        #                             accel_limit=accel_limit,
        #                             wheel_base=wheel_base,
        #                             steer_ratio=steer_ratio,
        #                             max_lat_accel=max_lat_accel,
        #                             max_steer_angle=max_steer_angle)
        self.controller = Controller(vehicle_mass=vehicle_mass,
                                     fuel_capacity=fuel_capacity,
                                     brake_deadband=brake_deadband,
                                     decel_limit=decel_limit,
                                     accel_limit=accel_limit,
                                     wheel_radius= wheel_radius,
                                     wheel_base=wheel_base,
                                     steer_ratio=steer_ratio,
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle)

        # apr14sat2018gary----
        # for current velocity from styx server
        #self.current_velocity = None        
        self.current_lin_velocity = None
        # for proposed velocity from pure pursuit                         
        #self.proposed_velocity = None
        #self.proposed_angular_velocity = None
        self.target_lin_velocity = None
        self.target_ang_velocity = None


        # apr14sat2018gary----
        # for autonomous drive
        #self.activated = False
        self.dbw_enabled = False

        # apr14sat2018gary----
        self.throttle = 0.
        self.brake = 0.
        self.steer = 0.

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # apr14sat2018gary----
            #throttle, brake, steer = self.controller.control(
            #                        self.proposed_velocity,
            #                        self.proposed_angular_velocity,
            #                        self.current_velocity,
            #                        self.activated)
            if not None in (self.current_lin_velocity, 
                            self.target_lin_velocity,
                            self.target_ang_velocity):
                self.throttle, self.brake, self.steer = self.controller.control(
                                                        self.target_lin_velocity,
                                                        self.target_ang_velocity,
                                                        self.current_lin_velocity,
                                                        self.dbw_enabled)
            # apr14sat2018gary----
            #if self.activated:
            #    self.publish(throttle, brake, steer)
            if self.dbw_enabled:
                self.publish(self.throttle, self.brake, self.steer)

        # apr29sun2018jz0229
        #rate.sleep()
            rate.sleep()

    def publish(self, throttle, brake, steer):
        # apr29sun2018jz0345
        rospy.logwarn("dbw sends out throttle: %f, brake: %f, steer: %f",
                        throttle, brake, steer)
        print(" ")
        # may04fri2018jz1114
        rospy.loginfo("target speed: %f [mps]", self.target_lin_velocity)        
        # apr30mon2018jz0435
        rospy.loginfo("current speed: %f [mps]", self.current_lin_velocity)
        print(" ")

        # apr30mon2018jz0548
        # change it back
        # apr29sun2018jz0347
        # sending out either throttle or brake ... but not both in the same time
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

        #if brake != 0:   # means brake is needed
        #    bcmd = BrakeCmd()
        #    bcmd.enable = True
        #    bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        #    bcmd.pedal_cmd = brake
        #    self.brake_pub.publish(bcmd)
        #else:
        #    tcmd = ThrottleCmd()
        #    tcmd.enable = True
        #    tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        #    tcmd.pedal_cmd = throttle
        #    self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)


    def current_velocity_cb(self, msg):
        # apr14sat2018gary----
        #self.current_velocity = msg.twist.linear.x
        self.current_lin_velocity = msg.twist.linear.x


    def twist_cmd_cb(self, msg):
        # apr14sat2018gary----
        #self.proposed_velocity = msg.twist.linear.x
        #self.proposed_angular_velocity = msg.twist.angular.z
        self.target_lin_velocity = msg.twist.linear.x
        self.target_ang_velocity = msg.twist.angular.z

        # apr29sun2018jz0408
        # apr29sun2018jz0759
        #rospy.loginfo("proposed linear velocity and twist angular velocity ") 
        #rospy.loginfo("from pure pursuit: %f and %f", self.target_lin_velocity,
        #                                              self.target_ang_velocity)
        #print(" ")

    def dbw_enabled_cb(self, msg):
        # apr14sat2018gary----
        #self.activated = msg.data
        rospy.loginfo('DBW_ENABLED : {}'.format(msg.data))
        self.dbw_enabled = msg.data



if __name__ == '__main__':
    DBWNode()
