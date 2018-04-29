from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

import time
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
ONE_GALLON = 0.00379

class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):

        min_speed = 0.1 # m/s
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # Throttle PID parameters
        kp = 1
        ki = 0.04
        kd = 0.01
        mn = decel_limit
        mx = accel_limit
        self.throttle_pid = PID(kp, ki, kd, mn, mx)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

        self.total_mass = self.vehicle_mass + self.fuel_capacity*ONE_GALLON*GAS_DENSITY
        self.brake_const = self.total_mass * self.wheel_radius

        self.previous_time = rospy.get_time()


    def control(self, target_lin_vel, target_ang_vel, current_lin_vel, dbw_enabled):
        if self.previous_time is None or not dbw_enabled:
            self.previous_time = rospy.get_time()
            self.throttle_pid.reset()
            return 0., 0., 0.

        steer = self.yaw_control.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)

        brake = 0.
        throttle = 0.

        error_lin_vel = target_lin_vel - current_lin_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.previous_time
        self.previous_time = current_time

        throttle = self.throttle_pid.step(error_lin_vel, sample_time)

        if throttle < 0 :
            brake = abs(throttle) * self.brake_const
            throttle = 0.

        return throttle, brake, steer
