from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

import time
import rospy

# may03thur2018jz0309
# density = mass / volume
# 0.755g/mL * 1kg/1000g * 3785mL/gal = 2.858 kg/gal
GAS_DENSITY = 2.858    # [kg/gal]
ONE_MPH = 0.44704
# may03thur2018jz0308
# no need for that
#ONE_GALLON = 0.00379

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
        # may02wed2018jz0440
        # may02wed2018jz1147
        #self.lpf_steer = LowPassFilter(0.2, 0.1)
        # from walkthru video
        self.lpf = LowPassFilter(0.5, 0.02)
        
        min_speed = 0.1 # m/s
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # Throttle PID parameters
        #kp = 1
        #ki = 0.04
        #kd = 0.01
        # may01tue2018jz0853
        #kp = 1
        #ki = 0
        #kd = 0
        # may01tue2018jz0910
        #kp = 0.5
        #ki = 0
        #kd = 0
        #mn = decel_limit
        #mx = accel_limit
        # may02wed2018jz1150
        # from walkthru video
        #kp = 0.3
        #ki = 0.1
        #kd = 0
        # may03thur2018jz0623
        #kp = 0.5
        #ki = 0.1
        #kd = 0
        # may03thur2018jz0835
        kp = 0.5
        ki = 0.01
        kd = 0
        mn = 0   # min throttle value
        # may03thur2018jz0519
        # throttle = 0.2 roughly corresponding to 40 kmph
        mx = 0.2   # max throttle value
        self.throttle_pid = PID(kp, ki, kd, mn, mx)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

        # may03thur2018jz0310
        #self.total_mass = self.vehicle_mass + self.fuel_capacity*ONE_GALLON*GAS_DENSITY
        # [kg] = [kg] + [gal] * [kg/gal]
        self.total_mass = self.vehicle_mass + self.fuel_capacity*GAS_DENSITY
        # may03thur2018jz0312
        #self.brake_const = self.total_mass * self.wheel_radius
        # [Nm] = (kg * m/s^2) * m = N * m

        self.previous_time = rospy.get_time()


    def control(self, target_lin_vel, target_ang_vel, current_lin_vel, dbw_enabled):
        if self.previous_time is None or not dbw_enabled:
            self.previous_time = rospy.get_time()
            self.throttle_pid.reset()
            return 0., 0., 0.

        steer = self.yaw_control.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)
        
        # may02wed2018jz0442
        #steer = self.lpf_steer(steer)
        # may02wed2018jz0615
        # may03wed2018jz1147
        #steer = self.lpf_steer.filt(steer)

        brake = 0.
        throttle = 0.
        
        # may03wed2018jz1146
        # from walkthru video
        current_lin_vel = self.lpf.filt(current_lin_vel)

        error_lin_vel = target_lin_vel - current_lin_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.previous_time
        # may03thur2018jz0331
        # another idea is
        # sample_time = 1/50   # 50Hz is the sample rate for dbw
        self.previous_time = current_time

        throttle = self.throttle_pid.step(error_lin_vel, sample_time)
                
        # may03thur2018jz0347
        # from walkthru video
        #if throttle < 0 :
        #    brake = abs(throttle) * self.brake_const
        #    throttle = 0.
        if target_lin_vel == 0. and current_lin_vel < 0.1:
            throttle = 0
            brake = 400   # [Nm]
        elif throttle < 0.1 and error_lin_vel < 0:
            throttle = 0
            decel = max(error_lin_vel, self.decel_limit)
            brake = self.total_mass * abs(decel) * self.wheel_radius

        return throttle, brake, steer
