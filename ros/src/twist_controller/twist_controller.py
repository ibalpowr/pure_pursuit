from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

import time
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# each can be separate

class Controller(object):
    def __init__(self, 
                 decel_limit,
                 accel_limit,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        
                 
        self.throttle_pid = PID()
        self.yaw_control = YawController()
        # wild guesses from 50Hz
        self.filter = LowPassFilter(0.2, 0.1)
    
        self.previous_time = None

    def control(self, target_lin_vel, target_ang_vel, current_lin_vel, dbw_enabled):
        if self.previous_time is None or not dbw_enabled:
            self.previous_time = rospy.get_time()
            return 0, 0, 0
        
        # should be close to 0.02 sec ... because of 50Hz
        sample_time = rospy.get_time() - self.previous_time
        
        error_lin_vel = target_lin_vel - current_lin_vel
        
        # impose upper and lower acceleration limits
        error_lin_vel = max(self.decel_limit*sample_time, 
                        min(self.accel_limit*sample_time, error_lin_vel))
        
        throttle = self.throttle_pid.step(error_lin_vel, sample_time)
        
        # impose uppper and lower throttle limits
        throttle = max(0.0, min(1.0, throttle))
        
        # for brake ... just proportional
        if error_lin_vel < 0:
            brake = -10.0 * error_lin_vel
            brake = max(brake, 1.0)
            throttle = 0.0    # no brake and throttle in the same time
        else:
            brake = 0.0
            
        # now for steering
        steer = self.yaw_control.get_steering(target_lin_vel,
                                              target_ang_vel,
                                              current_lin_vel)
        steer - self.filter.filt(steer)
        self.previous_time = time.time()     
    
        
        return throttle, brake, steer
