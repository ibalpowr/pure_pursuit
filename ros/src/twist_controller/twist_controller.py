from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

import time
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
ONE_GALLON = 0.00379

# each can be separate

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

		self.yaw_control = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

		# PID inputs
		kp = 0.5
		ki = 0.01
		kd = 0.01
		mn = decel_limit
		mx = accel_limit
		self.throttle_pid = PID(kp, ki, kd, mn, mx)

		tau = 0.2
		ts = 0.1
		self.low_pass_filter = LowPassFilter(tau, ts)

		self.vehicle_mass = vehicle_mass
		self.fuel_capacity = fuel_capacity
		self.brake_deadband = brake_deadband
		self.wheel_radius = wheel_radius
		self.accel_limit = accel_limit
		self.decel_limit = decel_limit

		#self.total_mass = self.vehicle_mass + self.fuel_capacity*ONE_GALLON * GAS_DENSITY
		#self.brake_const = self.total_mass * self.wheel_radius

		self.previous_time = rospy.get_time()


	def control(self, target_lin_vel, target_ang_vel, current_lin_vel, dbw_enabled):
		if self.previous_time is None or not dbw_enabled:
		    self.previous_time = rospy.get_time()
		    self.throttle_pid.reset()
		    return 0, 0, 0
		
		# now for steering
		steer = self.yaw_control.get_steering(target_lin_vel, target_ang_vel, current_lin_vel)
		brake = 0.
		throttle = 0.

		error_lin_vel = target_lin_vel - current_lin_vel

		# should be close to 0.02 sec ... because of 50Hz
		current_time = rospy.get_time()
		sample_time = current_time - self.previous_time
		self.previous_time = current_time


		# should be close to 0.02 sec ... because of 50Hz
		#sample_time = rospy.get_time() - self.previous_time
		
		#error_lin_vel = target_lin_vel - current_lin_vel
		
		# impose upper and lower acceleration limits
		#error_lin_vel = max(self.decel_limit*sample_time, 
		 #               min(self.accel_limit*sample_time, error_lin_vel))
		
		acceleration = self.throttle_pid.step(error_lin_vel, sample_time)

		if acceleration > 0 : 
			throttle = acceleration
		else :
			brake = abs(acceleration) * self.vehicle_mass * self.wheel_radius

		
		# impose uppper and lower throttle limits
		#throttle = max(0.0, min(1.0, throttle))
		
		# for brake ... just proportional
		#if error_lin_vel < 0:
		#    brake = -10.0 * error_lin_vel
		#    brake = max(brake, 1.0)
		#    throttle = 0.0    # no brake and throttle in the same time
		#else:
		#    brake = 0.0
		    
		# now for steering
		#steer = self.yaw_control.get_steering(target_lin_vel,
		#                                      target_ang_vel,
		#                                      current_lin_vel)
		#steer - self.filter.filt(steer)
		#self.previous_time = time.time()     
	    
		
		return throttle, brake, steer
