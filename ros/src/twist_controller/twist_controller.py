from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
ONE_GALLON = 0.00379

# TODO: Tune PID constants
K_p, K_i, K_d = 1, 0.001, 0.1 # Throttle P I D constants

class Controller(object):
    def __init__(self, , *args, **kwargs):
        # Input parameters
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.wheel_radius = kwargs["wheel_radius"]
        self.brake_deadband = kwargs["brake_deadband"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]

        self.wheel_base = kwargs["wheel_base"]
        self.steer_ratio = kwargs["steer_ratio"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.max_steer_angle = kwargs["max_steer_angle"]

        self.min_speed = 5 * ONE_MPH # 5 MPH to meter per seconds

        self.sample_time = 1./50

        # Torque = Force * radius  = mass * acceleration * radius
        # Torque = brake_const * acceleration
        # Looks like fuel_capacity is in gallons, so we transform to metric units
        self.total_mass = self.vehicle_mass + self.fuel_capacity*ONE_GALLON*GAS_DENSITY
        self.brake_const = self.total_mass * self.wheel_radius

        # Initialize different controllers
        self.speed_controller = PID(K_p, K_i, K_d, self.decel_limit, self.accel_limit)
        self.steer_controller = YawController(self.wheel_base, self.steer_ratio,
                                              self.min_speed, self.max_lat_accel,
                                              self.max_steer_angle)

    def reset(self):
        self.speed_controller.reset()

    def control(self, current_lin_vel, target_lin_vel, target_ang_vel):

        throttle = self.steer_controller.get_steering(current_lin_vel,
                                                      target_lin_vel,
                                                      target_ang_vel)
        # TODO: do we need to filter (measurement) error value?
        vel_error = target_velocity - current_velocity

        control = self.speed_controller.step(vel_error, self.sample_time)

        if control > 0.:
            brake = 0.
            throttle = control/self.accel_limit
        else:
            brake, throttle = 0., 0.
            if -control > self.brake_deadband:
                brake = self.brake_const*control

        return throttle, brake, steering
