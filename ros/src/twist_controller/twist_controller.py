from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # TODO: Implement
        min_speed = 0.1

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        steer_kp = 0.4
        steer_ki = 0.1
        steer_kd = 0.005
#        steer_kp = 1.15
#        steer_ki = 0.001
#        steer_kd = 0.1
        min_steer = -max_steer_angle
        max_steer = max_steer_angle
        self.steering_controller = PID(steer_kp, steer_ki, steer_kd, min_steer, max_steer)

        throttle_kp = 0.3
        throttle_ki = 0.1
        throttle_kd = 0.005
        #throttle_kp = 0.8
        #throttle_ki = 0.002
        #throttle_kd = 0.3
        min_throttle = 0.0 # Minimum throttle value
        max_throttle = 0.2 # Maximum throttle value (should be OK for simulation, not for Carla!)
        self.throttle_controller = PID(throttle_kp, throttle_ki, throttle_kd, min_throttle, max_throttle)

        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = .02 # Sample time (50 Hz)
        self.velocity_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, current_angular_vel, dbw_enabled, target_linear_vel, target_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.steering_controller.reset()
            return 0., 0., 0.

        current_vel = self.velocity_lpf.filt(current_vel)
        vel_error = target_linear_vel - current_vel
        angular_vel_error = target_angular_vel - current_angular_vel

        estimated_steering = self.yaw_controller.get_steering(target_linear_vel,
                                                              target_angular_vel,
                                                              current_vel)
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        steering_correction = self.steering_controller.step(angular_vel_error, sample_time)
        steering = estimated_steering + steering_correction
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if target_linear_vel == 0. and current_vel < 0.1:  # Car is willing to stop and drives slowly
            throttle = 0
            brake = 400  # to hold the car in place if we are stopped at a light. Acceleration ~ 1m/s^2

        elif vel_error < 0.:  # Car drives faster than we want
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        return throttle, brake, steering
