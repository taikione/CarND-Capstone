import rospy
from pid import PID
# from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
VEHICLE_MIN_SPEED = 0
MAX_STEER = 1.0
DT = 0.2


class Controller(object):
    def __init__(self, steer_ratio, wheel_base, max_let_accel, max_steer_angle,
                 decel_limit, accel_limit):

        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_let_accel = max_let_accel
        self.max_steer_angle = max_steer_angle
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit

        # self.yaw_controller = YawController(self.wheel_base, self.steer_ratio,
        #                                     VEHICLE_MIN_SPEED, self.max_let_accel, self.max_steer_angle)

        self.steer_pid = PID(0.07, 0.0005, 5, -self.max_steer_angle, self.max_steer_angle)
        self.throttle_pid = PID(1, 0.1, 0.1, self.decel_limit, self.accel_limit)
        self.brake_pid = PID(1, 0.1, 0.1, self.decel_limit, self.accel_limit)
        self.last_velocity_error = 0

    def control(self, target_linear_velocity, target_angular_velocity,
                current_linear_velocity, dbw_status, cte):
        '''Defines target throttle, brake and steering values'''

        if dbw_status:
            velocity_error = target_linear_velocity - current_linear_velocity

            if self.is_change_acc(velocity_error):
                self.throttle_pid.reset()
                self.brake_pid.reset()

            # TODO: implement throttle controller
            if velocity_error >= 0:
                throttle = self.throttle_pid.step(velocity_error, DT)
                brake = 0

            # TODO: implement brake controller
            else:
                throttle = 0
                brake = self.brake_pid.step(-velocity_error, DT)

            # implement steering controller
            # steering = self.yaw_controller.get_steering(target_linear_velocity,
            #                                             target_angular_velocity,
            #                                             current_linear_velocity)
            # steering = self.steer_pid.step(steering, DT)
            steering = self.steer_pid.step(cte, DT)

            self.last_velocity_error = velocity_error

        else:
            throttle = 0
            brake = 0
            steering = 0
            rospy.loginfo("dbw_status false")

        return throttle, brake, steering

    def is_change_acc(self, velocity_error):

        is_switch_brake = (self.last_velocity_error >= 0) and (velocity_error < 0)
        is_switch_acc = (self.last_velocity_error <= 0) and (velocity_error > 0)

        return is_switch_brake or is_switch_acc
