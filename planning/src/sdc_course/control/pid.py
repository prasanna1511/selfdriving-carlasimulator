from collections import deque
import math
import numpy as np
from sdc_course.utils.utility import *


class PIDLongitudinalController:
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, dt, K_P, K_D, K_I):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param dt: time differential in seconds
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """

        self._vehicle = vehicle
        self._dt = dt
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_velocity_ms, debug=False):
        """
        Execute one step of longitudinal control to reach a given target velocity.

        :param target_velocity_ms: target velocity in m/s
        :param debug: boolean for debugging
        :return: throttle control
        """
        current_velocity_ms = get_velocity_ms(self._vehicle)

        if debug:
            print("Current velocity = {}".format(current_velocity_ms))

        return self._pid_control(target_velocity_ms, current_velocity_ms)

    def _pid_control(self, target_velocity_ms, current_velocity_ms):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

        :param target_velocity_ms:  target velocity in m/s
        :param current_velocity_ms: current velocity of the vehicle in m/s
        :return: throttle/brake control
        """
        acceleration = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT LONGITUDINAL PID CONTROL HERE ######
        #######################################################################
        # Solution start
        # Add error to buffer
        error = target_velocity_ms - current_velocity_ms
        self._error_buffer.append(error)

        p = self._k_p * error
        i = self._k_i * sum(self._error_buffer) * self._dt

        # Check if buffer has minimum length for D, otherwise set to 0
        if len(self._error_buffer) > 1:
            d = self._k_d * (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
        else:
            d = 0.0

        acceleration = p + i + d

        # Solution end
        return acceleration


class PIDLateralController:
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, dt, K_P, K_D, K_I):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param dt: time differential in seconds
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._vehicle = vehicle
        self._dt = dt
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._error_buffer = deque(maxlen=10)

    def run_step(self, waypoints):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control
        """
        return self._pid_control(waypoints, self._vehicle.get_transform())

    def _get_steering_direction(self, v1, v2):
        """
        Note that Carla uses a left hand coordinate system, this is why a positive
        cross product requires a negative steering direction.
        :param v1: vector between vehicle and waypoint
        :param v2: vector in direction of vehicle
        :return: steering direction
        """
        cross_prod = v1[0] * v2[1] - v1[1] * v2[0]
        if cross_prod >= 0:
            return -1
        return 1

    def _pid_control(self, waypoints, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoints: local waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        ######################################################################
        ################## TODO: IMPLEMENT LATERAL PID CONTROL HERE ###########
        #######################################################################
        # Solution start
        nearest_waypoint, index = get_nearest_waypoint(self._vehicle, waypoints)

        # Compute cross-track error
        error_delta_x = nearest_waypoint[0] - vehicle_transform.location.x
        error_delta_y = nearest_waypoint[1] - vehicle_transform.location.y
        distance = math.sqrt(error_delta_x**2 + error_delta_y**2)

        # Compute steering direction
        x = vehicle_transform.location.x
        y = vehicle_transform.location.y
        yaw = math.radians(vehicle_transform.rotation.yaw)
        v1 = [nearest_waypoint[0] - x, nearest_waypoint[1] - y]
        v2 = [np.cos(yaw), np.sin(yaw)]
        direction = self._get_steering_direction(v1, v2)

        # Add error to buffer
        error = direction * distance
        self._error_buffer.append(error)

        # PID terms
        p = self._k_p * error
        i = self._k_i * sum(self._error_buffer) * self._dt

        if len(self._error_buffer) > 1:
            d = self._k_d * (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
        else:
            d = 0.0

        steering = p + i + d

        # Solution end
        return steering
