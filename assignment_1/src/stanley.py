import math
import numpy as np
from sdc_course.utils.utility import *


class StanleyLateralController:
    """
    StanleyLateralController implements lateral control using the stanley controller.
    """

    def __init__(self, vehicle, K_cte):
        self._vehicle = vehicle
        self._k_cte = K_cte

    def run_step(self, waypoints):
        return self._stanley_control(waypoints, self._vehicle.get_transform())

    def _get_heading_error(self, waypoints, ind_nearest, vehicle_yaw):
        waypoint_delta_x = waypoints[ind_nearest + 1][0] - waypoints[ind_nearest][0]
        waypoint_delta_y = waypoints[ind_nearest + 1][1] - waypoints[ind_nearest][1]
        waypoint_heading = np.arctan2(waypoint_delta_y, waypoint_delta_x)
        heading_error = ((waypoint_heading - vehicle_yaw) + np.pi) % (2 * np.pi) - np.pi
        return heading_error

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

    def _stanley_control(self, waypoints, vehicle_transform):
        """
        :param waypoint: list of waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT STANLEY CONTROL HERE ###############

        #get next waypoint
        nxt_wp = get_nearest_waypoint(self._vehicle, waypoints)
        # ind_nearest = nxt_wp[1]
        heading_error = self._get_heading_error(waypoints, nxt_wp[1], vehicle_transform.rotation.yaw)
        heading_error = heading_error * np.pi/180

        #get cross track error
        x_car = vehicle_transform.location.x
        y_car = vehicle_transform.location.y
        yaw_car = vehicle_transform.rotation.yaw
        yaw_car = yaw_car * np.pi/180

        velocity = self._vehicle.get_velocity()
        velocity_car = get_velocity_ms(self._vehicle)

        waypoint , ind_nearest = get_nearest_waypoint(self._vehicle, waypoints)
        #trajectory heading to nearest waypoint
        trajectory_heading =  tuple(waypoint)

        heading_error = self._get_heading_error(waypoints, ind_nearest, vehicle_transform.rotation.yaw)

        #get cross track error
        cte = self.get_cte_heading_error(velocity_car, trajectory_heading)
        v1 = [x_car - waypoint[0], y_car - waypoint[1]]
        v2 = [x_direction, y_direction]
        steering =( self._get_steering_direction(v1, v2))  * (cte *heading_error)

        #apply steering angle
        # if steering > 0.0:
        #     steering = min(steering, 0.3)
        # else:
        #     steering = max(steering, -0.3)
        
        #######################################################################
        return steering
