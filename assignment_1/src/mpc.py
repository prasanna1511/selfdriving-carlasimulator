from typing import Union
import math
import numpy as np
import carla

from pyilqr.pyilqr.costs import CompositeCost, QuadraticCost
from pyilqr.pyilqr.example_costs import SetpointTrackingCost, PolylineTrackingCost, Polyline
from pyilqr.pyilqr.example_dynamics import UnicycleDynamics
from pyilqr.pyilqr.ocp import OptimalControlProblem
from pyilqr.pyilqr.ilqr import ILQRSolver
from pyilqr.pyilqr.strategies import AbstractStrategy, OpenLoopStrategy, FunctionStrategy

from sdc_course.utils.car import Car
from sdc_course.utils.utility import *
from sdc_course.control.controller import AbstractController


class ModelPredictiveController(AbstractController):
    """
    Receding horizon controller using an iterative LQR solver. At each iteration, the nonlinear optimal control problem is solved with local linear-quadratic approximations.
    """

    def __init__(self, vehicle: Car, params):
        """
        Constructor method.
        :param vehicle: actor to apply to local planner logic onto
        :param params: All the controller related parameters
        """
        super().__init__(vehicle, params)
        self.dynamics = UnicycleDynamics(self.dt)
        self.prediction_horizon = 20
        self._initial_strategy = None
        self.reset_initial_strategy(self._initial_strategy)

    def reset_initial_strategy(self, initial_strategy: Union[AbstractStrategy, None]):
        if initial_strategy is None:
            initial_strategy = FunctionStrategy(lambda x, t: np.zeros(self.dynamics.dims[1]))
        object.__setattr__(self, "_initial_strategy", initial_strategy)

    def compute_steering_and_acceleration(self, target_velocity_ms, waypoints):
        """
        Computes steering and acceleration
        :param target_velocity_ms: desired vehicle velocity in m/s
        :param waypoints: local trajectory waypoints
        :return: control command for the vehicle.
        """
        steering = 0.0
        acceleration = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT MPC CONTROL HERE ###################

        #getvehicles location by local planner
        x_car = self._vehicle.get_transform().location.x
        y_car = self._vehicle.get_transform().location.y
        yaw_car = self._vehicle.get_transform().rotation.yaw
        #get current velocity
        current_velocity_ms = get_velocity_ms(self._vehicle)
        #get goal waypoint
        goal_waypoint_index = self._get_goal_waypoint_index(self._vehicle, waypoints, self._ld)
        goal_waypoint = waypoints[goal_waypoint_index]

        #get cross track error
                
        # ipdb.set_trace()
        #######################################################################
        return steering, acceleration
