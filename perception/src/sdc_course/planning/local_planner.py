import numpy as np
import math
from typing import List, Dict, Tuple

import carla

from ..utils import Car
from ..utils.utility import (
    get_locally_sampled_trajectory,
    get_target_speed,
    get_velocity_ms,
)
from .global_planner import GlobalPlanner
from .behavior_planner import BehaviorPlanner


class LocalPlanner:
    def __init__(
        self,
        global_planner: GlobalPlanner,
        behavior_planner: BehaviorPlanner,
        global_path: List[Tuple[float, float, float]],
        params: Dict,
    ):
        self.global_planner = global_planner
        self.behavior_planner = behavior_planner
        self.global_path = global_path
        self.lookahead_time = params["planning"]["lookahead_time"]
        self.resolution = params["planning"]["local_resolution"]
        self.best_traj = []

    def update_global_path(self, new_global_path):
        self.global_path = new_global_path

    def get_velocity_profile(self, vehicle: Car) -> float:
        """Get speed limit based on global plan and current maneuever.
        : param vehicle: car instance
        """
        velocity = get_target_speed(vehicle, self.global_path)
        if self.behavior_planner.current_maneuever in ["junction", "overtaking"]:
            velocity = min(velocity, 30.0)
        return velocity

    def get_local_path(self, vehicle: Car, parked_cars: List[carla.Location]) -> List[np.array]:
        """Get local path for given vehicle state and detected parked cars.
        : param vehicle: car instance
        : param parked cars: list of parked cars
        """
        # Get current pose of vehicle
        vehicle_transform = vehicle.get_transform()
        current = [None] * 3
        current[0] = vehicle_transform.location.x
        current[1] = vehicle_transform.location.y
        current[2] = math.radians(vehicle_transform.rotation.yaw)

        # Start next local path at closest point on previous local path if possible
        if len(self.best_traj) == 0:
            start = current
        else:
            start = self._get_closest_waypoint(current, self.best_traj)

        # Sample global path to get reference
        max_len = max(5.0, get_velocity_ms(vehicle) * self.lookahead_time)
        min_distance = 1.0
        reference = get_locally_sampled_trajectory(vehicle, self.global_path, max_len, min_distance)

        if len(reference) <= 2:
            return [reference]

        # Determine waypoint in lookahead distance from global plan
        delta_x = reference[-1][0] - reference[-2][0]
        delta_y = reference[-1][1] - reference[-2][1]
        end = [None] * 3
        end[0] = reference[-1][0]
        end[1] = reference[-1][1]
        end[2] = np.arctan2(delta_y, delta_x)
        length = self._get_trajectory_length(reference) + self._compute_distance(
            start, reference[0]
        )

        # Get trajectory candidates and sort them based on their scores
        parameter_list_unscored = self._compute_parameters(length, start, end)
        parameter_list = self._score_and_sort(parameter_list_unscored, parked_cars, reference)

        # Sample from parameterized trajectories
        n_points = int(length / self.resolution)
        trajs = []
        for parameters in parameter_list:
            trajs.append(self._generate_trajectory(parameters, n_points))

        # Fallback if no viable parameters found
        if len(trajs) == 0:
            trajs = [reference]

        # Update best local trajectory for next planning step
        self.best_traj = trajs[0]

        return trajs

    def _get_trajectory_length(self, trajectory):
        length = 0
        for i in range(len(trajectory) - 1):
            length += self._compute_distance(trajectory[i + 1], trajectory[i])
        return length

    def _compute_distance(self, p1, p2):
        distance = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        return distance

    def _get_closest_waypoint(self, s, waypoints):
        closest_state = None
        min_distance = float("Inf")
        for i, wp in enumerate(waypoints):
            distance = math.sqrt((s[0] - wp[0]) ** 2 + (s[1] - wp[1]) ** 2)
            if distance < min_distance:
                if i == len(waypoints) - 1:
                    delta_x = wp[0] - waypoints[i - 1][0]
                    delta_y = wp[1] - waypoints[i - 1][1]
                    heading = np.arctan2(delta_y, delta_x)
                    closest_state = [wp[0], wp[1], heading]
                else:
                    delta_x = waypoints[i + 1][0] - wp[0]
                    delta_y = waypoints[i + 1][1] - wp[1]
                    heading = np.arctan2(delta_y, delta_x)
                    closest_state = [wp[0], wp[1], heading]
                min_distance = distance
        return closest_state

    def _score_and_sort(
        self,
        parameter_list_unscored: List[np.array],
        obstacles: List[carla.Location],
        reference: List[Tuple[float, float]],
    ) -> List[np.array]:
        """Score the trajectories and sort them
        : param parameter_list_unscored: List of parameter sets as np.arrays with size 2x4
        : param obstacles: List of obstacles as carla.locations
        : param reference: List of global path locations up to a predefined lookahead

        : return: List of parameter sets as np.arrays with size 2x4
        """
        parameter_list_scored = parameter_list_unscored
        #######################################################################
        ######################### TODO: IMPLEMENT THIS ########################
        #######################################################################
        # Solution start
        scores = []
        for parameters in parameter_list_unscored:
            traj = self._generate_trajectory(parameters, 20)
            collision_cost = self._get_collision_cost(traj, obstacles)
            tracking_cost = self._get_reference_error(traj, reference)

            cost = collision_cost + 0.01 * tracking_cost
            scores.append((parameters, cost))
        scores.sort(key=lambda tup: tup[1])
        parameter_list_scored = [el[0] for el in scores]
        # Solution end
        return parameter_list_scored

    def _get_collision_cost(self, traj, obstacles):
        cost = 0.0
        for p_t in traj:
            for loc_o in obstacles:
                p_o = [loc_o.x, loc_o.y]
                distance = self._compute_distance(p_t, p_o)
                if distance < 10.0:
                    cost += 1 / distance
        return cost

    def _get_reference_error(self, traj, ref):
        cost = 0.0
        for p_t in traj:
            next_ref = self._get_closest_waypoint(p_t, ref)
            cost += self._compute_distance(p_t, next_ref)
        return cost

    def _generate_trajectory(self, parameters, n_points):
        u = np.linspace(0, 1, n_points)
        U = np.zeros((n_points, 4))
        for i, u_i in enumerate(u):
            U[i, :] = np.asarray([u_i**3, u_i**2, u_i, 1])
        trajectory = U @ parameters.T
        return trajectory

    def _compute_parameters(
        self, length: float, start: Tuple[float, float, float], end: Tuple[float, float, float]
    ) -> List[np.array]:
        """Compute parameters for multiple parametric curves with varying parameters and endpoints
        : param length : length of global path up to lookahead
        : param start : start pose
        : param end : end pose

        : return: List of parameter sets as np.arrays with size 2x4
        """
        c_list = [0.9 * length, 1.5 * length]

        parameters = []
        for c0 in c_list:
            for c1 in c_list:
                parameter = self._fit_cubic(start, end, c0, c1)
                parameters += [parameter] if parameter is not None else []

                end_location = carla.Location(end[0], end[1], 0.0)
                wp = self.global_planner.dao.get_waypoint(end_location)

                if wp.right_lane_marking.lane_change & carla.LaneChange.Right:
                    wp_offset = wp.get_right_lane()  # Next waypoint on right lane
                elif wp.left_lane_marking.lane_change & carla.LaneChange.Left:
                    wp_offset = wp.get_left_lane()  # Next waypoint on left lane
                else:
                    wp_offset = None

                if (
                    wp_offset is not None
                    and wp_offset.lane_type == carla.LaneType.Driving
                    and wp.road_id == wp_offset.road_id
                ):
                    loc_offset = wp_offset.transform.location
                    end_offset = (loc_offset.x, loc_offset.y, end[2])
                    parameter = self._fit_cubic(start, end_offset, c0, c1)
                    parameters += [parameter] if parameter is not None else []

        return parameters

    def _fit_cubic(
        self,
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
        c0: float,
        c1: float,
    ):
        """Fit a cubic spline to start and end conditions
        : param start : start pose
        : param end : end pose
        : param c0 : scaling parameter
        : param c1 : scaling parameter

        : return: Parameters as np.array with size 2x4
        """
        parameters = None
        #######################################################################
        ######################### TODO: IMPLEMENT THIS ########################
        #######################################################################
        # Solution start
        A11 = np.array([[0, 0, 0, 1], [1, 1, 1, 1], [0, 0, 1, 0], [3, 2, 1, 0]])
        A1 = np.hstack((A11, np.zeros((4, 4))))
        A2 = np.hstack((np.zeros((4, 4)), A11))
        A = np.vstack((A1, A2))
        b = [
            start[0],
            end[0],
            c0 * math.cos(start[2]),
            c1 * math.cos(end[2]),
            start[1],
            end[1],
            c0 * math.sin(start[2]),
            c1 * math.sin(end[2]),
        ]

        res = np.linalg.solve(A, b)
        parameters = np.vstack((res[:4], res[4:]))
        # Solution end
        return parameters
