from typing import List, Tuple
from ..utils.utility import (
    interpolate_waypoints_linear,
    get_locally_sampled_trajectory,
    get_target_speed,
)


class SimpleLocalPlanner:
    """Simple local planner that simply linearly interpolate given waypoints."""

    def __init__(self, global_path: List[Tuple[float, float, float]]):
        self.global_path = global_path
        self.global_path_interpolated, _ = interpolate_waypoints_linear(global_path)

    def get_velocity_profile(self, vehicle):
        return get_target_speed(vehicle, self.global_path_interpolated)

    def get_local_path(self, vehicle, max_len, min_distance):
        local_path = get_locally_sampled_trajectory(
            vehicle, self.global_path_interpolated, max_len, min_distance
        )
        return local_path

    def update_global_path(self, global_path):
        self.global_path = global_path
        self.global_path_interpolated, _ = interpolate_waypoints_linear(global_path)
