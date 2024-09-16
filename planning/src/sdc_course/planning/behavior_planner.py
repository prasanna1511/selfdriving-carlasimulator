from .global_planner import GlobalPlanner
from ..utils import Car
from typing import List
import carla


class BehaviorPlanner:
    """Behavior planner provides local adaptions depending on the current car or world state."""

    def __init__(self, global_planner: GlobalPlanner):
        self.global_planner = global_planner
        self.current_maneuever = "default"

    def update(self, vehicle: Car, parked_cars: List[carla.Location]):
        """update behavior planner with current car state.
        :param vehicle: current car state.
        """

        wp = self.global_planner.dao.get_waypoint(vehicle.get_transform().location)
        if parked_cars:
            self.current_maneuever = "overtaking"
        elif wp.is_junction:
            self.current_maneuever = "junction"
        else:
            self.current_maneuever = "default"
