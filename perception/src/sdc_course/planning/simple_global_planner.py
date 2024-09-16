from typing import Dict
from ..utils.utility import load_waypoints_from_txt


class SimpleGlobalPlanner:
    """
    Simple global planner reads waypoints from a file that was precomputed/recorded in advance.
    """

    def __init__(self, params: Dict):
        self._waypoints = load_waypoints_from_txt(params["waypoints_file"])

    def get_global_path(self):
        return self._waypoints
