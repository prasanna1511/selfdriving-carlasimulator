from typing import Dict, List, Tuple
import numpy as np
import carla
import pickle

from .global_route_planner_dao import GlobalRoutePlannerDAO
from .graph import Graph
from .a_star import a_star_search

import matplotlib.pyplot as plt


class GlobalPlanner:
    """Dynamic global planner that returns a global path from a given start to a given end position."""

    def __init__(self, world_map: carla.Map, params: Dict):
        """
        : param world_map : map representation from carla (carla.Map) containing the waypoints.
        : param params : parameter dictionary containing configuration parameters.
        """
        self.params = params
        self.dao = GlobalRoutePlannerDAO(world_map, self.params["planning"]["hop_resolution"])
        self.topology = None
        self.graph = None
        self.meta_to_edge = dict()
        self.xyz_to_node = dict()
        self.node_to_wp = dict()

        self.last_update_location = None
        self.last_edge = None
        self.last_adjacent_edges = None

        self._setup()

    def get_global_path(
        self, start: carla.Location, end: carla.Location
    ) -> List[Tuple[float, float, float]]:
        """plan global path from start location to end location. It will plan a route from the closest node in the graph to the closest node to the end position.

        : param start : start position (carla.Location) in world coordinates.
        : param end : end position in world (carla.Location) coordinates.

        : return : list of tuples (x, y, velocity) of the global path.
        """
        closest_to_start = self._find_closest_node(start)
        closest_to_end = self._find_closest_node(end)

        node_to_xy = dict(
            (node, (wp.transform.location.x, wp.transform.location.y))
            for node, wp in self.node_to_wp.items()
        )

        node_path = a_star_search(self.graph, node_to_xy, closest_to_start, closest_to_end)

        if node_path:
            global_path = self._get_path_from_nodes(node_path)
        else:
            print("A* is not implemented, using default path!")
            global_path = pickle.load(open("./src/sdc_course/planning/global_path.p", "rb"))

        return global_path

    def _find_closest_node(self, query: carla.Location) -> int:
        if len(self.graph.nodes) == 0:
            return None

        closest_node = None
        min_distance = None

        for node in self.graph.nodes:
            distance = self.node_to_wp[node].transform.location.distance(query)
            if closest_node is None or distance < min_distance:
                min_distance = distance
                closest_node = node

        return closest_node

    def _find_closest_edge_nodes(self, query: carla.Location) -> Tuple[int, int]:
        """
        Finds closest start and end edge nodes and returns them
        """
        first_node = self._find_closest_node(query)
        second_node = None
        first_is_start = None

        min_distance = None
        # Check segments with first node as start node
        for node in self.graph.nodes[first_node]:
            for wp in self.graph.get_path(first_node, node):
                distance = wp.transform.location.distance(query)
                if second_node is None or distance < min_distance:
                    min_distance = distance
                    second_node = node
                    first_is_start = True

        # Check segments with first node as end node
        for node in self.graph.nodes:
            if first_node in self.graph.nodes[node]:
                for wp in self.graph.get_path(node, first_node):
                    distance = wp.transform.location.distance(query)
                    if second_node is None or distance < min_distance:
                        min_distance = distance
                        second_node = node
                        first_is_start = False

        if first_is_start:
            return first_node, second_node
        else:
            return second_node, first_node

    def _get_adjacent_edges(self, from_node: int, to_node: int) -> Dict[str, List[Tuple[int, int]]]:
        """
        Find out-going edges that start at given to node and determine if the graph edge corresponds to a left or right turn.

        : param from_node : node id of the starting point of the corresponding edge.
        : param to_node : node id of the end point of the corresponding edge.

        : return: dictionary with "left", "right", and "straight" keys corresponding to lists of edges as pairs of
          (from, to)-node ids.
        """

        query_yaw = self.node_to_wp[to_node].transform.rotation.yaw

        threshold = 45

        adjacent_edges = {"left": [], "straight": [], "right": []}
        adjacent_nodes = self.graph.nodes[to_node]

        for adjacent_node in adjacent_nodes:
            adjacent_yaw = self.node_to_wp[adjacent_node].transform.rotation.yaw
            diff = self.normalize_angle(adjacent_yaw - query_yaw)

            if diff < -threshold:
                adjacent_edges["left"].append((to_node, adjacent_node))
            elif threshold < diff:
                adjacent_edges["right"].append((to_node, adjacent_node))
            else:
                adjacent_edges["straight"].append((to_node, adjacent_node))

        return adjacent_edges

    def normalize_angle(self, theta):
        """normalize angle to be in [-180, 180]"""
        while theta < -180:
            theta = theta + 360
        while theta > 180:
            theta = theta - 360

        return theta

    def integrate_traffic_sign(self, traffic_sign_position: np.array, traffic_sign_type: str):
        """add traffic sign type at specified world position.

        : param traffic_sign_position : location of the traffic sign
        : param traffic_sign_type : type of traffic sign in {"left_turn", "right_turn", "max_speed30", "max_speed50", "max_speed60"}

        """
        print(f"integrate  {traffic_sign_type} into global plan!")

        self.last_update_location = traffic_sign_position
        speed_limit = {"max_speed30": 30, "max_speed50": 50, "max_speed60": 60}

        location = carla.Location(traffic_sign_position[0], traffic_sign_position[1])

        #######################################################################
        ######################### TODO: IMPLEMENT THIS ########################
        #######################################################################
        ## Use the traffic sign information to modify the graph appropriately.
        ## Hint: There are new methods available: _find_closest_edge_nodes, _get_adjacent_edges.

    def _update_edge_cost(self, from_node, to_node, new_cost):
        self.graph.edges[(from_node, to_node)]["cost"] = new_cost

    def _update_speed_limit(self, from_node, to_node, new_speed_limit):
        self.graph.edges[(from_node, to_node)]["speed_limit"] = new_speed_limit

    def _setup(self):
        """
        Get topology from server data and build initial graph representation of the world
        """
        self.topology = self.dao.get_topology()
        (self.graph, self.meta_to_edge, self.xyz_to_node, self.node_to_wp) = self._build_graph()
        self._add_lane_changes_to_graph()

    def _get_cost(self, path):
        """
        Estimates cost-to-go from waypoints along path
        """
        acc_cost = 0
        for i in range(1, len(path)):
            acc_cost += path[i].transform.location.distance(path[i - 1].transform.location)
        return acc_cost

    def _build_graph(self):
        graph = Graph()
        meta_to_edge = dict()
        xyz_to_node = dict()
        node_to_wp = dict()

        # Iterate over segments containing start, end and path in between
        for segment in self.topology:
            entry_wp, exit_wp = segment["entry"], segment["exit"]
            entry_xyz, exit_xyz = segment["entryxyz"], segment["exitxyz"]
            path = segment["path"]
            road_id, section_id, lane_id = (entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id)

            # Add nodes to graph
            for xyz in entry_xyz, exit_xyz:
                if xyz not in xyz_to_node:
                    next_node = len(xyz_to_node)
                    xyz_to_node[xyz] = next_node
                    graph.add_node(next_node)
            node_to_wp[xyz_to_node[entry_xyz]] = entry_wp
            node_to_wp[xyz_to_node[exit_xyz]] = exit_wp

            n1 = xyz_to_node[entry_xyz]
            n2 = xyz_to_node[exit_xyz]

            # Add edges between nodes
            cost = self._get_cost(path)
            graph.add_edge(
                n1,
                n2,
                entry_xyz,
                exit_xyz,
                entry_wp,
                exit_wp,
                cost,
                path,
                self.params["planning"]["speed_limit"],
            )

            # Add to mapping from road, section and lane to corresponding edge nodes
            if road_id not in meta_to_edge:
                meta_to_edge[road_id] = dict()
            if section_id not in meta_to_edge[road_id]:
                meta_to_edge[road_id][section_id] = dict()
            meta_to_edge[road_id][section_id][lane_id] = (n1, n2)

        return graph, meta_to_edge, xyz_to_node, node_to_wp

    def _add_lane_changes_to_graph(self):
        for segment in self.topology:
            left_found, right_found = False, False

            segment_entry_wp = segment["entry"]
            segment_entry_node = self.xyz_to_node[segment["entryxyz"]]
            segment_path = segment["path"]

            # RIGHT lange change
            for wp in segment_path:
                if not segment_entry_wp.is_junction:
                    next_wp, neighboring_nodes = None, None
                    if (
                        wp.right_lane_marking.lane_change & carla.LaneChange.Right
                        and not right_found
                    ):
                        next_wp = wp.get_right_lane()  # Next waypoint on right lane
                        if (
                            next_wp is not None
                            and next_wp.lane_type == carla.LaneType.Driving
                            and wp.road_id == next_wp.road_id
                        ):
                            try:
                                closest_wp = self.dao.get_waypoint(next_wp.transform.location)
                                neighboring_nodes = self.meta_to_edge[closest_wp.road_id][
                                    closest_wp.section_id
                                ][closest_wp.lane_id]
                            except KeyError:
                                print("Failed to localize!")
                                neighboring_nodes = None

                            if neighboring_nodes is not None:
                                neighboring_path = self.graph.edges[neighboring_nodes]["path"]
                                neighboring_exit_xyz = self.graph.edges[neighboring_nodes]["to_xyz"]
                                neighboring_exit_wp = self.graph.edges[neighboring_nodes]["to_wp"]

                                path = [segment_path[0]]
                                current_wp_index = 1
                                while path[-1] != wp:
                                    path.append(segment_path[current_wp_index])
                                    current_wp_index += 1
                                path.append(wp)
                                (
                                    current_wp_index,
                                    closest_wp_on_path,
                                ) = self._get_closest_wp_on_path(
                                    closest_wp, neighboring_path, neighboring_exit_wp
                                )
                                path.append(closest_wp_on_path)
                                if neighboring_path:
                                    for i in range(current_wp_index, len(neighboring_path)):
                                        path.append(neighboring_path[i])
                                self.graph.add_edge(
                                    segment_entry_node,
                                    neighboring_nodes[1],
                                    segment["entryxyz"],
                                    neighboring_exit_xyz,
                                    segment_entry_wp,
                                    neighboring_exit_wp,
                                    self._get_cost(path),
                                    path,
                                    self.params["planning"]["speed_limit"],
                                )
                                right_found = True

            # LEFT lange change
            for wp in segment_path:
                if not segment_entry_wp.is_junction:
                    next_wp, neighboring_nodes = None, None
                    if wp.left_lane_marking.lane_change & carla.LaneChange.Left and not left_found:
                        next_wp = wp.get_left_lane()  # Next waypoint on left lane
                        if (
                            next_wp is not None
                            and next_wp.lane_type == carla.LaneType.Driving
                            and wp.road_id == next_wp.road_id
                        ):
                            try:
                                closest_wp = self.dao.get_waypoint(next_wp.transform.location)
                                neighboring_nodes = self.meta_to_edge[closest_wp.road_id][
                                    closest_wp.section_id
                                ][closest_wp.lane_id]
                            except KeyError:
                                print("Failed to localize!")
                                neighboring_nodes = True

                            if neighboring_nodes is not None:
                                neighboring_path = self.graph.edges[neighboring_nodes]["path"]
                                neighboring_exit_xyz = self.graph.edges[neighboring_nodes]["to_xyz"]
                                neighboring_exit_wp = self.graph.edges[neighboring_nodes]["to_wp"]

                                path = [segment_path[0]]
                                current_wp_index = 1
                                while path[-1] != wp:
                                    path.append(segment_path[current_wp_index])
                                    current_wp_index += 1
                                path.append(wp)
                                (
                                    current_wp_index,
                                    closest_wp_on_path,
                                ) = self._get_closest_wp_on_path(
                                    closest_wp, neighboring_path, neighboring_exit_wp
                                )
                                path.append(closest_wp_on_path)
                                if neighboring_path:
                                    for i in range(current_wp_index, len(neighboring_path)):
                                        path.append(neighboring_path[i])

                                self.graph.add_edge(
                                    segment_entry_node,
                                    neighboring_nodes[1],
                                    segment["entryxyz"],
                                    neighboring_exit_xyz,
                                    segment_entry_wp,
                                    neighboring_exit_wp,
                                    self._get_cost(path),
                                    path,
                                    self.params["planning"]["speed_limit"],
                                )
                                left_found = True

            if right_found and left_found:
                break

    def _get_closest_wp_on_path(self, wp, path, neighboring_exit_wp):
        closest_wp = neighboring_exit_wp
        closest_wp_index = None
        min_dist = float("Inf")
        for index, wp_path in enumerate(path):
            dist = wp_path.transform.location.distance(wp.transform.location)
            if dist < min_dist:
                min_dist = dist
                closest_wp = wp_path
                closest_wp_index = index
        return closest_wp_index, closest_wp

    def _get_path_from_nodes(self, node_path: List[int]) -> List[Tuple[float, float, float]]:
        """convert a list of nodes to a list of tuples with (x,y,velocity)"""
        if not node_path:
            return []

        wp_list = []
        vel_list = []
        from_node = node_path[0]  # Get first node

        for to_node in node_path[1:]:
            speed_limit = self.graph.edges[(from_node, to_node)]["speed_limit"]
            vel_list.append(speed_limit)

            wp_list.append(self.node_to_wp[from_node])
            for intermediate_wp in self.graph.get_path(from_node, to_node):
                wp_list.append(intermediate_wp)
                vel_list.append(speed_limit)

            from_node = to_node

        global_path = []

        for wp, velocity in zip(wp_list, vel_list):
            global_path.append([wp.transform.location.x, wp.transform.location.y, velocity])

        return global_path

    def _draw_edge(self, ax, from_node: int, to_node: int, color: Tuple[float, float, float]):
        if (from_node, to_node) not in self.graph.edges:
            return

        edge = self.graph.edges[(from_node, to_node)]

        x_coords = [wp.transform.location.x for wp in edge["path"]]
        y_coords = [-wp.transform.location.y for wp in edge["path"]]

        ax.plot(x_coords, y_coords, c=color)

    def plot(self, filename: str):
        """generate matplotlib plot with costs and speed_limits."""

        fig, (ax_cost, ax_limit) = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))

        cmap = plt.get_cmap("viridis")

        for (from_node, to_node), edge in self.graph.edges.items():
            cost = edge["cost"]
            speed_limit = int(edge["speed_limit"])

            self._draw_edge(ax_cost, from_node, to_node, cmap(min(cost / 1000, 1000)))

            color = (1, 0, 0)
            if speed_limit == 50:
                color = (0, 1, 0)
            if speed_limit == 60:
                color = (0, 0, 1)

            self._draw_edge(ax_limit, from_node, to_node, color)
        from matplotlib.lines import Line2D

        ax_limit.legend(
            (
                Line2D([0, 1], [0, 1], color=(1, 0, 0)),
                Line2D([0, 1], [0, 1], color=(0, 1, 0)),
                Line2D([0, 1], [0, 1], color=(0, 0, 1)),
            ),
            ["30", "50", "60"],
        )

        fig.savefig(filename)

    def plot_debug(self, filename: str):
        """plots the graph and node ids."""
        fig, (ax_debug) = plt.subplots(nrows=1, ncols=1, figsize=(10, 10))

        if self.last_update_location is not None:
            ax_debug.scatter(self.last_update_location[0], -self.last_update_location[1], s=25)

        for (from_node, to_node), edge in self.graph.edges.items():
            self._draw_edge(ax_debug, from_node, to_node, (0, 0, 0))

            if len(edge["path"]) > 0:
                wp = edge["path"][0]
                ax_debug.text(
                    wp.transform.location.x, -wp.transform.location.y, str(from_node), fontsize=6
                )

        if self.last_adjacent_edges is not None:
            for from_node, to_node in self.last_adjacent_edges["left"]:
                self._draw_edge(ax_debug, from_node, to_node, (1, 0, 0))

            for from_node, to_node in self.last_adjacent_edges["straight"]:
                self._draw_edge(ax_debug, from_node, to_node, (0, 1, 0))

            for from_node, to_node in self.last_adjacent_edges["right"]:
                self._draw_edge(ax_debug, from_node, to_node, (0, 0, 1))

        if self.last_edge is not None:
            from_node, to_node = self.last_edge
            self._draw_edge(ax_debug, from_node, to_node, (1, 1, 0))

        fig.savefig(filename)
