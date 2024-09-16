from typing import Tuple, List
import carla


class Graph:
    """
    Basic graph class which stores costs and paths as edge information
    """

    def __init__(self):
        self.nodes = dict()  # nodes maps node id to children ids.
        self.edges = dict()

    def add_node(self, node_id: int):
        """add a node with given id to the graph.
        :param node_id : node id of node to be added.
        """
        if node_id in self.nodes:
            raise ValueError("Node with id {} already exists.".format(node_id))
        self.nodes[node_id] = []

    def add_edge(
        self,
        from_node: int,
        to_node: int,
        from_xyz,
        to_xyz,
        from_wp,
        to_wp,
        cost,
        path,
        speed_limit,
    ):
        """
        Add an edge to the graph from node to a given end node with
        :param from_node: node id where the directed edge starts.
        :param to_node: node id where the directed edge ends.
        :param from_xyz: x,y,z) coordinates of entry waypoint
        :param to_xyz: (x,y,z) coordinates of the exit waypoint
        :param from_wp: corresponding entry waypoint for the start node.
        :param to_wp: corresponding exit waypoint for the end node.
        :param cost: costs
        :param path: along the edge as List[carla.Waypoint].
        :param speed_limit: maximum velocity permitted on the segment.
        """
        # Make to_node a child node of from_node
        self.nodes[from_node].append(to_node)
        self.edges[(from_node, to_node)] = {
            "from_xyz": from_xyz,
            "to_xyz": to_xyz,
            "from_wp": from_wp,
            "to_wp": to_wp,
            "cost": cost,
            "path": path,
            "speed_limit": speed_limit,
        }

    def get_path(self, from_node: int, to_node: int) -> List[carla.Waypoint]:
        """
        Returns the list of path waypoints of the edge connecting from_node to to_node
        """
        path = []
        #######################################################################
        ######################### TODO: IMPLEMENT THIS ########################
        #######################################################################
        # Solution start
        path = self.edges[(from_node, to_node)]["path"]
        # Solution end
        return path

    def get_children(self, node: int) -> List[int]:
        """
        Returns the list of child nodes
        """
        children = []
        #######################################################################
        ######################### TODO: IMPLEMENT THIS ########################
        #######################################################################
        # Solution start
        children = self.nodes[node]
        # Solution end
        return children

    def get_cost(self, from_node: int, to_node: int) -> int:
        """
        Returns the cost between from_node to to_node
        """
        cost = 0.0
        #######################################################################
        ######################### TODO: IMPLEMENT THIS ########################
        #######################################################################
        # Solution start
        cost = self.edges[(from_node, to_node)]["cost"]
        # Solution end
        return cost
