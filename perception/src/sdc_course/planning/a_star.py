from .graph import Graph
import math


def a_star_search(graph: Graph, node_to_xy, start: int, end: int):
    """
    Returns shortest path as a list of nodes ids.

    : param graph : graph definition.
    : param node_to_xy : mapping of nodes to x and y positions.
    : param start : id of the start node.
    : param end : id of the end/target node.
    """
    open_list = [start]  # List of nodes that have not been visited yet
    closed_list = []  # List of nodes that have been visited
    came_from = {start: None}  # Keep track of best predecessor of each node
    accumulated_cost = {start: 0}  # Keep track of each node and its accumulated cost
    estimated_cost = {start: 0}  # Keep track of each node and its estimated cost
    #######################################################################
    ######################### TODO: IMPLEMENT THIS ########################
    #######################################################################
    # Solution start
    while open_list:
        # Find node with lowest estimated cost in open_list
        next_node = min(open_list, key=lambda node: estimated_cost[node])

        # If end node is reached, backtrack through came_from dict and return the list of nodes from start to end
        if next_node == end:
            node_path = [end]
            from_node = came_from[end]
            while from_node != None:
                node_path = [from_node] + node_path
                from_node = came_from[from_node]
            return node_path

        # Remove this node from the open_list
        open_list.remove(next_node)

        # Add next_node to closed_list
        closed_list.append(next_node)

        # Get child nodes and evaluate their accumulated cost. Update accumulated_cost if necessary!
        children = graph.get_children(next_node)
        for child in children:
            cost = accumulated_cost[next_node] + graph.get_cost(next_node, child)

            if child in open_list and cost < accumulated_cost[child]:
                open_list.remove(child)
            if child in closed_list and cost < accumulated_cost[child]:
                closed_list.remove(child)
            if child not in open_list and child not in closed_list:
                accumulated_cost[child] = cost
                open_list.append(child)
                came_from[child] = next_node

                # Compute estimated cost based on heuristics
                loc_child_x, loc_child_y = node_to_xy[child]
                loc_end_x, loc_end_y = node_to_xy[end]
                heuristic = math.sqrt(
                    (loc_child_x - loc_end_x) ** 2 + (loc_child_y - loc_end_y) ** 2
                )
                estimated_cost[child] = cost + heuristic
    # Solution end
    return []
