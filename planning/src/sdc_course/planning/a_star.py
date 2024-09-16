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
    estimated_cost = {start: 0}  # Keep track of each node and its estimated cost //should be less than accumulated cost//


    #######################################################################
    ######################### TODO: IMPLEMENT THIS ########################
    #######################################################################
    
    # While there are still nodes to visit
    #explore which node has the lowest cost
    #add it to the closed list
    #remove it from the open list
    #add its children to the open list
    #update the cost of the children
        
    children = graph.get_children(start)
    open_list.append(children)

    cost =[]

    while open_list:
        for node in open_list:
            cost.append(accumulated_cost[node])

        min_cost = min(cost)
        min_cost_index = cost.index(min_cost)
        current_node = open_list[min_cost_index]
        closed_list.append(current_node)
        open_list.remove(current_node)
        # print(open_list)

        children = graph.get_children(current_node)
        open_list.append(children)
        print(open_list)

        for child in children:
            if child not in open_list:
                open_list.append(child)
                accumulated_cost[child] = accumulated_cost[current_node] + graph.edges[(current_node, child)]["cost"]
                estimated_cost[child] = accumulated_cost[child] + heuristic(node_to_xy[child], node_to_xy[end])
                came_from[child] = current_node

            elif accumulated_cost[child] > accumulated_cost[current_node] + graph.edges[(current_node, child)]["cost"]:
                accumulated_cost[child] = accumulated_cost[current_node] + graph.edges[(current_node, child)]["cost"]
                estimated_cost[child] = accumulated_cost[child] + heuristic(node_to_xy[child], node_to_xy[end])
                came_from[child] = current_node

            else:
                continue

        if current_node == end:
            break

    # return came_from
        return children
