import math
import time
import numpy as np
from sdc_course.utils.utility import load_params
from sdc_course.planning.graph import Graph
from sdc_course.planning.a_star import a_star_search
from sdc_course.planning.local_planner import LocalPlanner


class dummy_carla:
    def __init__(self):
        self.waypoint1 = None
        self.waypoint2 = None
        self.waypoint3 = None


def test_task_1():
    print("\nEvaluating task 1:")
    carla = dummy_carla()
    dummy_path = [carla.waypoint1, carla.waypoint2, carla.waypoint3]
    dummy_cost = 66.0
    id_start = 0
    id_end = 1
    g = Graph()
    g.add_node(id_start)
    g.add_node(id_end)
    g.add_edge(id_start, id_end, None, None, None, None, dummy_cost, dummy_path, None)

    try:
        path = g.get_path(id_start, id_end)
    except:
        path = None
    assert path == dummy_path, "Task 1.1 'get_path' failed!"

    try:
        children = g.get_children(id_start)
    except:
        children = None
    assert children == [id_end], "Task 1.2 'get_children' not passed!"

    try:
        cost = g.get_cost(id_start, id_end)
    except:
        cost = None
    assert cost == dummy_cost, "Task 1.3 'get_cost' not passed!"

    print("-> Task 1 passed")


def test_task_2():
    print("\nEvaluating task 2:")
    node_path_answer = []
    node_path_solution = [0, 1, 2, 3]
    node_to_xy = {}
    g = Graph()

    for n in range(0, 4):
        g.add_node(n)

    g.add_edge(0, 1, None, None, None, None, 10, None, None)
    g.add_edge(0, 2, None, None, None, None, 20, None, None)
    g.add_edge(1, 3, None, None, None, None, 30, None, None)
    g.add_edge(2, 3, None, None, None, None, 10, None, None)
    node_to_xy[0] = (0, 0)
    node_to_xy[1] = (0, 1)
    node_to_xy[2] = (1, 0)
    node_to_xy[3] = (1, 1)

    # Create a free loop
    g.add_edge(0, 0, None, None, None, None, 0, None, None)

    # Some more nodes on the first level to trick breadth first search methods
    n_additional_nodes = 10000
    for n in range(4, n_additional_nodes):
        g.add_node(n)
        g.add_edge(0, n, None, None, None, None, 100, None, None)
        node_to_xy[n] = (2, 0)

    # One deep branch with zero cost, leading away from the goal. A* would not select them.
    g.add_node(n_additional_nodes)
    g.add_edge(0, n_additional_nodes, None, None, None, None, 0, None, None)
    node_to_xy[n_additional_nodes] = (-100, -100)
    for n in range(n_additional_nodes + 1, 2 * n_additional_nodes):
        g.add_node(n)
        g.add_edge(n - 1, n, None, None, None, None, 0, None, None)
        node_to_xy[n] = (-100, -100)

    # Add a free shortcut to see if A Star checks for a cheaper path to nodes in the open list
    g.add_edge(1, 2, None, None, None, None, 0, None, None)

    time_start = time.time()
    node_path_answer = a_star_search(g, node_to_xy, 0, 3)
    time_needed = time.time() - time_start
    print(f"Found path in {round(time_needed, 5)} seconds.")

    assert (
        node_path_answer == node_path_solution
    ), f"Task 2 failed, your answer is {node_path_answer} but should be {node_path_solution}"

    print("-> Task 2 passed")


def test_task_3():
    print("\nEvaluating task 3:")
    params = load_params("params.yaml")
    local_planner = LocalPlanner(None, None, [(None, None, None)], params)
    start = [1, 2, math.pi / 3]
    end = [5, 4, math.pi / 9]
    c1 = 4.0
    c2 = 5.0
    parameters = local_planner._fit_cubic(start, end, c1, c2)
    solution = np.array(
        [[-1.3015369, 3.3015369, 2.0, 1.0], [1.17420233, -2.63830395, 3.46410162, 2.0]]
    )
    assert parameters is not None
    assert np.allclose(
        parameters, solution
    ), f"Task 3 failed, your answer is {parameters} but should be {solution}"

    print("-> Task 3 passed")


def test_task_4():
    print("\nEvaluating task 4:")
    print(" -> Task 4 needs to be evaluated in the simulator.")
    assert True


def main():
    test_task_1()
    test_task_2()
    test_task_3()
    test_task_4()


if __name__ == "__main__":
    main()
