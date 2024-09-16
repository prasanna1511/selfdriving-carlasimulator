import numpy as np

from sdc_course.perception.traffic_sign_map import TrafficSignMap
from sdc_course.perception.utils import image_to_world
from copy import deepcopy
from carla import Transform, Location, Rotation


class Camera_mock:
    def __init__(self):
        self.calibration = np.array([[280, 0, 400], [0, 280, 300], [0, 0, 1]])

    def get_transform(self):
        return Transform(Location(10, 15, 0), Rotation(0, 0, 0))


class Vehicle_mock:
    def get_camera(self):
        return Camera_mock()


class World_mock:
    def get_vehicle(self):
        return Vehicle_mock()


def test_task_1():
    print("\nEvaluating task 1:")
    print(
        " -> Task 1 needs to be evaluated in the simulator. Check if bounding boxes enclose the traffic signs.\n"
    )
    assert True


def test_task_2():
    print("\nEvaluating task 2:")
    p = image_to_world(World_mock(), (10, 10), 14.0)
    solution = np.array([24, -4.5, 14.5, 1])
    assert np.allclose(
        p, solution
    ), f"First test failed, your solution is {p} but should be {solution}"

    p = image_to_world(World_mock(), (10, 10), 123.0)
    solution = np.array([133, -156.321, 127.392, 1])
    assert np.allclose(
        p, solution
    ), f"Second test failed, your solution is {p} but should be {solution}"

    print("-> Task 2 passed")


def test_task_3():
    print("\nEvaluating task 3:")
    map = TrafficSignMap()
    assert len(map.get_traffic_signs()) == 0

    map.update_map((10, 10), "left_turn", 0.5)
    assert len(map.get_traffic_signs()) == 1, "there should be a traffic sign inside the map"
    sign_before = deepcopy(map.get_closest_sign((10, 10)))

    map.update_map((10, 10), "left_turn", 0.8)
    assert (
        len(map.get_traffic_signs()) == 1
    ), "there should be only a single traffic sign inside the map"
    sign_after = deepcopy(map.get_closest_sign((10, 10)))

    assert (
        sign_before.distribution[0] != sign_after.distribution[1]
    ), "there should be a change in the probability."

    map.update_map((15, 10), "right_turn", 0.8)
    assert (
        len(map.get_traffic_signs()) == 2
    ), "there should be now more traffic signs inside the map"

    print("-> Task 3 passed")


def test_task_4():
    print("\nEvaluating task 4:")
    print("-> Task 4 needs to be evaluated in the simulator.")
    assert True


def main():
    test_task_1()
    test_task_2()
    test_task_3()
    test_task_4()


if __name__ == "__main__":
    main()
