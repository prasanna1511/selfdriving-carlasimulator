import os
import math
import carla
from sdc_course.control.pid import PIDLateralController, PIDLongitudinalController
from sdc_course.control.purepursuit import PurePursuitLateralController
from sdc_course.control.stanley import StanleyLateralController
from sdc_course.control.mpc import ModelPredictiveController
from sdc_course.utils.utility import load_params


class dummy_car:
    def get_transform(self):
        return carla.Transform(carla.Location(24.2, 14.6, -13.7), carla.Rotation(0.3, 12.0, 1.2))

    def get_velocity(self):
        return carla.Vector3D(7.4, 10.5, 0.5)

    def get_control(self):
        return carla.VehicleControl(3.4, -2.2, 2.1)


def test_task_1():
    print("\nEvaluating task 1:")
    assert os.path.exists("results/recorded_trajectory.txt"), "-> Task 1 not passed!"
    print("-> Task 1 passed!")


def test_task_2():
    print("\nEvaluating task 2:")
    vehicle = dummy_car()
    pid_long = PIDLongitudinalController(vehicle, 0.11, 10.4, 20.8, 10.7)
    pid_long._error_buffer.append(6.4)
    pid_long._error_buffer.append(5.9)
    pid_long._error_buffer.append(5.2)
    acceleration = pid_long.run_step(18)
    assert math.isclose(
        acceleration, 69.6911156016661, rel_tol=1e-4
    ), f"Your computed acceleration with PID is {acceleration} which differs from the one we computed for these dummy values."
    pid_lat = PIDLateralController(vehicle, 0.95, 1.5, 2.8, 0.7)
    waypoints = [[25.1, 14.1, -13.2]]
    pid_lat._error_buffer.append(4.4)
    pid_lat._error_buffer.append(-2.9)
    pid_lat._error_buffer.append(3.2)
    steering = pid_lat.run_step(waypoints)
    assert math.isclose(
        steering, -11.56958438813039, rel_tol=1e-4
    ), f"Your computed steering with PID is {steering} which differs from the one we computed for these dummy values."
    print("-> Task 2 passed!")


def test_task_3():
    print("\nEvaluating task 3:")
    vehicle = dummy_car()
    waypoints = [[20.0, 10.4, -12.0], [25.1, 14.1, -13.2], [29.4, 18.9, -13.0]]

    purepursuit = PurePursuitLateralController(vehicle, 9.3, 3.4, 39.2)
    steering = purepursuit.run_step(waypoints)
    assert steering != 0
    assert math.isclose(
        steering, 0.01709177472245794, rel_tol=1e-4
    ), f"Your computed steering with Pure Pursuit is {steering} which differs from the one we computed for these dummy values."

    stanley = StanleyLateralController(vehicle, 20.2)
    steering = stanley.run_step(waypoints)
    assert steering != 0
    assert math.isclose(
        steering, -0.386304237978627, rel_tol=1e-4
    ), f"Your computed steering with Stanley is {steering} which differs from the one we computed for these dummy values."
    print("-> Task 3 passed!")


def test_task_4():
    print("\nEvaluating task 4:")
    vehicle = dummy_car()
    mpc = ModelPredictiveController(vehicle, load_params("./params.yaml"))
    waypoints = [
        [24.3, 14.7, -13.6],
        [24.6, 14.8, -13.2],
        [24.9, 14.9, -13.0],
        [25.1, 15.0, -12.0],
    ]
    control = mpc.compute_control(15.3, waypoints)
    throttle = control.throttle
    brake = control.brake
    steering = control.steer
    assert math.isclose(throttle, 0.0, rel_tol=1e-4)
    assert math.isclose(steering, -0.4767518639564514, rel_tol=1e-4)
    assert math.isclose(brake, 0.30000001192092896, rel_tol=1e-4)
    print("-> Task 4 passed!")


def main():
    test_task_1()
    test_task_2()
    test_task_3()
    test_task_4()


if __name__ == "__main__":
    main()
