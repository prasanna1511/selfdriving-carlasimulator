import carla
from sdc_course.utils.car import Car
from sdc_course.utils.utility import *


class AbstractController:
    """
    Controller Class
    """

    def __init__(self, vehicle: Car, params):
        """
        Constructor method.
        :param vehicle: actor to apply to local planner logic onto
        :param params: All the controller related parameters
        """

        self.max_brake = params["control"]["max_brake"]
        self.max_throttle = params["control"]["max_throttle"]
        self.max_steer = params["control"]["max_steering"]
        self.L = params["model"]["L"]
        self.dt = params["sampling_time"]

        self._vehicle = vehicle
        self.past_steering = self._vehicle.get_control().steer

    def compute_control(self, target_velocity_kmph, waypoints):
        """
        Computes one step of control
        :param target_velocity_kmph: desired vehicle velocity in kmph
        :param waypoints: local trajectory waypoints
        :return: control command for the vehicle.
        """
        # target waypoint reached.
        if len(waypoints) < 4:
            return carla.VehicleControl(0, 0, brake=1.0)

        target_velocity_ms = target_velocity_kmph / 3.6

        current_steering, acceleration = self.compute_steering_and_acceleration(
            target_velocity_ms, waypoints
        )

        # Set limits
        control = carla.VehicleControl()
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering

        return control

    def compute_steering_and_acceleration(self, target_velocity_ms, waypoints):
        raise NotImplementedError("Controller is not specified!")
