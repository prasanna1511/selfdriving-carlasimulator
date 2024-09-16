import carla
from sdc_course.utils.car import Car
from sdc_course.utils.utility import *
from .controller import AbstractController
from .pid import PIDLongitudinalController, PIDLateralController
from .stanley import StanleyLateralController
from .purepursuit import PurePursuitLateralController


class DecomposedController(AbstractController):
    """
    DecomposedController is the combination of two controllers
    (lateral and longitudinal) to perform the low level control a vehicle from client side.
    The longitudinal control is a PID controller, whereas the lateral controller can be
    chosen from a PID controller, Pure Pursuit Contoller or a Stanley Controller.
    """

    def __init__(self, vehicle: Car, params):
        """
        Constructor method.
        :param vehicle: actor to apply to local planner logic onto
        :param params: All the controller related parameters
        """
        super().__init__(vehicle, params)

        # Longitudinal
        args_longitudinal = params["control"]["longitudinal"]["pid"]
        self._lon_controller = PIDLongitudinalController(
            self._vehicle, self.dt, **args_longitudinal
        )

        # Lateral
        lateral_strategy = params["control"]["strategy"]
        args_lateral = params["control"]["lateral"][lateral_strategy]
        if lateral_strategy == "stanley":
            self._lat_controller = StanleyLateralController(self._vehicle, **args_lateral)
        elif lateral_strategy == "pure-pursuit":
            self._lat_controller = PurePursuitLateralController(
                self._vehicle, self.L, **args_lateral
            )
        else:
            self._lat_controller = PIDLateralController(self._vehicle, self.dt, **args_lateral)

    def compute_steering_and_acceleration(self, target_velocity_ms, waypoints):
        """
        Computes one step of control invoking both lateral and longitudinal
        :param target_velocity_ms: desired vehicle velocity in m/s
        :param waypoints: local trajectory waypoints
        :return: control command for the vehicle.
        """
        current_steering = self._lat_controller.run_step(waypoints)
        acceleration = self._lon_controller.run_step(target_velocity_ms)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.
        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1
        return current_steering, acceleration
