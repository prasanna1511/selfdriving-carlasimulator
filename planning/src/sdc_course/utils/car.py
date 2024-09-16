import queue
from typing import Dict, Optional

import numpy as np
import carla

from .utility import carla2numpy


class Car:
    """Vehicle representation containing the methods to query sensor information (get_sensor_data), but also control
    the vehicle.
    """

    def __init__(
        self, params: dict, world: carla.World, spawn_position: Optional[carla.Transform] = None
    ):
        self._world = world
        self.params = params

        if spawn_position is None:
            m = world.get_map()
            spawn_position = m.get_spawn_points()[0]

        blueprint_library = world.get_blueprint_library()
        self._vehicle = world.spawn_actor(blueprint_library.filter("vehicle.*")[0], spawn_position)

        self._queues = []
        self.current_frame = None
        self._raw_data = None
        self._current_data = {}

        camera_location = (0.25, 0, 1.5)

        image_width = 800
        image_height = 600
        image_fov = 110

        bp = blueprint_library.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(image_width))
        bp.set_attribute("image_size_y", str(image_height))
        bp.set_attribute("fov", str(image_fov))
        self._camera_rgb = world.spawn_actor(
            bp, carla.Transform(carla.Location(*camera_location)), attach_to=self._vehicle
        )

        calibration = np.identity(3)
        calibration[0, 2] = image_width / 2.0
        calibration[1, 2] = image_height / 2.0
        calibration[0, 0] = calibration[1, 1] = image_width / (
            2.0 * np.tan(image_fov * np.pi / 360.0)
        )
        self._camera_rgb.calibration = calibration
        self._camera_rgb.width = image_width
        self._camera_rgb.height = image_height

        bp = blueprint_library.find("sensor.camera.depth")
        bp.set_attribute("image_size_x", str(image_width))
        bp.set_attribute("image_size_y", str(image_height))
        bp.set_attribute("fov", str(image_fov))
        self._camera_depth = world.spawn_actor(
            bp, carla.Transform(carla.Location(*camera_location)), attach_to=self._vehicle
        )

        self._camera_depth.calibration = calibration
        self._camera_depth.width = image_width
        self._camera_depth.height = image_height

        self._sensors = [self._camera_rgb, self._camera_depth]
        self._sensor_names = ["rgb", "depth"]
        self._current_data = {name: None for name in self._sensor_names}

        self._autopilot = False

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        for sensor in self._sensors:
            make_queue(sensor.listen)

        self.L = params["model"]["L"]

    def destroy(self):
        """free all Carla resources"""
        self._vehicle.destroy()
        self._camera_rgb.destroy()
        self._camera_depth.destroy()

    def set_autopilot(self, state: bool):
        """set automatic driving mode."""
        self._autopilot = state

    @property
    def autopilot_engaged(self) -> bool:
        """Determine if autopilot is engaged."""
        return self._autopilot

    def get_sensor_data(self) -> Dict[str, np.array]:
        """get tuple with sensor data."""
        return self._current_data

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.current_frame:
                return data

    def update(self, frame, timeout=2.0):
        """update sensor information based on world tick information.
        frame: which frame data to retrieve.
        """
        if self.current_frame != frame:
            self.current_frame = frame
            self._raw_data = [self._retrieve_data(q, timeout) for q in self._queues]
            self._current_data = {
                name: carla2numpy(data) for name, data in zip(self._sensor_names, self._raw_data)
            }

    def get_control(self):
        """get control of underlying CARLA vehicle.
        :return: carla.VehicleControl
        """
        return self._vehicle.get_control()

    def apply_control(self, control):
        """apply given control command."""
        self._vehicle.apply_control(control)

    def get_vehicle(self):
        """get underlying carla.Vehicle"""
        return self._vehicle

    def get_transform(self):
        """get world pose of the car. The used controller defines if the front axis, rear axis or center of gravity is used as origin
        :return: world pose as carla.Transform
        """
        transform_cog = self._vehicle.get_transform()
        if self.params["control"]["strategy"] in ["stanley", "pid"]:
            location_front_axis = carla.Location(
                self._vehicle.get_transform().transform(carla.Location(self.L / 2, 0, 0))
            )
            transform = carla.Transform(location_front_axis, transform_cog.rotation)
        else:
            location_rear_axis = carla.Location(
                self._vehicle.get_transform().transform(carla.Location(-self.L / 2, 0, 0))
            )
            transform = carla.Transform(location_rear_axis, transform_cog.rotation)

        return transform

    def get_velocity(self):
        """
        get the vehicle's velocity.

        :return: velocity in XXX
        """
        return self._vehicle.get_velocity()

    def get_camera(self, type: str = "rgb"):
        if type == "depth":
            return self._camera_depth
        return self._camera_rgb
