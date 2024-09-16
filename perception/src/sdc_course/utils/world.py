import numpy as np
import random
import queue
import os
import yaml
import traceback
import numpy as np
from typing import List, Tuple, Optional

import carla
from carla import Location, Transform

from .car import Car
from .utility import carla2numpy


class World:
    """Wrapper around the carla World handling initialization, including spawning of traffic signs, etc."""

    def __init__(
        self,
        params: dict,
        spawn_position: Optional[str] = None,
        scenario_name: Optional[str] = None,
    ):
        """

        Args:
            spawn_position (str, optional): Location, angle. Defaults to None.
            scenario_name (str, optional): [description]. Defaults to None.
        """
        self.params = params
        self._client = carla.Client("localhost", 2000)
        self._client.load_world("Town03")
        self._client.set_timeout(5.0)
        self._world = self._client.get_world()

        self._delta_seconds = params["sampling_time"]

        self._old_settings = self._world.get_settings()
        self.frame = self._world.apply_settings(
            carla.WorldSettings(synchronous_mode=True, fixed_delta_seconds=self._delta_seconds)
        )

        self._camera_observer = None
        self._raw_observer_data = None
        self._observer_data = None
        self._actors = []
        self._parked_cars = []
        self._traffic_signs = []
        self._vehicle = None
        self._package_path = os.path.dirname(os.path.abspath(__file__))

        try:
            self._vehicle = Car(self.params, self._world, spawn_position)
            self.spawn_observer()
            self._spawn_signs(scenario_name)
            self._spawn_parked_cars(scenario_name)

        except Exception as ex:
            traceback.print_exc()
            # if something breaks here, we don't want to have some zombies.
            self.destroy()

    def destroy(self):
        """destroy all actors"""

        if self._vehicle:
            self._vehicle.destroy()
        if self._camera_observer:
            self._camera_observer.destroy()
        self._client.apply_batch([carla.command.DestroyActor(x) for x in self._actors])

        self._actors = []
        self._parked_cars = []
        self._traffic_signs = []
        self._world.apply_settings(self._old_settings)  # back to asynchronous mode.

    def _load_signs(self, scenario_name: str) -> List[Tuple[float, float, float, int]]:
        signs = []
        sign_file = os.path.join(self._package_path, "town03_signs.txt")
        if not os.path.exists(sign_file):
            raise FileNotFoundError("sign locations missing.")

        with open(sign_file) as fp:
            for line in fp:
                coords = [float(coord) for coord in line.split(",")]
                signs.append((Location(*coords[:3]), coords[3]))

        if scenario_name is None:
            signs = [(loc, angle, random.choice(range(5))) for loc, angle in signs]
        else:
            scenario_file = os.path.join(self._package_path, f"{scenario_name}.yaml")
            if not os.path.exists(scenario_file):
                raise FileNotFoundError("scenario file does not exist")
            with open(scenario_file) as fp:
                scenario = yaml.safe_load(fp)
                signs = [
                    (*sign_pose, sign_type)
                    for sign_pose, sign_type in zip(signs, scenario["signs"])
                ]

        return signs

    def _spawn_signs(self, scenario_name: Optional[str] = None):
        blueprint_library = self._world.get_blueprint_library()

        traffic_signs = [
            blueprint_library.find("static.prop.leftturn"),
            blueprint_library.find("static.prop.rightturn"),
            blueprint_library.find("static.prop.maxspeed30"),
            blueprint_library.find("static.prop.maxspeed50"),
            blueprint_library.find("static.prop.maxspeed60"),
        ]
        pole_bp = blueprint_library.find("static.prop.pole")

        signs = self._load_signs(scenario_name)

        for loc, angle, sign_type in signs:
            bp = traffic_signs[sign_type]
            t = carla.Transform(loc, carla.Rotation(yaw=angle))

            self._actors.append(self._world.spawn_actor(bp, t))
            self._actors.append(self._world.spawn_actor(pole_bp, t))

            self._traffic_signs.append({"transform": t, "type": sign_type})

    def _spawn_parked_cars(self, scenario_name=None):
        if scenario_name is None:
            return

        scenario_file = os.path.join(self._package_path, f"{scenario_name}.yaml")
        if not os.path.exists(scenario_file):
            raise FileNotFoundError("scenario file does not exist")

        blueprint_library = self._world.get_blueprint_library()

        cars = [
            blueprint_library.find("vehicle.mercedes.coupe"),
            blueprint_library.find("vehicle.audi.a2"),
            blueprint_library.find("vehicle.dodge.charger_police"),
            blueprint_library.find("vehicle.toyota.prius"),
            blueprint_library.find("vehicle.audi.etron"),
        ]

        with open(scenario_file) as fp:
            scenario = yaml.safe_load(fp)
            if "parked_cars" not in scenario:
                return

            for car_location in scenario["parked_cars"]:
                bp = random.choice(cars)
                t = carla.Transform(
                    Location(*car_location[:2], 2), carla.Rotation(yaw=car_location[2])
                )

                self._actors.append(self._world.spawn_actor(bp, t))
                self._actors[-1].apply_control(carla.VehicleControl(hand_brake=True))
                self._parked_cars.append(t)

    def spawn_observer(self, width: int = 800, height: int = 600):
        """create and spawn the third person observer behind the car."""
        if not self._camera_observer is None:
            self._camera_observer.destroy()

        blueprint_library = self._world.get_blueprint_library()
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(width))
        camera_bp.set_attribute("image_size_y", str(height))

        self._camera_observer = self._world.spawn_actor(
            camera_bp,
            carla.Transform(carla.Location(x=-0, z=0), carla.Rotation(pitch=0)),
            attach_to=self._vehicle.get_vehicle(),
            attachment_type=carla.AttachmentType.SpringArm,
        )
        self.set_observer_mode(0)

        self._observer_queue = queue.Queue()
        self._camera_observer.listen(self._observer_queue.put)

    def set_observer_mode(self, mode: int):
        """
        :mode: 0, = behind the car (near), 1 = behind the car (far), 2 = above the car (bird's eye view)
        """

        if mode == 1:
            self._camera_observer.set_transform(
                carla.Transform(carla.Location(x=-20.5, z=5), carla.Rotation(pitch=-2))
            )
        elif mode == 2:
            self._camera_observer.set_transform(
                carla.Transform(carla.Location(x=-0, z=25), carla.Rotation(pitch=-90))
            )
        else:
            self._camera_observer.set_transform(
                carla.Transform(carla.Location(x=-10.5, z=2.8), carla.Rotation(pitch=-2))
            )

    def get_snapshot(self) -> carla.WorldSnapshot:
        """get world snapshot with the current state."""
        return self._world.get_snapshot()

    def get_vehicle(self) -> Car:
        """Get the vehicle with steering and sensors."""
        return self._vehicle

    def get_view(self, name: str = "observer") -> np.array:
        """get a specific image from a certain sensor."""

        if name == "observer":
            return self._observer_data

        return self._vehicle.get_sensor_data()[name]

    def tick(self, timeout=2.0):
        """advance simulation, fill data buffers, etc."""
        self.frame = self._world.tick()
        while True:
            data = self._observer_queue.get(timeout=timeout)
            if data.frame == self.frame:
                self._raw_observer_data = data
                self._observer_data = carla2numpy(data, "RGB")
                break

        self._vehicle.update(self.frame, timeout)

    def get_debug_helper(self):
        """get the debug helper for drawing debug information."""
        return self._world.debug

    def get_world_map(self) -> carla.Map:
        """get the world's map for path planning."""
        return self._world.get_map()

    def get_parked_cars(self) -> List[carla.Transform]:
        """geting list of parked cars by their carla.Transform"""
        return self._parked_cars

    def get_traffic_signs(self) -> List[dict]:
        """get list of traffic sings in the world.
        returns a list of dicts with keys: transform, type
        """
        return self._traffic_signs
