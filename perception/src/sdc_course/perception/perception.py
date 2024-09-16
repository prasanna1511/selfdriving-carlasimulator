import sys
from typing import List, Tuple
import numpy as np
from math import atan2, degrees, acos, sqrt
import carla

from ..utils import World
from .utils import transform_to_numpy, BoundingBox, CATEGORIES


class Perception:
    """Perception module providing information about

    - parked cars using information from the world.
    - traffic signs using a vision-based detector.


    """

    def __init__(self, world: World):
        self._detector = None
        self._world = world
        self.max_distance = 25

    def detect_parked_cars(
        self, max_distance: float = 20.0, field_of_view: float = 90.0
    ) -> List[carla.Location]:
        """
        Determine parked cars around the vehicle.

        It's a work around that uses the known poses of spawned parked cars to determine which parked cars are nearby.
        """
        parked_cars = self._world.get_parked_cars()
        cars = []

        car_pose = self._world.get_vehicle().get_transform()
        for parked_car in parked_cars:
            vec_car2parked = np.array(
                [
                    parked_car.location.x - car_pose.location.x,
                    parked_car.location.y - car_pose.location.y,
                ]
            )
            distance2parked = sqrt(np.dot(vec_car2parked, vec_car2parked))

            if distance2parked < max_distance:
                cars.append(parked_car.location)

        return cars

    def detect_traffic_signs(self) -> List[BoundingBox]:
        """
        Detect traffic signs in the field of view of the camera.

        In a real system, this would be a detector like YOLO.

        However, we wanted to make it work without PyTorch/Tensorflow and
        this way we can ensure that it even works on not so beefy systems.

        return:

          List of 2D image bounding boxes.
        """

        signs = self._world.get_traffic_signs()
        sign_categories = {i: name for i, name in enumerate(CATEGORIES)}

        cam = self._world.get_vehicle().get_camera()
        cam_transform = transform_to_numpy(cam.get_transform())
        cam_calibration = cam.calibration
        world_to_cam = np.linalg.inv(cam_transform)

        car_pose = self._world.get_vehicle().get_transform()
        vec_car_forward = np.array(
            [car_pose.get_forward_vector().x, car_pose.get_forward_vector().y]
        )
        vec_car_forward = vec_car_forward / sqrt(np.dot(vec_car_forward, vec_car_forward))

        detections = []

        def world_to_cam_pos(p, T) -> np.array:
            pos = np.dot(T, p.T)
            # source: PythonAPI/examples/client_bounding_boxes.py
            # left handed coordinate system (x forward, y right, z up)
            coords_cam = np.dot(world_to_cam, pos.T)
            # right handed coordinate system (x right, y down, z forward)
            coords_cam = np.array([coords_cam[1], -coords_cam[2], coords_cam[0]]).T

            return coords_cam

        def world_to_image_pos(p, T) -> np.array:
            coords_cam = world_to_cam_pos(p, T)

            image_pos = np.dot(cam_calibration, coords_cam)
            image_pos[0] = image_pos[0] / image_pos[2]
            image_pos[1] = image_pos[1] / image_pos[2]

            return image_pos

        def clamp(value, minimum, maximum):
            return min(max(value, minimum), maximum)

        for sign in signs:
            # transform
            points = []
            points.append(np.array([0.3, 0.0, 1.7, 1.0]))
            points.append(np.array([-0.3, 0.0, 1.7, 1.0]))
            points.append(np.array([0, 0.0, 2.0, 1.0]))
            points.append(np.array([0, 0.0, 1.4, 1.0]))

            T = transform_to_numpy(sign["transform"])

            coords_cam = world_to_cam_pos(points[0], T)
            # y axis is pointing backwards in the direction of the sign.
            sign_backward = T[:2, 1]
            sign_backward = sign_backward / sqrt(np.dot(sign_backward, sign_backward))

            sign_angle = abs(degrees(acos(np.dot(vec_car_forward, sign_backward))))

            if sign_angle > 45:
                continue
            if coords_cam[2] < 0:
                continue  # point is behind the plane.
            distance = np.linalg.norm(coords_cam)

            if distance < self.max_distance:
                min_x, min_y, _ = world_to_image_pos(points[0], T)
                max_x, max_y = min_x, min_y

                # compute projected bounding box coordinates.
                for point in points:
                    image_pos = world_to_image_pos(point, T)
                    min_x = int(min(image_pos[0], min_x))
                    min_y = int(min(image_pos[1], min_y))
                    max_x = int(max(image_pos[0], max_x))
                    max_y = int(max(image_pos[1], max_y))

                min_x = clamp(min_x, 0, cam.width - 1)
                max_x = clamp(max_x, 0, cam.width - 1)
                min_y = clamp(min_y, 0, cam.height - 1)
                max_y = clamp(max_y, 0, cam.height - 1)

                confidence, sign_type = self.sample_type(distance, sign["type"])

                if max_y - min_y > 0 and max_x - min_x > 0:
                    detections.append(
                        BoundingBox(
                            min_x + 0.5 * (max_x - min_x),
                            min_y + 0.5 * (max_y - min_y),
                            max_x - min_x,
                            max_y - min_y,
                            sign_categories[sign_type],
                            confidence,
                        )
                    )
                    detections[-1].world_position = np.dot(T, np.array([0, 0, 1.7, 1.0]).T)

        return detections

    def sample_type(self, distance, sign_type):
        """given the type of the sign and distance, we introduce a little bit randomness to introduce some "realness",
        since a detector will never work perfectly. Thus, we model a bit inaccuracies in the detections.
        """
        P = {
            0: np.array([0.95, 0.05, 0, 0, 0]),
            1: np.array([0.05, 0.95, 0, 0, 0]),
            2: np.array([0.0, 0.0, 0.9, 0.1, 0.1]),
            3: np.array([0.0, 0.0, 0.15, 0.8, 0.05]),
            4: np.array([0.0, 0.0, 0.15, 0.01, 0.85]),
        }
        scale = min(max(1.0 - (distance - 3.0) / (self.max_distance - 3.0), 0.1), 2.0)

        P_modified = P[sign_type] ** scale / np.sum(P[sign_type] ** scale)
        new_sign_type = np.random.choice([0, 1, 2, 3, 4], p=P_modified)
        confidence = P_modified[sign_type]

        return confidence, new_sign_type
