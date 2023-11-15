""" Utility methods used by various parts.
"""

import numpy as np
import csv
import math
import yaml
from typing import Tuple, List

import carla


def wrapToPi(theta):
    while theta < -np.pi:
        theta = theta + 2 * np.pi
    while theta > np.pi:
        theta = theta - 2 * np.pi
    return theta


def waypoint_distance(wp1: Tuple[float, float, float], wp2: Tuple[float, float, float]):
    """compute euclidean distance between waypoints."""
    return math.sqrt((wp1[0] - wp2[0]) ** 2 + (wp1[1] - wp2[1]) ** 2)


def load_params(params_file: str):
    """load parameters from the given parameter file."""
    with open(params_file) as fh:
        params = yaml.safe_load(fh)

    return params


def carla2numpy(carla_image: carla.Image, mode: str = "RGB"):
    """convert carla image to numpy array.
    Note: carla.image is BGRA with uint8 per color.
    """
    arr = np.frombuffer(carla_image.raw_data, dtype=np.uint8).reshape(
        carla_image.height, carla_image.width, 4
    )
    if mode == "RGB":
        return arr[:, :, :3][:, :, ::-1]
    elif mode == "BGR":
        return arr[:, :, :3]


def load_waypoints_from_txt(waypoints_file: str):
    """load waypoints from given filename.
    :param waypoint_file: name of the file containing the waypoints.
    """
    with open(waypoints_file) as waypoints_fh:
        waypoints = list(csv.reader(waypoints_fh, delimiter=",", quoting=csv.QUOTE_NONNUMERIC))

    return waypoints


def get_locally_sampled_trajectory(
    vehicle: "Car",
    trajectory: List[Tuple[float, float, float]],
    max_len: float,
    min_distance: float,
):
    """sample trajectory of specified maximum length near to the car."""

    _, ind_nearest = get_nearest_waypoint(vehicle, trajectory)
    ind_start = max(ind_nearest, 0)
    trajectory_sampled = [trajectory[ind_start]]
    traj_length = 0
    ind_last = ind_start
    ind_current = ind_start + 1
    while traj_length < max_len and ind_current < len(trajectory):
        distance = waypoint_distance(trajectory[ind_current], trajectory[ind_last])
        if distance > min_distance:
            trajectory_sampled.append(trajectory[ind_current])
            ind_last = ind_current
            traj_length += distance
        ind_current += 1

    return trajectory_sampled


def get_velocity_kmph(vehicle):
    """
    Compute velocity of a vehicle in Km/h.
        :param vehicle: the vehicle for which velocity is calculated
        :return: velocity as a float in Km/h
    """
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)


def get_velocity_ms(vehicle: "Car") -> float:
    """
    Compute velocity of a vehicle in m/s.

        :param vehicle: the vehicle for which velocity is calculated
        :return: velocity as a float in m/s
    """
    vel = vehicle.get_velocity()
    return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)


def get_target_speed(vehicle: "Car", trajectory: List[Tuple[float, float, float]]) -> float:
    nearest_waypoint, _ = get_nearest_waypoint(vehicle, trajectory)
    if len(nearest_waypoint) == 3:
        speed = max(nearest_waypoint[2], 5.0)
    else:
        speed = 50.0  # kmph
    return speed


def get_nearest_waypoint(
    vehicle: "Car", trajectory: List[Tuple[float, float, float]]
) -> Tuple[Tuple[float, float, float], int]:
    """get nearest waypoint for given vehicle.

    : param vehicle: Car
    : param trajectory: waypoint list

    : return : tuple of nearest waypoint and index of nearest waypoint
    """
    distances = []
    for wp in trajectory:
        dist = compute_distance_to_waypoint(vehicle, wp)
        distances.append(dist)
    ind_nearest = distances.index(min(distances))
    nearest_waypoint = trajectory[ind_nearest]

    return nearest_waypoint, ind_nearest


def compute_distance_to_waypoint(vehicle: "Car", waypoint: Tuple[float, float, float]) -> float:
    """compute distance of vehicle to given waypoint."""
    vloc = vehicle.get_transform().location
    distance = math.sqrt((vloc.x - waypoint[0]) ** 2 + (vloc.y - waypoint[1]) ** 2)
    return distance


def interpolate_waypoints_linear(
    waypoints: List[Tuple[float, float, float]], INTERP_DISTANCE_RES: float = 0.1
):
    """function to linearly interpolate between waypoints with a given interpolation distance"""
    waypoints_np = np.array(waypoints)

    wp_distance = []  # distance array
    for i in range(1, len(waypoints)):
        wp_distance.append(waypoint_distance(waypoints[i], waypoints[i - 1]))
    wp_distance.append(0)  # last distance is 0 because it is the distance
    # from the last waypoint to the last waypoint

    # Linearly interpolate between waypoints and store in a list
    wp_interp = []  # interpolated values
    # (rows = waypoints, columns = [x, y, v])
    wp_interp_hash = []  # hash table which indexes waypoints_np
    # to the index of the waypoint in wp_interp
    interp_counter = 0  # counter for current interpolated point index

    for i in range(waypoints_np.shape[0] - 1):
        # Add original waypoint to interpolated waypoints list (and append
        # it to the hash table)
        wp_interp.append(list(waypoints_np[i]))
        wp_interp_hash.append(interp_counter)
        interp_counter += 1

        # Interpolate to the next waypoint. First compute the number of
        # points to interpolate based on the desired resolution and
        # incrementally add interpolated points until the next waypoint
        # is about to be reached.
        num_pts_to_interp = int(np.floor(wp_distance[i] / float(INTERP_DISTANCE_RES)) - 1)

        if num_pts_to_interp > 1:
            wp_vector = waypoints_np[i + 1] - waypoints_np[i]
            wp_uvector = wp_vector / np.linalg.norm(wp_vector)
            for j in range(num_pts_to_interp):
                next_wp_vector = INTERP_DISTANCE_RES * float(j + 1) * wp_uvector
                wp_interp.append(list(waypoints_np[i] + next_wp_vector))
                interp_counter += 1

    # add last waypoint at the end
    wp_interp.append(list(waypoints_np[-1]))
    wp_interp_hash.append(interp_counter)
    interp_counter += 1

    return wp_interp, wp_interp_hash
