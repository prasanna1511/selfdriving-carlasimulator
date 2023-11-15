#!/usr/bin/python3
import carla
import time
import math
import os
import numpy as np
from sdc_course.utils import Window, World
from sdc_course.utils.utility import load_params

from sdc_course.control import DecomposedController, ModelPredictiveController
from sdc_course.planning.simple_global_planner import SimpleGlobalPlanner
from sdc_course.planning.simple_local_planner import SimpleLocalPlanner

if __name__ == "__main__":
    # load the parameter from the given parameter file.
    params = load_params("params.yaml")

    # Load fixed path
    global_planner = SimpleGlobalPlanner(params)
    global_path = global_planner.get_global_path()

    # spawn car at the begining of the track
    spawn_position = carla.Transform(
        carla.Location(x=global_path[0][0], y=global_path[0][1], z=1.0)
    )
    world = World(params, spawn_position)

    # Setup local planner
    local_planner = SimpleLocalPlanner(global_path)

    debug = world.get_debug_helper()

    # record tracked trajectory
    if not os.path.isdir("results"):
        os.mkdir("results")

    trajectory = []
    trajectory_file = open("results/tracked_trajectory.txt", "w")

    try:
        # Common setup
        vehicle = world.get_vehicle()
        vehicle.set_autopilot(True)

        window = Window(world)
        window.get_pane().set_waypoints(global_path)

        if params["control"]["strategy"] == "mpc":
            controller = ModelPredictiveController(vehicle, params)
        else:
            controller = DecomposedController(vehicle, params)

        start_time = time.time()
        while not window.should_close:
            world.tick()  # advance simulation by one timestep.
            sensor_data = vehicle.get_sensor_data()

            # get trajectory information
            target_speed = local_planner.get_velocity_profile(vehicle)
            target_trajectory = local_planner.get_local_path(
                vehicle,
                max_len=params["planning"]["max_len"],
                min_distance=params["planning"]["min_distance"],
            )

            # Pass to controller
            control = controller.compute_control(target_speed, target_trajectory)
            vehicle.apply_control(control)

            # Update info pane
            with window.get_pane() as pane:
                pane.add_text("Since start: {:5.1f} s".format(time.time() - start_time))
                pane.add_text("Controller:  {:>8s}".format(params["control"]["strategy"]))
                pane.add_image(sensor_data["rgb"])

            # Visualize local trajectory
            for i, point in enumerate(target_trajectory):
                loc = carla.Location(point[0], point[1], vehicle.get_transform().location.z + 1.0)
                debug.draw_point(loc, size=0.05, life_time=0.1, color=carla.Color(0, 255, 0))

            window.update()

            if window.get_target_location() is not None:
                start = vehicle.get_transform().location
                end = carla.Location(*window.get_target_location())

                global_path = global_planner.get_global_path(start, end)
                local_planner.update_global_path(global_path)

                window.get_pane().set_waypoints(global_path)

            # save trajectory to disk
            pose = vehicle.get_transform()
            vel = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)  # in km/h
            trajectory_data = "{:.6f},{:.6f},{:.3f}\n".format(
                pose.location.x, pose.location.y, speed
            )
            trajectory_file.write(trajectory_data)
            trajectory.append([pose.location.x, pose.location.y, speed])

    except KeyboardInterrupt:
        trajectory_file.close()
        print("\nCancelled by user. Bye!")

    finally:
        print("destroying actors")
        world.destroy()
