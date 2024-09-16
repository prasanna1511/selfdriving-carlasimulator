#!/usr/bin/python3
import carla
import time
import math
import numpy as np
from sdc_course.utils import Window, World
from sdc_course.utils.utility import load_params

from sdc_course.control import DecomposedController, ModelPredictiveController
from sdc_course.planning.global_planner import GlobalPlanner
from sdc_course.planning.behavior_planner import BehaviorPlanner
from sdc_course.planning.local_planner import LocalPlanner
from sdc_course.perception.perception import Perception

if __name__ == "__main__":
    # load the parameter from the given parameter file.
    params = load_params("params.yaml")

    # Set start and end position according to yaml file
    start = carla.Location(*params["planning"]["start"])
    end = carla.Location(*params["planning"]["end"])
    spawn_position = carla.Transform(start, carla.Rotation(yaw=-90))

    # Create world and get map
    world = World(params, spawn_position, params["planning"]["scenario"])
    world_map = world.get_world_map()

    # Init global planner and plan global path
    global_planner = GlobalPlanner(world_map, params)
    global_path = global_planner.get_global_path(start, end)

    # Init behavior planner
    behavior_planner = BehaviorPlanner(global_planner)

    # Setup local planner
    local_planner = LocalPlanner(global_planner, behavior_planner, global_path, params)

    # Setup dummy perception system
    perception = Perception(world)

    debug = world.get_debug_helper()

    # record tracked trajectory
    trajectory = []
    trajectory_file = open("tracked_trajectory.txt", "w")

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

            parked_cars = perception.detect_parked_cars()
            global_path = global_planner.get_global_path(start, end)
            local_planner.update_global_path(global_path)

            window.get_pane().set_waypoints(global_path)

            # Update behavior planner if needed
            behavior_planner.update(vehicle, parked_cars)

            # get trajectory information
            target_speed = local_planner.get_velocity_profile(vehicle)
            local_trajectories = local_planner.get_local_path(vehicle, parked_cars)
            target_trajectory = local_trajectories.pop(0)

            # Pass to controller
            control = controller.compute_control(target_speed, target_trajectory)
            vehicle.apply_control(control)

            # Update info pane
            with window.get_pane() as pane:
                pane.add_text("Since start: {:5.1f} s".format(time.time() - start_time))
                pane.add_text("Controller:  {:>8s}".format(params["control"]["strategy"]))
                pane.add_image(sensor_data["rgb"])

            # Visualize local trajectory
            for traj in local_trajectories:
                for i, point in enumerate(traj):
                    loc = carla.Location(
                        point[0], point[1], vehicle.get_transform().location.z + 1.0
                    )
                    debug.draw_point(loc, size=0.05, life_time=0.1, color=carla.Color(255, 0, 0))

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
