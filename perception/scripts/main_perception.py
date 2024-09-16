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

from sdc_course.perception.traffic_sign_map import TrafficSignMap
from sdc_course.perception.utils import image_to_world

if __name__ == "__main__":
    # load the parameter from the given parameter file.
    params = load_params("params.yaml")

    # Set start and end position according to yaml file
    start = carla.Location(*params["perception"]["start"])
    end = carla.Location(*params["perception"]["end"])
    spawn_position = carla.Transform(start, carla.Rotation(yaw=-180))

    # Create world and get map
    world = World(params, spawn_position, params["perception"]["scenario"])
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
    traffic_sign_map = TrafficSignMap()

    # threshold for a stabilized traffic sign in the traffic_sign_map
    confidence_threshold = 0.95

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

            # 1. Detect traffic signs.
            sign_detections = perception.detect_traffic_signs()

            # 2. localize the detected traffic sing in the world.
            for detection in sign_detections:
                #######################################################################
                ######################### TODO: IMPLEMENT THIS ########################
                #######################################################################
                # Determine distance from depth image and update traffic_sign_map with detection.
                # Hint: See https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera
                img_depth = vehicle.get_sensor_data()["depth"]

                distance_meters = 0
                point_world = None


                if point_world is None:
                    point_world = detection.world_position

                traffic_sign_map.update_map(
                    point_world[:2], detection.category, detection.confidence
                )

            # 3. update planner for each confident traffic sign.
            for traffic_sign in traffic_sign_map.get_traffic_signs():
                if np.max(traffic_sign.distribution) > confidence_threshold:
                    if traffic_sign.integrated:
                        continue

                    global_planner.integrate_traffic_sign(
                        traffic_sign.position, traffic_sign.category
                    )
                    traffic_sign.integrated = True
                    # global_planner.plot("graphs.png")  # plot cost and speed_limits

            global_path = global_planner.get_global_path(start, end)
            local_planner.update_global_path(global_path)

            window.get_pane().set_waypoints(global_path)

            # Update behavior planner if needed
            behavior_planner.update(vehicle, parked_cars)

            # get trajectory information
            target_speed = local_planner.get_velocity_profile(vehicle)
            local_trajectories = local_planner.get_local_path(vehicle, parked_cars)
            target_trajectory = local_trajectories[0]

            # Pass to controller
            control = controller.compute_control(target_speed, target_trajectory)
            vehicle.apply_control(control)

            # Update info pane
            with window.get_pane() as pane:
                pane.add_text("Since start: {:5.1f} s".format(time.time() - start_time))
                pane.add_text("parked_cars: {:>8d}".format(len(parked_cars)))

                pane.add_text("Detections:")
                pane.add_image(sensor_data["rgb"])

                #######################################################################
                ######################### TODO: IMPLEMENT THIS ########################
                #######################################################################
                # TODO: draw all detection using pane.add_bounding_box
                # TODO: output text.
                for detection in sign_detections:
                    x = detection.x - 0.5 * detection.width 
                    y = detection.y - 0.5 * detection.height 
                    width = detection.width
                    height = detection.height
                    confidence = detection.confidence
                    category = detection.category
                    window.get_pane().add_bounding_box(x, y , width , height)
                    
                    window.get_pane().add_text("x: {:>8f}".format(x))
                    window.get_pane().add_text("y: {:>8f}".format(y))
                    window.get_pane().add_text("width: {:>8f}".format(width))
                    window.get_pane().add_text("height: {:>8f}".format(height))
                    window.get_pane().add_text("category: {:>8s}".format(category))
                    window.get_pane().add_text("confidence: {:>8f}".format(confidence))
                


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
