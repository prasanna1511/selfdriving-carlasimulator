#!/usr/bin/python3
import carla
import os
import pygame
from pygame.locals import K_r

from sdc_course.utils import Window, World
from sdc_course.utils.utility import get_velocity_kmph, load_params

if __name__ == "__main__":
    # load the parameter from the given parameter file.
    params = load_params("./params.yaml")

    if not os.path.isdir("results"):
        os.mkdir("results")

    # add vehicle
    spawn_position = carla.Transform(carla.Location(-50.664417, 206.753799, 1.0))
    world = World(params, spawn_position)

    trajectory_file = None
    trajectory = []

    try:
        # Common setup
        vehicle = world.get_vehicle()
        vehicle.set_autopilot(False)
        window = Window(world)

        # Track recording states
        save_data = False
        prev_keys = pygame.key.get_pressed()

        while not window.should_close:
            world.tick()  # advance simulation by one timestep.
            keys = pygame.key.get_pressed()

            if keys[K_r] and not prev_keys[K_r]:
                save_data = not save_data

                if save_data:
                    trajectory = []
                    trajectory_file = open("results/recorded_trajectory.txt", "w")
                else:
                    trajectory_file.close()

            prev_keys = keys

            if save_data:
                loc = vehicle.get_transform().location
                velocity = get_velocity_kmph(vehicle)

                trajectory_data = "{:.6f},{:.6f},{:.3f}\n".format(loc.x, loc.y, velocity)
                trajectory_file.write(trajectory_data)
                trajectory.append([loc.x, loc.y, velocity])

            # Update info pane
            with window.get_pane() as pane:
                pane.set_waypoints(trajectory)
                if save_data:
                    pane.add_text(">>> Recording <<<")

            window.update()

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")
        if trajectory_file:
            trajectory_file.close()

    finally:
        print("destroying actors")
        world.destroy()

        if trajectory_file:
            trajectory_file.close()
