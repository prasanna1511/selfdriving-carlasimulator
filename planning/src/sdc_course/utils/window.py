""" PyGame window for visualization of the simulation state. """

import math
import os
from typing import Callable, List, Tuple
import numpy as np

import pygame
from pygame.constants import K_c
from pygame.locals import K_a, K_d, K_s, K_w

from .world import World
from .car import Car


class Pane:
    """
    Widget for displaying information on-top of the window.

    Usage:

    with window.get_pane() as pane:
      pane.add_text("test")


    """

    def __init__(self, x: int, y: int, width: int, height: int):
        self._vehicle_state = {}
        self._position = (x, y)
        self._size = (width, height)
        self._waypoints = []

        # font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = "courier" if os.name == "nt" else "mono"
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = "ubuntumono"
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == "nt" else 14)

        self._vehicle_state = {"location": (0, 0, 0)}
        self._items = []
        self._image = None
        self._bboxes = []
        package_path = os.path.dirname(os.path.abspath(__file__))

        self._map_image = pygame.image.load(os.path.join(package_path, "town03_map.png"))
        self._map_dimensions = (0, 0, 0, 0, 0.0)

    def set_vehicle_state(self, vehicle: Car):
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        control = vehicle.get_control()

        self._vehicle_state["location"] = (
            transform.location.x,
            transform.location.y,
            transform.location.z,
        )
        self._vehicle_state["yaw"] = transform.rotation.yaw
        self._vehicle_state["speed"] = 3.6 * math.sqrt(
            velocity.x**2 + velocity.y**2 + velocity.z**2
        )
        self._vehicle_state["control"] = (control.throttle, control.steer)

    def __enter__(self):
        self._items.clear()
        self._image = None
        self._bboxes = []

        return self

    def __exit__(self, *args, **kwargs):
        pass

    def clear_text(self):
        """remove all text from the pane."""
        self._items.clear()

    def add_text(self, text: str):
        """add given text to a new line of the pane."""
        self._items.append(text)

    def add_image(self, image: np.array):
        """add image"""
        self._image = pygame.surfarray.make_surface(np.swapaxes(image, 0, 1))

    def add_bounding_box(self, x: int, y: int, width: int, height: int):
        """add bounding box at upper-left corner (x,y) with given width and height."""
        self._bboxes.append((x, y, width, height))

    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """set visualized trajectory to the given waypoints."""
        self._waypoints = waypoints

    def get_map_dimensions(self) -> Tuple[int, int, int, int, float]:
        """returns x,y,width,height,factor"""
        return self._map_dimensions

    def draw(self, display):
        info_surface = pygame.Surface(self._size)
        info_surface.set_alpha(100)
        display.blit(info_surface, self._position)

        w_img, h_img = self._map_image.get_size()
        overview_map = self._map_image.copy()

        for point in self._waypoints:
            pygame.draw.circle(
                overview_map, (0, 255, 0), (int(268 + 1.4 * point[0]), int(324 + 1.4 * point[1])), 2
            )

        px, py, pz = self._vehicle_state["location"]
        pygame.draw.circle(
            overview_map, (255, 0, 0), (int(268 + 1.4 * px), int(324 + 1.4 * py)), 10
        )

        scaled_size = (int(self._size[0]), int(self._size[0] / w_img * h_img))
        scaled_map = pygame.transform.scale(overview_map, scaled_size)
        display.blit(scaled_map, self._position)
        self._map_dimensions = (*self._position, *scaled_size, w_img / self._size[0])

        v_offset = scaled_map.get_height() + 2

        surface = self._font_mono.render(
            "{:10s} ({:3.1f}, {:3.1f}, {:3.1f})".format(
                "Position:", *self._vehicle_state["location"]
            ),
            True,
            (255, 255, 255),
        )
        display.blit(surface, (8, v_offset))
        v_offset += 18

        surface = self._font_mono.render(
            "{:5s} {:3.1f}".format("Yaw:", self._vehicle_state["yaw"]), True, (255, 255, 255)
        )
        display.blit(surface, (8, v_offset))
        v_offset += 18

        surface = self._font_mono.render(
            "{:10s} {:3.3f} km/h".format("Speed:", self._vehicle_state["speed"]),
            True,
            (255, 255, 255),
        )
        display.blit(surface, (8, v_offset))
        v_offset += 18

        surface = self._font_mono.render(
            "{:10s} ({:3.3f},{:3.3f})".format("Control:", *self._vehicle_state["control"]),
            True,
            (255, 255, 255),
        )
        display.blit(surface, (8, v_offset))
        v_offset += 18

        if self._image:
            w_img, h_img = self._image.get_size()
            image_copy = self._image.copy()

            for box in self._bboxes:
                pygame.draw.rect(image_copy, (255, 0, 0), box, 5)

            scaled_size = (int(self._size[0]), int(self._size[0] / w_img * h_img))
            scaled_image = pygame.transform.scale(image_copy, scaled_size)
            display.blit(scaled_image, (0, v_offset))

            v_offset += scaled_image.get_height() + 2

        for it in self._items:
            surface = self._font_mono.render(it, True, (255, 255, 255))
            display.blit(surface, (8, v_offset))
            v_offset += 18


class Window:
    """
    Window for visualization.
    """

    def __init__(self, world: World, width: int = 800, height: int = 600):
        self._world = world
        self._should_close = False
        self._view_name = "observer"

        pygame.init()
        pygame.font.init()

        self._width = width
        self._height = height

        self._initialize_font()
        self._clock = pygame.time.Clock()
        self._display = pygame.display.set_mode(
            (self._width, self._height), pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        self._world.spawn_observer(self._width, self._height)

        self._kb_callbacks = []
        self._pane = Pane(0, 0, 220, self._height)

        self._prev_keys = pygame.key.get_pressed()
        self._current_mode = 0
        self.target_location = None

    def __del__(self):
        pygame.quit()

    def _initialize_font(self):
        fonts = pygame.font.get_fonts()
        default_font = "ubuntumono"
        font = default_font if default_font in fonts else fonts[0]
        font = pygame.font.match_font(font)
        self._font = pygame.font.Font(font, 14)

    def register_keyboard_callback(self, callback: Callable[[int], int]):
        """register custom handler for keyboard commands."""
        self._kb_callbacks.append(callback)

    @property
    def should_close(self):
        """should the window close due to keyboard or mouse interaction."""
        return self._should_close

    def set_view(self, name: str):
        """set name of the view that should be rendered. Default: observer"""
        self._view_name = name

    def get_pane(self):
        """get window pane on the left side of the screen."""
        return self._pane

    def set_trajectory(self, waypoints: List[Tuple[float, float]]):
        """set trajectory to the given waypoints."""
        self._pane.set_waypoints(waypoints)

    def get_target_location(self):
        return self.target_location

    def _updated_target_location(self, pos: Tuple[int, int]):
        map_x, map_y, map_w, map_h, factor = self._pane.get_map_dimensions()
        px = pos[0] - map_x
        py = pos[1] - map_y

        if px < 0 or py < 0 or px > map_w or py > map_h:
            self.target_location = None
            return

        px = factor * px
        py = factor * py

        self.target_location = [(px - 268) / 1.4, (py - 324) / 1.4]

    def update(self):
        """update window state, capture window events, and trigger repaint."""

        self.target_location = None

        keys = pygame.key.get_pressed()
        event_consumed = False
        for callback in self._kb_callbacks:
            if callback(keys):
                event_consumed = True
                break

        ## process other events.
        if not event_consumed:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._should_close = True

                if event.type == pygame.MOUSEBUTTONUP:
                    self._updated_target_location(pygame.mouse.get_pos())

            if keys[K_c] and not self._prev_keys[K_c]:
                self._current_mode = (self._current_mode + 1) % 3
                self._world.set_observer_mode(self._current_mode)

            # manual control.
            car = self._world.get_vehicle()
            if not car.autopilot_engaged:
                control = car.get_control()
                if keys[K_w]:
                    control.throttle = min(1, control.throttle + 0.05)
                    control.reverse = False
                elif keys[K_s]:
                    control.throttle = min(1, control.throttle + 0.05)
                    control.reverse = True
                else:
                    control.throttle = 0
                if keys[K_a]:
                    control.steer = max(-1.0, min(control.steer - 0.05, 0))
                elif keys[K_d]:
                    control.steer = min(1.0, max(control.steer + 0.05, 0))
                else:
                    control.steer = 0
                # control.hand_brake = keys[K_SPACE]
                # if keys[K_SPACE] and not self._prev_keys[K_SPACE]:
                #   self.save_data = True

                car.apply_control(control)

        self._prev_keys = keys

        array = self._world.get_view(self._view_name)
        image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        self._display.blit(image_surface, (0, 0))

        self._pane.set_vehicle_state(self._world.get_vehicle())
        self._pane.draw(self._display)

        pygame.display.flip()
        pygame.event.pump()
