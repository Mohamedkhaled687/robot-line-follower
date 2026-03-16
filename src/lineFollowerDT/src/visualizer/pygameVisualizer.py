"""
pygameVisualizer.py
-------------------

Simple real-time 2D top-down visualization of a line-following robot
using pygame (as suggested by the capstone project specification).

Public API mirrors RealTimePlotter:

    vis = PygameVisualizer(width=800, height=600, update_frequency=10)
    vis.update_data(t, x_robot, y_robot, theta_robot,
                    x_path, y_path, theta_path,
                    v_cmd, omega_cmd, e_lat)
    vis.close()
"""

from __future__ import annotations
import math
from collections import deque

import pygame

WHITE  = (255, 255, 255)
BLACK  = (0, 0, 0)
GREY   = (200, 200, 200)
GREEN  = (0, 180, 80)
BLUE   = (30, 100, 220)
RED    = (220, 50, 50)
CYAN   = (0, 200, 200)
TRAIL  = (140, 180, 255)


class PygameVisualizer:

    def __init__(self, width: int = 800, height: int = 600,
                 update_frequency: int = 10,
                 pixels_per_meter: float = 80.0):
        self.width = width
        self.height = height
        self.update_frequency = update_frequency
        self.ppm = pixels_per_meter
        self._update_counter = 0

        self.trail: deque = deque(maxlen=2000)
        self.path_pts: deque = deque(maxlen=2000)

        self._cam_x = 0.0
        self._cam_y = 0.0

        self._t = 0.0
        self._e_lat = 0.0
        self._v_cmd = 0.0
        self._omega_cmd = 0.0

        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Line-Following Robot — pygame 2D")
        self.font = pygame.font.SysFont("monospace", 16)
        self.clock = pygame.time.Clock()

    def _world_to_screen(self, wx: float, wy: float):
        sx = int((wx - self._cam_x) * self.ppm + self.width / 2)
        sy = int((-wy + self._cam_y) * self.ppm + self.height / 2)
        return sx, sy

    def update_data(self, t, x_robot, y_robot, theta_robot,
                    x_path, y_path, theta_path,
                    v_cmd, omega_cmd, e_lat):
        self._t = t
        self._e_lat = e_lat
        self._v_cmd = v_cmd
        self._omega_cmd = omega_cmd

        self.trail.append((x_robot, y_robot))
        self.path_pts.append((x_path, y_path))

        self._update_counter += 1
        if self._update_counter < self.update_frequency:
            return
        self._update_counter = 0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        self._cam_x = x_robot
        self._cam_y = y_robot

        self.screen.fill(WHITE)

        self._draw_grid()
        self._draw_path()
        self._draw_trail()
        self._draw_path_ref(x_path, y_path)
        self._draw_robot(x_robot, y_robot, theta_robot)
        self._draw_hud()

        pygame.display.flip()
        self.clock.tick(60)

    def close(self):
        pygame.quit()

    def _draw_grid(self):
        step = 1.0
        left = self._cam_x - self.width / (2 * self.ppm) - step
        right = self._cam_x + self.width / (2 * self.ppm) + step
        top = self._cam_y + self.height / (2 * self.ppm) + step
        bottom = self._cam_y - self.height / (2 * self.ppm) - step

        x = math.floor(left)
        while x <= right:
            sx, _ = self._world_to_screen(x, 0)
            pygame.draw.line(self.screen, GREY, (sx, 0), (sx, self.height))
            x += step

        y = math.floor(bottom)
        while y <= top:
            _, sy = self._world_to_screen(0, y)
            pygame.draw.line(self.screen, GREY, (0, sy), (self.width, sy))
            y += step

    def _draw_path(self):
        if len(self.path_pts) < 2:
            return
        screen_pts = [self._world_to_screen(px, py) for px, py in self.path_pts]
        pygame.draw.lines(self.screen, GREEN, False, screen_pts, 3)

    def _draw_trail(self):
        for i, (tx, ty) in enumerate(self.trail):
            alpha = int(80 + 120 * i / max(len(self.trail), 1))
            color = (min(alpha, 180), min(alpha + 40, 220), 255)
            sx, sy = self._world_to_screen(tx, ty)
            pygame.draw.circle(self.screen, color, (sx, sy), 2)

    def _draw_path_ref(self, px, py):
        sx, sy = self._world_to_screen(px, py)
        pygame.draw.circle(self.screen, GREEN, (sx, sy), 7, 2)

    def _draw_robot(self, x, y, theta):
        L = 0.15
        cx, cy = self._world_to_screen(x, y)

        nose = self._world_to_screen(
            x + L * math.cos(theta),
            y + L * math.sin(theta))
        left = self._world_to_screen(
            x + L * 0.5 * math.cos(theta + 2.5),
            y + L * 0.5 * math.sin(theta + 2.5))
        right = self._world_to_screen(
            x + L * 0.5 * math.cos(theta - 2.5),
            y + L * 0.5 * math.sin(theta - 2.5))

        pygame.draw.polygon(self.screen, BLUE, [nose, left, right])
        pygame.draw.polygon(self.screen, BLACK, [nose, left, right], 2)

        hx, hy = self._world_to_screen(
            x + L * 1.4 * math.cos(theta),
            y + L * 1.4 * math.sin(theta))
        pygame.draw.line(self.screen, RED, (cx, cy), (hx, hy), 2)

    def _draw_hud(self):
        lines = [
            f"t = {self._t:.3f} s",
            f"e_lat = {self._e_lat*1000:.1f} mm",
            f"v_cmd = {self._v_cmd:.3f} m/s",
            f"w_cmd = {self._omega_cmd:.3f} rad/s",
        ]
        for i, line in enumerate(lines):
            surf = self.font.render(line, True, BLACK)
            self.screen.blit(surf, (10, 10 + i * 22))
