import sys

import pygame

from simulation.visualization.shared import get_lidar_points
from simulation.visualization.visualizer import Visualizer


def scale_point(x, y):
    return 50 + (x + 2) * 50, 50 + (y + 2) * 50


class PyGameVisualizer(Visualizer):
    def __init__(self):
        pygame.init()
        pygame.display.init()
        self.surface = pygame.display.set_mode((320, 240))

    def visualize_lidar(self, robotX, robotY, readings):
        """
        Display the points where the lidar hits in py game
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        points = get_lidar_points(readings)
        self.surface.fill((0, 0, 0))

        pygame.draw.circle(self.surface, (0, 255, 0), scale_point(robotX, robotY), 2)
        for x, y in points:
            pygame.draw.circle(self.surface, (255, 255, 255), scale_point(robotX + x, robotY + y), 2)
        pygame.display.update()
