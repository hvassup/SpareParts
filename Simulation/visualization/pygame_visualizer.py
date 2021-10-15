import sys

import pygame

from simulation.visualization.shared import get_lidar_points
from simulation.visualization.visualizer import Visualizer

offset = 150
scale = 100
def scale_point(x, y):
    return offset + (x) * scale, offset + (y) * scale


class PyGameVisualizer(Visualizer):
    def __init__(self):
        pygame.init()
        pygame.display.init()
        self.surface = pygame.display.set_mode((300, 300))

    def visualize_lidar(self, robotX, robotY, robotDirection, readings):
        """
        Display the points where the lidar hits in py game
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        points = get_lidar_points(readings, robotDirection)
        self.surface.fill((0, 0, 0))

        x1, y1 = scale_point(-1, -1)
        x2, y2 = scale_point(1, 1)
        pygame.draw.rect(self.surface, (0, 0, 255), (x1, y1, x2 - x1, y2 - y1))

        pygame.draw.circle(self.surface, (0, 255, 0), scale_point(robotX, robotY), 2)
        for x, y in points:
            pygame.draw.circle(self.surface, (255, 255, 255), scale_point(robotX + x, robotY + y), 2)
        pygame.display.update()
