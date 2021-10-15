import sys
from matplotlib.pyplot import sca

import pygame

from simulation.visualization.shared import get_lidar_points
from simulation.visualization.visualizer import Visualizer

offset = 150
scale = 100
def scale_point(x, y):
    return offset + (x) * scale, offset + (y) * scale

def scale_size(w, h):
    return w * scale, h * scale

class PyGameVisualizer(Visualizer):
    def __init__(self):
        pygame.init()
        pygame.display.init()
        self.surface = pygame.display.set_mode((300, 300))
    
    def clear(self):
        self.surface.fill((0, 0, 0))
    
    def show(self):
        pygame.display.update()

    def visualize_safe_zone(self, pos, size):
        rect = scale_point(*pos) + scale_size(*size)
        pygame.draw.rect(self.surface, (0, 255, 0), rect)
        pass

    def visualize_lidar(self, robotX, robotY, robotDirection, readings):
        """
        Display the points where the lidar hits in py game
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        points = get_lidar_points(readings, robotDirection)

        pygame.draw.circle(self.surface, (0, 255, 0), scale_point(robotX, robotY), 2)
        for x, y in points:
            pygame.draw.circle(self.surface, (255, 255, 255), scale_point(robotX + x, robotY + y), 2)
        
    def visualize_world(self, world, height, width):
        rect = scale_point(-width/2, -height/2) + scale_size(width, height)
        pygame.draw.rect(self.surface, (0, 0, 255), rect)
