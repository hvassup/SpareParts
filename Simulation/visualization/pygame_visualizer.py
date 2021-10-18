import sys
from matplotlib.pyplot import sca
import numpy as np
import pygame
import cv2
import math

from shapely.geometry.point import Point
from shared.util import deg_to_rad
from simulation.visualization.shared import get_lidar_points
from simulation.visualization.visualizer import Visualizer

offset = 150
scale = 10
def scale_point(x, y):
    return offset + (x) * scale, offset + (y) * scale

def scale_size(w, h):
    return w * scale, h * scale

def show_triangulation(points, distances):
    import matplotlib.pyplot as plt
    offset = 0.02
    n = len(points)
    a = Point(*points[n // 3]).buffer(distances[n // 3] + offset)
    b = Point(*points[n * 2 // 3]).buffer(distances[n * 2 // 3] + offset)
    c = Point(*points[0]).buffer(distances[0] + offset)

    plt.plot(*a.exterior.xy)
    print(a.centroid)
    plt.plot(*b.exterior.xy)
    print(b.centroid)
    # plt.plot(*c.exterior.xy)
    pos = a.intersection(b).intersection(c).centroid
    plt.plot(*pos.xy)
    print('Estimated position:', pos)
    x, y = points[n // 3]
    x1, y1 = pos.coords[0]
    # plt.show()
    return (x - x1, y - y1)


class PyGameVisualizer(Visualizer):
    def __init__(self):
        pygame.init()
        pygame.display.init()
        self.surface = pygame.display.set_mode((600, 300))

    def draw_text(self, msg, point):
        font1 = pygame.font.SysFont('chalkduster.ttf', 16)
        img = font1.render(msg, True, (255, 255, 255))
        self.surface.blit(img, point)

    def clear(self):
        self.surface.fill((0, 0, 0))
    
    def show(self):
        pygame.display.update()
    
    def draw_rect(self, color, pos, size):
        rect = scale_point(*pos) + scale_size(*size)
        pygame.draw.rect(self.surface, color, rect)
    
    def visualize_danger_spots(self, spots):
        for pos, size in spots:
            self.draw_rect((255, 0, 0), pos, size)

    def visualize_safe_zone(self, pos, size):
        self.draw_rect((0, 255, 0), pos, size)
    
    def draw_line_to_point(self, x, y, robotX, robotY):
        pygame.draw.line(self.surface, (255, 0, 255), scale_point(robotX, robotY), scale_point(x + robotX, y + robotY))
    
    def get_point_at_rotation(self, angle, points, robotX, robotY):
        point_idx = round(angle / 360 * len(points)) % len(points)
        point = points[point_idx]
        self.draw_line_to_point(*point, robotX, robotY)

    def visualize_lidar(self, robotX, robotY, robotDirection, readings):
        """
        Display the points where the lidar hits in py game
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        points = get_lidar_points(readings, robotDirection)

        rotated_points = get_lidar_points(readings, 0)

        pos = show_triangulation(rotated_points, readings)

        center, width_height, angle = cv2.minAreaRect(np.asarray(get_lidar_points(readings, 0)).astype(np.int))
        print(center, width_height)
        angle_rad = deg_to_rad(angle)

        self.draw_text(f'Rectangle angle: {angle}', (300, 10))
        self.draw_text(f'Estimated pos: {pos}', (300, 30))

        # self.get_point_at_rotation(angle, points, robotX, robotY)
        # self.get_point_at_rotation(angle + 90, points, robotX, robotY)
        # self.get_point_at_rotation(angle + 180, points, robotX, robotY)
        # self.get_point_at_rotation(angle + 270, points, robotX, robotY)
        # pygame.draw.line(self.surface, (255, 0, 255), scale_point(robotX, robotY), scale_point(robotX + math.cos(angle_rad), robotY +  + math.sin(angle_rad)))


        # pygame.draw.polygon(self.surface, (0, 0, 0), ((0, 100), (0, 200), (200, 200), (200, 300), (300, 150), (200, 0), (200, 100)))

        pygame.draw.circle(self.surface, (0, 255, 0), scale_point(robotX, robotY), 2)
        for x, y in points:
            pygame.draw.circle(self.surface, (255, 255, 255), scale_point(robotX + x, robotY + y), 2)
        
    def visualize_world(self, world, height, width):
        # y_height = height / world.shape[0]
        # x_width = width / world.shape[1]

        # for y in range(0, world.shape[0]):
        #     for x in range(0, world.shape[1]):
        #         self.draw_rect((0, 0, 255), (3 - width/2 + x * x_width, -height/2 + y * y_height), (x_width, y_height))
        self.draw_rect((0, 0, 255), (-width/2, -height/2), (width, height))
