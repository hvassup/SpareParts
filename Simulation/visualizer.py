from os import read
import matplotlib.pyplot as plt
import csv
import math
import cv2
import numpy as np

def visualize():
    with open('trajectory.dat', newline='') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            x, y, z, q = map(lambda x: float(x), row)
            plt.quiver(x, y, z, q)
    plt.xlim(-1, 1)
    plt.ylim(-1, 1)
    plt.show()


def get_lidar_points(readings):
    resolution = len(readings)
    points = []
    for i in range(0, resolution):
        angle = i / 180 * math.pi # Convert to radians
        _x = math.cos(angle) * readings[i]
        _y = math.sin(angle) * readings[i]
        points.append((float(_x), float(_y)))
    return points

# fuction to visualise lidar readings
def plot_lidar(x, y, readings, world):
    # Eliminate fucked readings. "if reading is longer than the track - remove it"
    plt.scatter(x, y)
    plt.plot(*world.xy)
    resolution = len(readings)
    points = []
    for i in range(0, resolution):
        angle = i / 180 * math.pi # Convert to radians
        _x = math.cos(angle) * readings[i]
        _y = math.sin(angle) * readings[i]
        plt.scatter(x - _x, y - _y)
        points.append((float(_x), float(_y)))
    
    plt.show()


def scale_point(x, y):
    return ((x + 2) * 50, (y + 2) * 50)


import pygame

def show_lidar(robotX, robotY, readings, surface):
    points = get_lidar_points(readings)
    surface.fill((0, 0, 0))

    pygame.draw.circle(surface, (0, 255, 0), scale_point(robotX, robotY), 2)
    for x, y in points:
        pygame.draw.circle(surface, (255, 255, 255), scale_point(robotX - x, robotY - y), 2)
    pygame.display.update()
