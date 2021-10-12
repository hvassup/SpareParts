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
    print(cv2.minAreaRect(np.asarray(points).astype(np.int)))
    plt.show()