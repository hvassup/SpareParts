import matplotlib.pyplot as plt
import csv
import math

def visualize():
    with open('trajectory.dat', newline='') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            x, y, z, q = map(lambda x: float(x), row)
            plt.quiver(x, y, z, q)
    plt.xlim(-1, 1)
    plt.ylim(-1, 1)
    plt.show()

def plot_lidar(x, y, readings):
    # Eliminate fucked readings. "if reading is longer than the track - remove it"
    resolution = len(readings)
    for i in range(0, resolution):
        a = math.sin(i) * readings[i]
        b = math.cos(i) * readings[i]
        print(a, b)
    pass