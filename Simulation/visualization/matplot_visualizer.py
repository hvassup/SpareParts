import math

import matplotlib.pyplot as plt

from Simulation.visualization.shared import get_lidar_points
from Simulation.visualization.visualizer import Visualizer


class MatPlotVisualizer(Visualizer):
    def visualize_lidar(self, robotX, robotY, readings):
        """
        Plot the points where the lidar hits in mat plot lib
        """
        plt.scatter(robotX, robotY)

        points = get_lidar_points(readings)
        for x, y in points:
            plt.scatter(x + robotX, y + robotY)

        plt.show()
