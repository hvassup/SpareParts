import math

import matplotlib.pyplot as plt

from simulation.visualization.shared import get_lidar_points
from simulation.visualization.visualizer import Visualizer


class MatPlotVisualizer(Visualizer):
    def visualize_lidar(self, robotX, robotY, robotDirection, readings):
        """
        Plot the points where the lidar hits in mat plot lib
        """
        plt.scatter(robotX, robotY)

        points = get_lidar_points(readings)
        for x, y in points:
            plt.scatter(x + robotX, y + robotY)

        plt.show()
