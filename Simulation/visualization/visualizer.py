"""
Interface for visualizing things
"""


class Visualizer:
    def visualize_lidar(self, robotX, robotY, robotDirection, readings):
        """
        :param robotX: robots X position
        :param robotY: robots Y position
        :param readings: lidar distances
        """
        raise Exception("Not yet implemented!")

    def clear(self):
        raise Exception("Not yet implemented!")
    
    def show(self):
        raise Exception("Not yet implemented!")

    def visualize_world(self, world):
        raise Exception("Not yet implemented!")

    def visualize_safe_zone(self, pos, size):
        raise Exception("Not yet implemented!")

    def visualize_danger_spots(self, pos, size):
        raise Exception("Not yet implemented!")
