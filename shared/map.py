import numpy as np

from shared.BFS import BFS


class Map():
    def __init__(self, MAP_WIDTH, MAP_HEIGHT, MAP_RESOLUTION=5):
        self.WIDTH = int(MAP_WIDTH * 100)
        self.HEIGHT = int(MAP_HEIGHT * 100)
        self.RESOLUTION = int(MAP_RESOLUTION)
        self.map = np.zeros((self.WIDTH // self.RESOLUTION, self.HEIGHT // self.RESOLUTION), dtype=bool)
        self.safe_spots = set()

    def map_cord_to_real(self, x, y):
        return (x * self.RESOLUTION - self.WIDTH / 2) / 100, (y * self.RESOLUTION - self.HEIGHT / 2) / 100

    def real_coordinate_to_map_coordinate(self, x, y):
        x *= 100
        y *= 100
        x += self.WIDTH / 2
        y += self.HEIGHT / 2
        return int(x / self.RESOLUTION), int(y / self.RESOLUTION)

    # Mark coordinate on map as dangerous (mark as a 1)
    def mark_as_danger(self, realX, realY):
        x, y = self.real_coordinate_to_map_coordinate(realX, realY)
        self.map[x][y] = 1
        self.safe_spots.add(self.map_cord_to_real(x, y))

    # Mark coordinate on map as safe (mark as a 0)
    def mark_as_safe(self, realX, realY):
        x, y = self.real_coordinate_to_map_coordinate(realX, realY)
        self.map[x][y] = 0

    # Return whether coordinate on map is a danger area
    def is_danger(self, x, y):
        if x < 0 or y < 0 or y >= self.map.shape[1] or x >= self.map.shape[0]:
            return True
        return self.map[x][y] == 1

    # Get all neighbor fields that are not marked as dangerous
    def get_safe_neighbors(self, x, y):
        neighbor_coords = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]
        neighbors = filter(lambda i: not self.is_danger(i[0], i[1]), neighbor_coords)
        return neighbors

    # Run BFS from a starting location to a target location for example: find_safe_path((0, 0), (50, 50))
    # Returns a tuple containing the safe path
    def find_safe_path(self, _fromPoint, _toPoint):
        fromPoint = self.real_coordinate_to_map_coordinate(*_fromPoint)
        toPoint = self.real_coordinate_to_map_coordinate(*_toPoint)
        path = BFS(fromPoint, toPoint, self.get_safe_neighbors)

        return list(map(lambda x: self.map_cord_to_real(*x), path))
