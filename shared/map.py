import numpy as np
from shared.BFS import BFS

# Width and Height of map in CM
MAP_WIDTH = 100
MAP_HEIGHT = 200

# Resolution: How many cm per grid
MAP_RESOLUTION = 5

# Create map of zeros
map = np.zeros((MAP_WIDTH // MAP_RESOLUTION, MAP_HEIGHT // MAP_RESOLUTION))

# Mark coordinate on map as dangerous (mark as a 1)
def mark_as_danger(x, y):
    map[y][x] = 1
    
# Mark coordinate on map as safe (mark as a 0)
def mark_as_safe(x, y):
    map[y][x] = 0

# Return whether coordinate on map is a danger area
def is_danger(x, y):
    if x < 0 or y < 0 or y >= map.shape[0] or x >= map.shape[1]:
        return True
    return map[y][x] == 1

# Get all neighbor fields that are not marked as dangerous
def get_safe_neighbors(x, y):
    neighbor_coords = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]
    neighbors = filter(lambda i: not is_danger(i[0], i[1]), neighbor_coords)
    return neighbors

# Run BFS from a starting location to a target location for example: find_safe_path((0, 0), (50, 50))
# Returns a tuple containing the safe path
def find_safe_path(fromPoint, toPoint):
    return BFS(fromPoint, toPoint, get_safe_neighbors)