from random import random


def generate_random_danger_spots(W, H):
    size = (15, 30)
    spots = []
    for i in range(0, 6):
        pos = (- W / 2 + random() * (W - size[0]*2), - H / 2 + random() * (H - size[1]*2))
        spots.append((pos, size))
    return spots