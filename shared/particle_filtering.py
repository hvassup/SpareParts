import math
from random import random

from shared.state import W, H, april_tag_angles
from shared.util import clamp, rand, angle_difference
from simulation.sensor_sim import get_lidar


def generate_random_particles(n):
    particles = []
    for _ in range(0, n):
        particle = rand(W), rand(H), random() * 360
        particles.append(particle)
    return particles


xy_noise = 0.01
angle_noise = math.radians(50)


# add some noise to an existing state
def add_noise_to_state(s):
    x, y, angle = s
    _W = W * 0.98
    _H = H * 0.98
    return clamp(x + rand(xy_noise), -_W / 2, _W / 2), clamp(y + rand(xy_noise), -_H / 2, _H / 2), angle + rand(
        angle_noise)


# Randomly pick weighted samples
def stochastic_universal_resampling(n_picks, samples, weights):
    indexes = []
    for i in range(0, n_picks):
        indexes += [i] * weights[i]

    P = (len(indexes) / n_picks)
    start = random() * P

    new_samples = []
    i = start
    while i < len(indexes):
        new_samples.append(samples[indexes[int(i)]])
        i += P
    return new_samples


def resample(samples, weights):
    n = len(samples)
    picked_samples = stochastic_universal_resampling(n, samples, weights)
    noisy_samples = list(map(add_noise_to_state, picked_samples))
    return noisy_samples


max_lidar_reading = math.sqrt(H ** 2 + W ** 2)


# Approximate normalization of a weight
def normalize_weight(w, lidar_resolution):
    return int((w / (max_lidar_reading * (lidar_resolution + 1))) ** 2 * 100)


# Simulate sensor readings and compare with actual readings to evaluate state
def compare_states(s1, lidar_reading, april_tags):
    x1, y1, angle = s1

    lidar_resolution = 4
    real_resolution = len(lidar_reading)

    reading1 = get_lidar(x1, y1, angle, lidar_resolution)
    step = real_resolution // lidar_resolution

    weight = 0
    for i in range(0, lidar_resolution):
        weight += abs(reading1[i] - lidar_reading[step * i])
    weight -= lidar_resolution * max_lidar_reading

    # April tag rating
    angle_diff = 0
    if len(april_tags) > 0:
        middle_april_tag = april_tags[round(len(april_tags) / 2)]
        if middle_april_tag[1] != 19:
            desired_angle = april_tag_angles[middle_april_tag[1]]

            angle_diff = math.degrees(abs(angle_difference(desired_angle, angle)))
            if angle_diff < 90:
                weight -= max_lidar_reading

    return normalize_weight(weight, lidar_resolution), angle_diff
