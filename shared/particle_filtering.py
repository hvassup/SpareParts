import math
from random import randint, random
from shared.state import W, H
from shared.util import clamp, rand
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
    return clamp(x + rand(xy_noise), -_W/2, _W/2), clamp(y + rand(xy_noise), -_H/2, _H/2), angle + rand(angle_noise)

# Resample
def stochastic_universal_resampling(n_picks, samples, weights):
    a = []
    for i in range(0, n_picks):
        a += ([i] * weights[i])
    
    P = (len(a) // n_picks)
    start = randint(0, P)
    
    return [samples[a[i]] for i in range(start, len(a), P)]

# This is possible to do in O(N log N)
# but now it is done in O(N^2) :(
def resample(samples, weights):
    n = len(samples)    
    picked_samples = stochastic_universal_resampling(n, samples, weights)
    noisy_samples = list(map(add_noise_to_state, picked_samples))
    return noisy_samples

max_lidar_reading = math.sqrt(H**2 + W**2)

# Approximate normalization of a weight
def normalize_weight(w, lidar_resolution):
    return int((w / (max_lidar_reading * lidar_resolution)) ** 2 * 100)

# Simulate sensor readings and compare with actual readings to evaluate state
def compare_states(s1, lidar_reading):
    x1, y1, angle1 = s1

    lidar_resolution = 4
    real_resolution = len(lidar_reading)

    reading1 = get_lidar(x1, y1, angle1, lidar_resolution)
    
    weight = 0
    for i in range(0, lidar_resolution):
        weight += max_lidar_reading - abs(reading1[i] - lidar_reading[real_resolution // lidar_resolution * i])
    return normalize_weight(weight, lidar_resolution)
    