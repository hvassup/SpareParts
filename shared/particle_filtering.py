from random import random

def generate_particles(W, H, n=50):
    particles = []
    for _ in range(0, 50):
        particle = (random() * W - W/2, random() * H - H/2)
        particles.append(particle)
    return particles

def compare_states(s1, s2):
    pass