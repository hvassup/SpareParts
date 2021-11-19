from math import sqrt
import random
from typing import NewType
import numpy as np

# Randomly pick weighted samples
def stochastic_universal_resampling(n_picks, samples, weights):
    indexes = []
    for i in range(0, n_picks):
        indexes += [i] * weights[i]

    P = (len(indexes) / n_picks)
    start = random.random() * P

    new_samples = []
    i = start
    while i < len(indexes):
        new_samples.append(samples[indexes[int(i)]])
        i += P
    return new_samples

def generate_random_genome(genome_size=10):
    return np.random.uniform(low=-1, high=1, size=genome_size)

def generate_random_population(n):
    return [generate_random_genome() for _ in range(0, n)]

def calc_fitness(left_wheel, right_wheel, sensors, is_inside):
    if not is_inside:
        return -10
    # (-number of times robot bumped into wall)
    # OR
    # Accumulated wheel speed & Low sensor values & Symmetrical wheel speeds
    diff = abs(left_wheel - right_wheel)
    V = (abs(left_wheel) + abs(right_wheel)) / 2
    max_sensor_value = max(sensors) / 5020
    return V * (1 - sqrt(diff)) * (1 - max_sensor_value)

def rank_based_selection(population, fitnesses):
    indexes = np.argsort(fitnesses)
    weights = []
    
    for i, idx in enumerate(indexes):
        n_count = (i + 1) ** 2
        for _ in range(0, n_count):
            weights.append(idx)
    
    picks = [population[indexes[-1]]]
    for _ in range(0, len(population) - 1):
        rand_idx = random.choice(weights)
        picks.append(population[rand_idx])
    
    return picks

def elitism(population, fitnesses, n):
    """
    Take n best genomes
    Keep n best for next generation [In case mutations suck (inbreeding)]
    and mutate copies of the n best for the rest of the population
    """
    n_best = np.argpartition(fitnesses, -n)[-n:]
    new_population = []

    for i in n_best:
        print('Keeping:', i, fitnesses[i])
        new_population.append(np.copy(population[i]))
        for _ in range(1, int(len(population) / n)):
            new_population.append(np.copy(population[i]))
    
    assert len(new_population) == len(population)

    return new_population

## Could potentially be changed to support multi-mutation
def mutate_single(genome):
    idx = random.randint(0, len(genome) - 1)
    genome[idx] = np.random.uniform(low=-1, high=1)
    return genome

def crossover_pair(p1, p2):
    idx = random.randint(0, len(p1) - 1)
    return np.concatenate((p1[:idx], p2[idx:])), np.concatenate((p2[:idx], p1[idx:]))

def crossover(population, crossover_rate=0.7, mutation_prob=0.2):
    random.shuffle(population)
    ## Should be randomly selected
    for i in range(0, len(population), 2):
        if random.random() <= crossover_rate:
            p1, p2 = crossover_pair(population[i], population[i + 1])
            if random.random() <= mutation_prob:
                population[i] = mutate_single(p1)
                population[i + 1] = mutate_single(p2)
            else:
                population[i] = p1
                population[i + 1] = p2

    return population