import matplotlib.pyplot as plt


def plot_fitness_history(fitnesses_history):
    for p in fitnesses_history:
        plt.plot(*p, 'o')
    plt.show()