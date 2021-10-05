import matplotlib.pyplot as plt
import csv

def visualize():
    with open('trajectory.dat', newline='') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            x, y, z, q = map(lambda x: float(x), row)
            plt.quiver(x, y, z, q)
    plt.xlim(-1, 1)
    plt.ylim(-1, 1)
    plt.show()