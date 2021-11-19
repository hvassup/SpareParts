import math


def activation_function(result):
    return min(max(0, result), 1)

def perceptron(sensors, weights, bias):
    res = 0
    assert len(sensors) == len(weights)
    for i in range(0, len(sensors)):
        res += sensors[i] * weights[i]
    return activation_function(res + bias)