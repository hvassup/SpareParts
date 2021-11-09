import random
import numpy as np
from shared.enums import State, Action

# Learning rate
alpha = 0.5

# epsilon = exploration
epsilon = 0.25

# Discount factor
gamma = 0.9

state_size = 4
action_size = 4

# Initialize Q table
Q = np.zeros((state_size, action_size))

def reward(state, action) -> int:
    if state == State.NO:
        if action == Action.F:
            return 100
        if action == Action.R:
            return 50
        if action == Action.L:
            return 50
        if action == Action.B:
            return 10
    return -10

def update_q_table(state, action, new_state):
    Q[state, action] = Q[state, action] + alpha * (reward(new_state, action) + gamma * np.max(Q[new_state, :]) - Q[state, action])
    for y in range(0, Q.shape[1]):
        for x in range(0, Q.shape[1]):
            print(Q[x][y], end='\t')
        print()


def get_next_action(state):
    if random.uniform(0, 1) < epsilon:
        """
        Explore: select a random action
        """
        possible_state = []
        for i in range(0, action_size):
            if Q[state, i] == 0:
                possible_state.append(i)

        if len(possible_state) > 0:
            return random.choice(possible_state)

        return random.randint(0, action_size - 1)

    else:
        """
        Exploit: select the action with max value (future reward)
        """
        return np.argmax(Q[state, :])
