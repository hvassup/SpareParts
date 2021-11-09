import random
import numpy as np
from shared.enums import State, Action

# Learning rate
alpha = 0.1

# epsilon = exploration
epsilon = 1

temperature = 0

# Discount factor
gamma = 0.9

state_size = 4
action_size = 4

# Initialize Q table
Q = np.random.randn(state_size, action_size)

def reward(state, new_state, action) -> int:
    reward = 0

    if state != State.NO and new_state == State.NO:
        reward = 10
    elif state == State.NO and new_state == State.NO:
        if action == Action.F:
            reward = 1
    elif state == State.NO and new_state != State.NO:
        reward = 0
    else:
        reward = -1
    
    if action == Action.B:
        reward -= 0.1
    
    return reward
        # if action == Action.F:
        #     return 100
        # if action == Action.R:
        #     return 50
        # if action == Action.L:
        #     return 50
        # if action == Action.B:
        #     return 10
    # return -10

def update_q_table(state, action, new_state):
    global temperature, epsilon
    temperature = min(temperature + 1, 100)
    epsilon = 1 - (temperature * 0.009)

    Q[state, action] = Q[state, action] + alpha * (reward(state, new_state, action) + gamma * np.max(Q[new_state, :]) - Q[state, action])
    for y in range(0, Q.shape[1]):
        for x in range(0, Q.shape[1]):
            print(round(Q[x][y], 2), end='\t')
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
