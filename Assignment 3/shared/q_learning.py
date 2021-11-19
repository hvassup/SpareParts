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

state_size = len(State)
action_size = len(Action)

# Initialize Q table
Q = np.random.randn(state_size, action_size)

def reward(state, new_state, action) -> int:
    if state != State.NoObject and new_state == State.NoObject:
        return 10
    elif state == State.NoObject and new_state == State.NoObject:
        if action == Action.Forward:
            return 11
    else:
        return -10
    
    return 0
    # reward = 0

    
    # if state != State.NoObject and new_state == State.NoObject:
    #     reward = 10
    #     if action == Action.Right or action == Action.Left:
    #         reward = 11
    # elif state == State.NoObject and new_state == State.NoObject:
    #     if action == Action.Forward:
    #         reward = 1

    # else:
    #     reward = -10
    
    # if action == Action.Back:
    #     reward -= 1
    
    # return reward
        # if action == Action.F:
        #     return 100
        # if action == Action.R:
        #     return 50
        # if action == Action.L:
        #     return 50
        # if action == Action.B:
        #     return 10
    # return -10

def print_q_table():
    print('############')
    row_format ="{:>20}" * (len(Action) + 1)
    print(row_format.format("", *[str(e) for e in Action]))
    for y in range(0, Q.shape[0]):
        values = np.round(Q[y][:], 2)
        print(row_format.format(str(State(y)), *values))
    print('############')

def update_q_table(state, action, new_state):
    global temperature, epsilon
    temperature = min(temperature + 1, 1000)
    epsilon = 1 - (temperature * 0.0009)

    Q[state, action] = Q[state, action] + alpha * (reward(state, new_state, action) + gamma * np.max(Q[new_state, :]) - Q[state, action])
    print_q_table()


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
