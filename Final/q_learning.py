import random
import numpy as np
from enums import State, Action

# Learning rate
alpha = 0.01

# epsilon = exploration
epsilon = 0.01

temperature = 0

# Discount factor
gamma = 0.9

state_size = len(State)
action_size = len(Action)

# Initialize Q table
# Q = np.random.randn(state_size, action_size)
Q = np.asarray([
    [-100,-100, 100,80],
    [-100,100, -100,80],
    [-100,-50,-50, 100],
    [100.0, 50, 50, 0]
])
def reward(state, new_state, action) -> int:
    if state != State.NoSensors and new_state == State.NoSensors:
        return 10
    elif state == State.NoSensors and new_state == State.NoSensors:
        if action == Action.Forward:
            return 11
    elif state == State.BothSensors and action == Action.Back:
        return 5
    else:
        if new_state == State.BothSensors:
            return -11
        return -10
    
    return 0

def print_q_table():
    print('############')
    row_format ="{:>20}" * (len(Action) + 1)
    print(row_format.format("", *[str(e) for e in Action]))
    for y in range(0, Q.shape[0]):
        values = np.round(Q[y][:], 2)
        print(row_format.format(str(State(y)), *values))
    print('############')

def update_q_table(state, action, new_state):
    print(state, action, new_state)
    global temperature, epsilon
    
    # temperature += 1
    # if temperature == 1000:
    #     epsilon = 0.01

    Q[state, action] = Q[state, action] + alpha * (reward(state, new_state, action) + gamma * np.max(Q[new_state, :]) - Q[state, action])


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

"""
Working table
                          Action.Forward         Action.Left        Action.Right         Action.Back
    State.LeftSensor              -100              -100               100              80
   State.RightSensor               -100              100               -100              80
   State.BothSensors               -100              -50              -50               100
     State.NoSensors               100.0               50               50               0
"""
