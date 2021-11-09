import math
from threading import current_thread
from shared.action_states import get_current_state
from shared.enums import Action
import shared.q_learning
from shared.sensor_sim import distance_to_sensor_reading, get_sensor_distance
from shared.step import single_sim_step
from shared.util import clamp, rotate_point
from simulation.visualization.pygame_visualizer import PyGameVisualizer
from shared.state import world, R, L, W, H, robot_timestep, simulation_timestep
visualizer = PyGameVisualizer()

# Variables

x = 0.0  # robot position in meters - x direction - positive to the right
y = 0.0  # robot position in meters - y direction - positive up
q = 0.0  # robot heading with respect to x-axis in radians

left_wheel_velocity = 0.5  # robot left wheel velocity in radians/s
right_wheel_velocity = 0.5  # robot right wheel velocity in radians/s


def get_front_sensor(angle):
    return distance_to_sensor_reading(get_sensor_distance(x, y, q, angle))

# Kinematic model

# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot
def simulationstep(_left_wheel_velocity, _right_wheel_velocity):
    global x, y, q, particles

    for step in range(int(robot_timestep / simulation_timestep)):  # step model time/timestep times
        x, y, q = single_sim_step(x, y, q, _left_wheel_velocity, _right_wheel_velocity)
        x = clamp(x, -W/2 + L/2, W/2 - L/2)
        y = clamp(y, -H/2 + L/2, H/2 - L/2)

def get_front_sensors():
    return [get_front_sensor(-30), get_front_sensor(-15), get_front_sensor(0), get_front_sensor(15), get_front_sensor(30)]

def get_state():
    front_sensors = get_front_sensors()
    return get_current_state(front_sensors)


def perform_action(action):
    global left_wheel_velocity, right_wheel_velocity
    if action == Action.Forward:
        left_wheel_velocity, right_wheel_velocity = (0.1, 0.1)
    elif action == Action.Left:
        left_wheel_velocity, right_wheel_velocity = (-0.1, 0.1)
    elif action == Action.Right:
        left_wheel_velocity, right_wheel_velocity = (0.1, -0.1)
    # elif action == Action.Back:
    #     left_wheel_velocity, right_wheel_velocity = (-0.1, -0.1)

def draw():
    sensors = get_front_sensors()
    
    visualizer.clear()
    ## Draw world
    p = [(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2)]
    for i in range(0, len(p)):
        visualizer.draw_line(*p[(i - 1) % len(p)], *p[i], (0, 0, 0))

    visualizer.draw_point(x, y, (255, 0, 255), L / 2)

    def draw_sensor_line(angle, col):
        r1x, r1y = rotate_point(0.05, 0, q + math.radians(angle))
        r2x, r2y = rotate_point(0.05 + 0.1, 0, q + math.radians(angle))
        visualizer.draw_line(x + r1x, y + r1y, x + r2x, y + r2y, col)
    
    for i, ang in enumerate([-30, -15, 0, 15, 30]):
        col = (0, 255, 0)
        if sensors[i] != 0:
            col = (255, 0, 0)
        draw_sensor_line(ang, col)
    
    visualizer.show()

def simulate():
    draw()
    ## Read from sensors
    current_state = get_state()
    action_to_perform = Action(shared.q_learning.get_next_action(current_state))
    perform_action(action_to_perform)
    for i in range(0, 100):
        simulationstep(left_wheel_velocity, right_wheel_velocity)
        draw()

    next_state = get_state()
    print('current_state:', current_state, 'action_to_perform:', action_to_perform, 'next_state:', next_state)
    shared.q_learning.update_q_table(current_state, action_to_perform, next_state)

while True:
    try:
        simulate()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        raise e