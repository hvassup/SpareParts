from math import cos, sin
from shared.state import R, L, simulation_timestep

def single_sim_step(x, y, q, _left_wheel_velocity, _right_wheel_velocity):
    v_x = cos(q) * (R * _left_wheel_velocity / 2 + R * _right_wheel_velocity / 2)
    v_y = sin(q) * (R * _left_wheel_velocity / 2 + R * _right_wheel_velocity / 2)
    omega = (R * _right_wheel_velocity - R * _left_wheel_velocity) / (2 * L)

    return (x + v_x * simulation_timestep, y + v_y * simulation_timestep, q + omega * simulation_timestep)