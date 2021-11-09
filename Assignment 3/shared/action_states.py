from shared.enums import State


def get_current_state(sensors):
    sensor1, sensor2, sensor3, sensor4, sensor5 = sensors
    if sensor3 > 0:
        return State.ObjectFront
    if sensor1 > 0 or sensor2 > 0:
        return State.ObjectLeft
    if sensor4 > 0 or sensor5 > 0:
        return State.ObjectRight
    return State.NoObject