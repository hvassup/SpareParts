from shared.util import is_point_inside_rectangle

def is_sensor_in_danger_spot(x, y, spots):
    for spot in spots:
        if is_point_inside_rectangle(x, y, *spot):
            return True
    return False