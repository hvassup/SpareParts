# Get wheel speed multipliers based on the 5 front sensors
def sensor_readings_to_motor_speeds(sensors):
    sensor1, sensor2, sensor3, sensor4, sensor5 = list(map(lambda x: x / 5025, sensors))

    left_mult = 1 - (sensor4 + sensor5) + sensor3 / 10
    right_mult = 1 - (sensor1 + sensor2) - sensor3 / 10
    return left_mult, right_mult
