def sensor_readings_to_motor_speeds(sensor1, sensor2, sensor3, sensor4, sensor5):
    sensor1 /= 5020
    sensor2 /= 5020
    sensor3 /= 5020
    sensor4 /= 5020
    sensor5 /= 5020

    left_mult = 1 - (sensor4 + sensor5) + sensor3 / 10
    right_mult = 1 - (sensor1 + sensor2) - sensor3 / 10
    return left_mult, right_mult
