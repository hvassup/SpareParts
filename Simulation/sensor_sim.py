def distance_to_sensor_reading(distance):
    """
    Takes a distance in meters and returns a simulated sensor reading
    Parameters
        ----------
        distance : number
            The distance from the robot to the world in meters
    Returns
    -------
        float
            number between 0 and 5020
    """
    distance -= 0.05  # Simulate, that the sensors are not perfectly centered on the robot

    max_dist = 0.2
    max_reading = 5020

    if distance > max_dist:
        return 0
    return (1 - (distance / max_dist)) * max_reading

def read_april_tag(angle):
    """
    Takes an angle and simulates what april tag the robot would be looking at
    April tags go from 0 -> 15
    0 is on the middle on the right
    4 is on the middle of the bottom
    8 is on the middle of the left
    12 is on the middle of the top
    """
    
