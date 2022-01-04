from enum import IntEnum

class BottomColor(IntEnum):
    WHITE = 0
    BLACK = 1,
    GRAY = 2,

class Color(IntEnum):
    RED = 0
    YELLOW = 1,
    GREEN = 2,
    BLUE = 2,
    PURPLE = 2,

class State(IntEnum):
    LeftSensor = 0
    RightSensor = 1
    BothSensors = 2
    NoSensors = 3

class Action(IntEnum):
    Forward = 0
    Left = 1
    Right = 2
    Back = 3
