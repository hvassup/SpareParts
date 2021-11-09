from enum import IntEnum

class State(IntEnum):
    ObjectLeft = 0
    ObjectRight = 1
    ObjectFront = 2
    NoObject = 3

class Action(IntEnum):
    Forward = 0
    Left = 1
    Right = 2
    # Back = 3
