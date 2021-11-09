from enum import IntEnum

class State(IntEnum):
    OL = 0
    OR = 1
    OF = 2
    NO = 3

class Action(IntEnum):
    F = 0
    L = 1
    R = 2
    B = 3
