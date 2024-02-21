from enum import Enum


class Direction(Enum):
    FORWARD = (1, 1, 1, 1)
    BACKWARD = (-1, -1, -1, -1)
    STOP = (0, 0, 0, 0)
    LEFT = (-1, 1, -1, 1)
    RIGHT = (1, -1, 1, -1)
    FORWARD_LEFT = (1, 1, 0.5, 0.5)
    FORWARD_RIGHT = (0.5, 0.5, 1, 1)
