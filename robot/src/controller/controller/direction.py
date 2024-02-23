from enum import Enum


class Direction(Enum):
    """Direction enum."""

    FORWARD = (1, 1, 1, 1)
    BACKWARD = (-1, -1, -1, -1)
    STOP = (0, 0, 0, 0)
    TURN_LEFT = (0, 0, 1, 1)
    TURN_RIGHT = (1, 1, 0, 0)
    ROTATE_LEFT = (-1, -1, 1, 1)
    ROTATE_RIGHT = (1, 1, -1, -1)
    # TODO: LEFT, RIGHT (horizontal)
