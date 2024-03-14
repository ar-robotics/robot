from enum import Enum, IntEnum


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


class Preset(IntEnum):
    """Preset enum."""

    SPEED = 50
    UNPINCH_ANGLE = 45
    PINCH_ANGLE = 180
    WRIST_ANGLE = 135
    ARM_TILT_ANGLE = 90
    ARM_ROTATION_ANGLE = 90
    ARM_SHOULDER_ANGLE = 90
    ARM_ELBOW_ANGLE = 90
