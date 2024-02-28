import json


def get_config() -> dict:
    """Returns the config."""
    config = {}
    path = "/robot/src/controller/controller/config.json"

    with open(path, "r") as f:
        config = json.load(f)

    return config


def get_threshold() -> int:
    """Returns the threshold."""
    return get_config()["threshold"]


def get_production() -> bool:
    """Returns if we are in production mode or not.

    Production mode means that the robot will interact with the expansion board (true)
    or not (false).
    """
    return get_config()["production"]
