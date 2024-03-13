import json

from .exceptions import NotInProductionMode


def in_production_mode(func):
    def function_wrapper(self, *args, **kwargs):
        if self.production:
            return func(self, *args, **kwargs)

        raise NotInProductionMode("Enable production mode to use this method.")

    return function_wrapper


def get_config() -> dict:
    """Returns the config from a local JSON file.

    JSON files do not get compiled, so values can be changed without recompiling the
    code. Hence the reason for having some options in a JSON file.
    """
    config = {}
    # path for the Docker container
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


def get_data_hertz() -> int:
    """Returns the hertz of which robot data should be sent to the VR headset."""
    return get_config()["data_hertz"]


def get_sleep_mode_hertz() -> int:
    """Returns the hertz of which how often the robot should check."""
    return get_config()["sleep_mode_hertz"]
