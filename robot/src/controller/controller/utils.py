import json
import time

from .exceptions import NotInProductionMode


def in_production_mode(func):
    """Decorator which checks if we are in production mode or not.

    If we are in production mode, the function will be executed. If not, the
    `NotInProductionMode` exception will be raised.

    Args:
        func: function to decorate

    Returns:
        function_wrapper: wrapper function
    """

    def function_wrapper(self, *args, **kwargs):
        if self.production:
            return func(self, *args, **kwargs)

        raise NotInProductionMode("Enable production mode to use this method.")

    return function_wrapper


def set_last_message(func):
    """Decorator which sets the last message time and is_sleeping to False.

    Args:
        func: function to decorate

    Returns:
        function_wrapper: wrapper function
    """

    def function_wrapper(self, *args, **kwargs):
        self.last_command_received = time.time()
        self.is_sleeping = False
        return func(self, *args, **kwargs)

    return function_wrapper


def fill_vector_msg(msg, key: str, data: list):
    """Fills a std_msgs.msg.Vector3 message for given key.

    Args:
        msg: message
        key: key to fill for
        data: x, y, z data

    Returns:
        message with filled data
    """
    obj = getattr(msg, key)

    for i, n in zip(["x", "y", "z"], [0, 1, 2]):
        setattr(obj, i, data[n])

    return msg


def get_config() -> dict:
    """Returns the config from a local JSON file.

    JSON files do not get compiled, so values can be changed without recompiling the
    code. Hence the reason for having some options in a JSON file.

    Returns:
        config
    """
    config = {}
    # path for the Docker container
    path = "/robot/src/controller/controller/config.json"

    with open(path, "r") as f:
        config = json.load(f)

    return config


def get_threshold() -> int:
    """Returns the threshold.

    Returns:
        threshold
    """
    return get_config()["threshold"]


def get_production() -> bool:
    """Returns if we are in production mode or not.

    Production mode means that the robot will interact with the expansion board (true)
    or not (false).

    Returns:
        production mode
    """
    return get_config()["production"]


def get_data_hertz() -> int:
    """Returns the hertz of which robot data should be sent to the VR headset.

    Returns:
        hertz
    """
    return get_config()["data_hertz"]


def get_sleep_mode_hertz() -> int:
    """Returns the hertz of which how often the robot should check.

    Returns:
        hertz
    """
    return get_config()["sleep_mode_hertz"]


def get_sleep_mode_after() -> int:
    """Returns the time after which the robot should go to sleep mode.

    Returns:
        seconds
    """
    return get_config()["sleep_mode_after"]
