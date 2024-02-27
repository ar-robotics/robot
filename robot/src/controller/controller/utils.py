import json


def get_config() -> dict:
    config = {}
    path = "/robot/src/controller/controller/config.json"

    with open(path, "r") as f:
        config = json.load(f)

    return config


def get_threshold() -> int:
    return get_config()["threshold"]


def get_production() -> bool:
    return get_config()["production"]
