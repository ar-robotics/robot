import logging
import time

from .utils import FileFormatter

# TODO: add CSV also
LOG_FILE = f"./logger/logs/{int(time.time())}.log"

file_handler = logging.FileHandler(LOG_FILE, encoding="utf-8")

logging.basicConfig(
    level=logging.DEBUG,
    handlers=[
        file_handler,
    ],
)


file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(FileFormatter())


class Logger:
    def __init__(self) -> None:
        pass

    def log(self, topic: str, msg) -> None:
        logging.info(f"{topic}: {msg}")

    def handle_vr_data(self, msg) -> None:
        self.log("_vr_data", msg)
