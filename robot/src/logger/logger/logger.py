import logging
import time

from .utils import FileFormatter

# TODO: add CSV also
try:
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

except FileNotFoundError:
    # happens when running sphinx due to different file path
    pass


class Logger:
    def __init__(self) -> None:
        """Initializes the logger."""
        pass

    def log(self, topic: str, msg) -> None:
        """Logs messages.

        Args:
            topic: topic
            msg: message
        """
        logging.info(f"{topic}: {msg}")

    def handle_vr_data(self, msg) -> None:
        """Handles VRData messages.

        Args:
            msg: VRData message
        """
        self.log("_vr_data", msg)

    def handle_vr_hand(self, msg) -> None:
        """Handles VRHand messages.

        Args:
            msg: VRHand message
        """
        self.log("_vr_hand", msg)

    def handle_vr_mode(self, msg) -> None:
        """Handles VRMode messages.

        Args:
            msg: VRMode message
        """
        self.log("_vr_mode", msg)

    def handle_robot_data(self, msg) -> None:
        """Handles RobotData messages.

        Args:
            msg: RobotData message
        """
        self.log("robot_data", msg)
