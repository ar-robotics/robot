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
        """Logs messages to a local file.

        Will log messages as DEBUG, except for "message" topic.
        For "message" topic, it will log with the level specified in the message.

        Args:
            topic: topic
            msg: message
        """
        if topic != "message":
            logging.debug(f"{topic=}: {msg}")
            return

        # logs with correct level
        # 10 is DEBUG, 20 is INFO, 30 is WARNING, 40 is ERROR, 50 is CRITICAL
        logging.log(msg.level, f"{msg.message}")

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

    def handle_message(self, msg) -> None:
        """Handles Message messages.

        Args:
            msg: Message message
        """
        self.log("message", msg)
