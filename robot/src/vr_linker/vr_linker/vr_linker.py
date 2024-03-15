import json
import time

try:
    from interfaces.msg import VRData, VRHand, VRMode
except ModuleNotFoundError:
    # unittest cannot find this module
    pass


class VRLinker:
    def __init__(self, node) -> None:
        self.node = node

    def _to_json(self, data: dict | bytes) -> str | bytes:
        """Converts data to JSON.

        Args:
            data: data to convert

        Returns:
            JSON data as dict or bytes
        """
        try:
            if isinstance(data, bytes):
                return json.loads(data)

            return json.dumps(data).encode()
        except json.decoder.JSONDecodeError:
            print("Could not parse JSON data", data.decode())

    def _send(self, data: dict) -> None:
        """Sends data over the TCP socket.

        Args:
            data: data
        """
        if self.node.client_socket is None:
            return

        try:
            self.node.client_socket.sendall(self._to_json(data))
        except BrokenPipeError as e:
            print("Client disconnected", e)
            self.node.client_disconnected(e)

    @staticmethod
    def _get_vector_data(msg, key: str) -> list[float]:
        """Gets the x, y, z data from a vector message.

        Args:
            msg: message
            key: key to get data from

        Returns:
            x, y, z data
        """
        obj = getattr(msg, key)
        return [getattr(obj, i) for i in ["x", "y", "z"]]

    @staticmethod
    def _calculate_speed(speeds: list[float]) -> float:
        """Calculates the speed from the x and y speeds.

        Args:
            speeds: x and y speeds

        Returns:
            speed
        """
        v_x, v_y, _ = speeds
        return (v_x**2 + v_y**2) ** 0.5

    def handle_robot_data(self, msg) -> None:
        """Receives RobotData messages and sends over socket.

        Args:
            msg: RobotData message
        """
        print(f"-> {time.time()} {msg}")

        # motion = self._get_vector_data(msg, "motion")
        # speed = self._calculate_speed(motion)

        motion = self._get_vector_data(msg, "motion")
        speed = self._calculate_speed(motion)

        data = {
            "accelerometer": self._get_vector_data(msg, "accelerometer"),
            "gyroscope": self._get_vector_data(msg, "gyroscope"),
            "magnetometer": self._get_vector_data(msg, "magnetometer"),
            "motion": motion,
            "speed": speed,
            "battery": msg.battery,
            "voltage": msg.voltage,
            "mode": msg.mode,
        }

        print(f"{speed=} {msg.mode=} {msg.voltage=}")

        self._send(data)

    def process_message(self) -> None:
        """Processes a message sent by the VR headset over TCP socket."""
        data = self.node.client_socket.recv(1024)

        if not data:
            return

        data = self._to_json(data)
        print(f"-> {time.time()} {data}")

        self._publish_data(data)

        self._send({"success": True})

    def _is_vr_hand_message(self, data: dict) -> bool:
        """Checks if the message is a VRHand message.

        Args:
            data: data

        Returns:
            True if the message is a VRHand message
        """
        return "x" in data and "speed" not in data

    def _is_vr_data_message(self, data: dict) -> bool:
        """Checks if the message is a VRData message.

        Args:
            data: data

        Returns:
            True if the message is a VRData message
        """
        return "x" in data and "y" in data and "speed" in data

    def _is_mode_message(self, data: dict) -> bool:
        """Checks if the message is a mode message.

        Args:
            data: data

        Returns:
            True if the message is a mode message
        """
        return "mode" in data

    def _publish_vr_hand(self, data: dict) -> None:
        """Publishes VRHand messages.

        Args:
            data: data
        """
        vr_hand = VRHand()
        vr_hand.x = data["x"]
        vr_hand.y = data["y"]
        vr_hand.pinch = data["pinch"]
        vr_hand.strength = data["strength"]

        self.node.pub_vr_hand.publish(vr_hand)
        print(f"<- {time.time()} {vr_hand}")

    def _publish_vr_data(self, data: dict) -> None:
        """Publishes VRData messages.

        Args:
            data: data
        """
        vr_data = VRData()
        vr_data.x = data["x"]
        vr_data.y = data["y"]
        vr_data.speed = data["speed"]

        self.node.pub_vr.publish(vr_data)
        print(f"<- {time.time()} {vr_data}")

    def _publish_mode(self, data: dict) -> None:
        """Publishes mode messages.

        Args:
            data: data
        """
        mode = VRMode()
        mode.mode = data["mode"]
        self.node.pub_vr_mode.publish(mode)
        print(f"<- {time.time()} {mode}")

    def _publish_data(self, data: dict) -> None:
        """Publishes VRData messages.

        Args:
            data: data
        """
        if not data:
            return

        if self._is_mode_message(data):
            self._publish_mode(data)
            return

        if self._is_vr_data_message(data):
            self._publish_vr_data(data)
            return

        if self._is_vr_hand_message(data):
            self._publish_vr_hand(data)
            return
