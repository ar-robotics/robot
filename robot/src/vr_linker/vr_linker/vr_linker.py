import json
import time

try:
    from interfaces.msg import VRData, VRHand
except ModuleNotFoundError:
    # unittests have different path than the main program
    # set them to None to avoid import errors
    VRData, VRHand = None, None


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
        self.node.client_socket.sendall(self._to_json(data))

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
        return "pinch" in data and "wrist" in data

    def _is_vr_data_message(self, data: dict) -> bool:
        """Checks if the message is a VRData message.

        Args:
            data: data

        Returns:
            True if the message is a VRData message
        """
        return "x" in data and "y" in data and "speed" in data

    def _publish_vr_hand(self, data: dict) -> None:
        """Publishes VRHand messages.

        Args:
            data: data
        """
        vr_hand = VRHand()
        vr_hand.pinch = data["pinch"]
        vr_hand.wrist = data["wrist"]

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

    def _publish_data(self, data: dict) -> None:
        """Publishes VRData messages.

        Args:
            data: data
        """
        if not data:
            return

        if self._is_vr_hand_message(data):
            self._publish_vr_hand(data)
            return

        if self._is_vr_data_message(data):
            self._publish_vr_data(data)
            return
