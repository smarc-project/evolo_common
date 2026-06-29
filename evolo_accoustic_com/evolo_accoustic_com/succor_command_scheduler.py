import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SuccorCommandScheduler(Node):
    """
    Receives JSON scheduling commands (from an MQTT bridge) and forwards
    raw serial command strings to the serial_parser node on a timer.
    Also subscribes to serial responses and re-publishes them as JSON for the MQTT bridge.

    Incoming JSON formats (on command_topic):
        {"command": "$B02LL", "repeating": true,  "period": 10.0}
        {"command": "$B02LL", "repeating": false}
        {"command": "$B02LL", "cancel": true}

    Outgoing commands (on serial_out_topic):
        std_msgs/String with msg.data = raw serial command, e.g. "$B02LL"

    Incoming responses (on serial_in_topic):
        std_msgs/String with msg.data = raw device response, e.g. "#B00107-0000.2"

    Outgoing feedback (on feedback_topic):
        {"response": "#B00107-0000.2"}
    """

    def __init__(self):
        super().__init__("succor_command_scheduler")

        self.declare_parameter("command_topic",  "/evolo/waraps/sensor/succor/command")
        self.declare_parameter("serial_out_topic", "/evolo/sensors/succor/to")
        self.declare_parameter("serial_in_topic",  "/evolo/sensors/succor/from")
        self.declare_parameter("feedback_topic", "/evolo/waraps/sensor/succor/feedback")
        self.declare_parameter("tick_rate", 10.0)  # Hz — scheduler resolution

        command_topic    = self.get_parameter("command_topic").get_parameter_value().string_value
        serial_out_topic = self.get_parameter("serial_out_topic").get_parameter_value().string_value
        serial_in_topic  = self.get_parameter("serial_in_topic").get_parameter_value().string_value
        feedback_topic   = self.get_parameter("feedback_topic").get_parameter_value().string_value
        tick_rate        = self.get_parameter("tick_rate").get_parameter_value().double_value

        self._serial_pub  = self.create_publisher(String, serial_out_topic, 10)
        self._feedback_pub = self.create_publisher(String, feedback_topic, 10)
        self.create_subscription(String, command_topic,   self._command_cb,  10)
        self.create_subscription(String, serial_in_topic, self._response_cb, 10)
        self.create_timer(1.0 / tick_rate, self._tick)

        # {command_str: {"period": float, "next_at": float}}
        self._scheduled: dict = {}

        self.get_logger().info(
            f"Commands:  '{command_topic}' → '{serial_out_topic}'\n"
            f"Responses: '{serial_in_topic}' → '{feedback_topic}'"
        )

    # ------------------------------------------------------------------

    def _command_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON: {e} — received: {msg.data!r}")
            return

        if data.get("cancel"):
            count = len(self._scheduled)
            self._scheduled.clear()
            self.get_logger().info(f"Cancelled all scheduled commands ({count})")
            return

        command = data.get("command", "")
        if not command:
            self.get_logger().warn("Message missing 'command' field")
            return

        if data.get("repeating", False):
            period = float(data.get("period", 10.0))
            self._scheduled[command] = {"period": period, "next_at": time.monotonic() + period}
            self.get_logger().info(f"Scheduled {command!r} every {period}s")

        else:
            self._send(command)

    def _response_cb(self, msg: String):
        # Ignore echoed TX commands ($...), only forward device responses (#...)
        if msg.data.startswith("$"):
            return
        out = String()
        out.data = json.dumps({"response": msg.data})
        self._feedback_pub.publish(out)
        self.get_logger().info(f"Feedback: {msg.data!r}")

    def _tick(self):
        now = time.monotonic()
        for command, entry in list(self._scheduled.items()):
            if now >= entry["next_at"]:
                self._send(command)
                entry["next_at"] = now + entry["period"]

    def _send(self, command: str):
        msg = String()
        msg.data = command
        self._serial_pub.publish(msg)
        self.get_logger().info(f"Sent: {command!r}")


def main(args=None):
    rclpy.init(args=args)
    node = SuccorCommandScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
