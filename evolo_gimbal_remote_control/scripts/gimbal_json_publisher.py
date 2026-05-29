#!/usr/bin/env python3
"""
Subscribes to GimbalFeedback and publishes a JSON-encoded
std_msgs/String on the ROS topic /evolo/waraps/sensor/camera/feedback.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from z1_pro_msgs.msg import GimbalFeedback


# Map internal gimbal_mode strings → WARAPS mode strings
_MODE_MAP = {
    GimbalFeedback.GIMBAL_MODE_OFF:      "STOP",
    GimbalFeedback.GIMBAL_MODE_RPY:      "EULER",
    GimbalFeedback.GIMBAL_MODE_GEOPOINT: "GEO_POI",
    GimbalFeedback.GIMBAL_MODE_IMG_POI:  "TRACK",
    GimbalFeedback.GIMBAL_MODE_ODOM_POI: "TRACK",
}

_DEFAULT_JSON_TOPIC = "/evolo/waraps/sensor/camera/feedback"
_DEFAULT_FB_TOPIC   = "/evolo/gimbal_camera/gimbal_fb"


class GimbalJsonPublisher(Node):

    def __init__(self):
        super().__init__("gimbal_json_publisher")

        self.declare_parameter("feedback_topic", _DEFAULT_FB_TOPIC)
        self.declare_parameter("json_topic",     _DEFAULT_JSON_TOPIC)

        fb_topic   = self.get_parameter("feedback_topic").get_parameter_value().string_value
        json_topic = self.get_parameter("json_topic").get_parameter_value().string_value

        self._sub = self.create_subscription(
            GimbalFeedback,
            fb_topic,
            self._on_feedback,
            10,
        )
        self._pub = self.create_publisher(String, json_topic, 10)
        self.msg = None
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            f"Subscribed to '{fb_topic}', "
            f"publishing JSON on '{json_topic}'"
        )

    def _on_feedback(self, msg: GimbalFeedback):
        self.msg = msg

    def timer_callback(self):
        if self.msg == None:
            return
        msg = self.msg
        mode = _MODE_MAP.get(msg.gimbal_mode, "STOP")

        payload = {
            "mode":      mode,
            "roll":      msg.gcudata.absolute_roll,
            "pitch":     msg.gcudata.absolute_pitch,
            "yaw":       msg.gcudata.absolute_yaw,
            "latitude":  msg.geopoint_poi.latitude,
            "longitude": msg.geopoint_poi.longitude,
            "altitude":  msg.geopoint_poi.altitude,
        }

        out = String()
        out.data = json.dumps(payload)
        self._pub.publish(out)
        self.get_logger().debug(f"Published: {out.data}")
        self.msg = None


def main():
    rclpy.init()
    node = GimbalJsonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
