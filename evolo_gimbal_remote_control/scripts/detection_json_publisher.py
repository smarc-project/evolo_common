#!/usr/bin/env python3
"""
Subscribes to yolo_msgs/DetectionArray and publishes one JSON-encoded
std_msgs/String per detected object.

Bearing and inclination are computed by projecting the detection bounding-box
centre through the camera FOV into a 3-D ray, then rotating that ray from the
camera optical frame into the global (ENU) frame via TF.

JSON output (one ROS message per detection):
  {"bearing": <deg>, "inclination": <deg>, "confidence": <float>,
   "class": <str>, "id": <str>}

bearing    : compass degrees, 0 = North, clockwise positive
inclination: degrees above (+) / below (-) horizon
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
from tf_transformations import quaternion_matrix
from std_msgs.msg import String

from yolo_msgs.msg import DetectionArray


_DEFAULT_DETECTIONS_TOPIC = "/yolo/detections"
_DEFAULT_JSON_TOPIC       = "/evolo/waraps/sensor/camera/feedback"
_DEFAULT_CAMERA_FRAME     = "evolo/z1_optical_frame"
_DEFAULT_GLOBAL_FRAME     = "evolo/odom"
_DEFAULT_IMAGE_WIDTH       = 1920
_DEFAULT_IMAGE_HEIGHT      = 1080
_DEFAULT_CAMERA_APERTURE   = 57.1  # camera_aperture parameter used by YoloActionServer


class DetectionJsonPublisher(Node):

    def __init__(self):
        super().__init__("detection_json_publisher")

        self.declare_parameter("detections_topic", _DEFAULT_DETECTIONS_TOPIC)
        self.declare_parameter("json_topic",       _DEFAULT_JSON_TOPIC)
        self.declare_parameter("camera_frame",     _DEFAULT_CAMERA_FRAME)
        self.declare_parameter("global_frame",     _DEFAULT_GLOBAL_FRAME)
        self.declare_parameter("image_width",       _DEFAULT_IMAGE_WIDTH)
        self.declare_parameter("image_height",      _DEFAULT_IMAGE_HEIGHT)
        self.declare_parameter("camera_aperture",   _DEFAULT_CAMERA_APERTURE)

        detections_topic   = self.get_parameter("detections_topic").get_parameter_value().string_value
        json_topic         = self.get_parameter("json_topic").get_parameter_value().string_value
        self._camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self._global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self._img_w        = self.get_parameter("image_width").get_parameter_value().integer_value
        self._img_h        = self.get_parameter("image_height").get_parameter_value().integer_value
        camera_aperture    = self.get_parameter("camera_aperture").get_parameter_value().double_value

        # Matches YoloActionServer exactly:
        #   angle_per_pixel = radians(aperture) / image_width
        # The same scale is applied to both x and y axes (width-normalised).
        self._angle_per_pixel = math.radians(camera_aperture) / self._img_w

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(
            DetectionArray,
            detections_topic,
            self._on_detections,
            10,
        )
        self._pub = self.create_publisher(String, json_topic, 10)

        self.get_logger().info(
            f"Subscribed to '{detections_topic}', publishing JSON on '{json_topic}'. "
            f"TF: '{self._camera_frame}' → '{self._global_frame}'. "
            f"camera_aperture={camera_aperture:.1f}°"
        )

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _pixel_to_ray_cam(self, cx: float, cy: float) -> np.ndarray:
        """
        Convert pixel (cx, cy) to a unit direction ray in the camera optical
        frame (x-right, y-down, z-forward).

        Uses the same linear approximation as YoloActionServer:
            angle_per_pixel = radians(camera_aperture) / image_width
            rx = (cx - cx0) * angle_per_pixel
            ry = (cy - cy0) * angle_per_pixel
        Both axes share the same scale (width-normalised), matching the
        existing gimbal tracker pipeline.
        """
        rx = (cx - self._img_w / 2.0) * self._angle_per_pixel
        ry = (cy - self._img_h / 2.0) * self._angle_per_pixel
        rz = 1.0
        ray = np.array([rx, ry, rz], dtype=float)
        return ray / np.linalg.norm(ray)

    def _lookup_rotation(self) -> np.ndarray | None:
        """
        Return the 3×3 rotation matrix R such that  v_global = R @ v_cam.
        Uses the latest available transform to avoid timestamp sync issues.
        Returns None if the transform is unavailable.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._camera_frame,
                Time(),                   # latest available
                Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup ({self._camera_frame} → {self._global_frame}) failed: {e}",
                throttle_duration_sec=2.0,
            )
            return None

        q = tf.transform.rotation
        return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    @staticmethod
    def _ray_to_bearing_inclination(ray: np.ndarray) -> tuple[float, float]:
        """
        Convert a direction vector in ENU (x=East, y=North, z=Up) to
        compass bearing [0–360°, 0=North, CW] and inclination [deg].
        """
        x, y, z = ray
        bearing_deg    = math.degrees(math.atan2(x, y)) % 360.0
        inclination_deg = math.degrees(math.atan2(z, math.sqrt(x * x + y * y)))
        return bearing_deg, inclination_deg

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------

    def _on_detections(self, msg: DetectionArray):
        if not msg.detections:
            return

        R = self._lookup_rotation()
        if R is None:
            return

        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y

            ray_cam    = self._pixel_to_ray_cam(cx, cy)
            ray_global = R @ ray_cam
            bearing, inclination = self._ray_to_bearing_inclination(ray_global)

            payload = {
                "bearing":     round(bearing, 2),
                "inclination": round(inclination, 2),
                "confidence":  round(float(det.score), 4),
                "class":       det.class_name,
                "id":          det.id,
            }
            out = String()
            out.data = json.dumps(payload)
            self._pub.publish(out)
            self.get_logger().debug(f"Published: {out.data}")


def main():
    rclpy.init()
    node = DetectionJsonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
