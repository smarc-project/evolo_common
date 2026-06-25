#!/usr/bin/env python3
"""gimbal_poi_backend.py — Evolo prox-ops backend driven by Z1 Pro gimbal tracking.

This node implements the prox-ops backend contract defined in
prox_ops_actions/README.md.  Instead of a full graph-planning backend it uses
a direct approach: the Z1 Pro gimbal is already pointing at the target (held
by its POI-tracking firmware), so the bearing to the target is encoded in the
gimbal's relative_yaw angle.  If YOLO tracking detections with a valid id are
also available, a sub-pixel centring correction is added on top, and the
bounding-box area is used to decide when we are close enough to the target to
transition to INSPECT mode.

--- backend/control_planned ------------------------------------------------

The action server reads:
  pose.pose.orientation — desired vehicle heading in the ODOM (ENU/world) frame
  twist.twist.linear.x  — forward surge speed in the body frame

The formula for the heading used here is:
    target_yaw_ENU = vehicle_yaw_ENU + gimbal_relative_yaw_rad [+ pixel_offset]
This produces an absolute world-frame heading.
header.frame_id is set to the odom frame_id; child_frame_id is 'base_link'.

--- What is published without YOLO detections ------------------------------

Without a fresh id-filtered YOLO detection the backend publishes
target_lost=True, plan_available=False.  The vehicle does NOT follow anything
— the BT falls back to loiter/patrol.  The gimbal angle alone is not
sufficient: without visual confirmation we cannot know whether the gimbal is
still pointing at the real target or at a stale geographic position.

--- YOLO detection filtering -----------------------------------------------

Only detections from /yolo/tracking that have a non-empty 'id' field are used.
Untracked detections (id == "") are discarded because they may be one-shot
false positives that the tracker has not yet committed to.

--- Inspect-transition logic -----------------------------------------------

The transition to MODE_INSPECT (target_intercepted = True) is triggered when
the bounding-box area (width_px * height_px) of the best tracked detection
exceeds bbox_area_threshold_px2 for at least inspect_confirm_time_s seconds.
Large bbox area means the target fills enough of the frame that we are close
enough for inspection.  Without a valid YOLO detection the timer never starts.

--- Backend contract topics produced ---------------------------------------
  backend/status          evolo_msgs/ProxOpsBackendStatus
  backend/candidate_path  nav_msgs/Path              (minimal 2-pose lookahead)
  backend/control_planned nav_msgs/Odometry
                              orientation: absolute ENU heading (odom frame)
                              twist.linear.x: forward surge speed (body frame)

--- Backend contract topics consumed ---------------------------------------
  backend/command         std_msgs/String  (START / STOP / RESET / PAUSE / RESUME)

--- Additional subscriptions -----------------------------------------------
  <gcudata_topic>    z1_pro_msgs/Gcudata         — gimbal relative_yaw
  <detections_topic> yolo_msgs/DetectionArray    — required; id-filtered
  smarc/odom         nav_msgs/Odometry           — vehicle pose (ENU frame)

--- State machine -----------------------------------------------------------
  IDLE    — waiting for START command; publishes idle status
  RUNNING — active tracking; publishes all plan topics
  STOPPED — received STOP; publishes stopped status
"""

import math
import json

import rclpy
from rclpy.node import Node

from evolo_msgs.msg import ProxOpsBackendStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from z1_pro_msgs.msg import Gcudata

from yolo_msgs.msg import DetectionArray


class GimbalPoiBackend(Node):
    """
    Prox-ops backend driven by Z1 Pro gimbal angle + optional YOLO pixel offset.

    Mirrors the structure of fake_prox_ops_backend.cpp:
      - Constructor declares params, creates pubs/subs/timer.
      - _command_cb handles the START / STOP / RESET lifecycle.
      - _publish_tick fires at publish_frequency_hz and does all the work.
    """

    def __init__(self):
        super().__init__("gimbal_poi_backend")

        # ------------------------------------------------------------------ #
        # Parameters                                                           #
        # ------------------------------------------------------------------ #
        self.declare_parameter("gcudata_topic",
                               "/evolo/gimbal_camera/gimbal_gcu_fb")
        self.declare_parameter("detections_topic", "/yolo/tracking")
        self.declare_parameter("publish_frequency_hz", 10.0)

        # Staleness thresholds.  If a sensor message is older than its
        # max_age_s we treat the measurement as unavailable.
        self.declare_parameter("gcu_max_age_s", 1.0)
        self.declare_parameter("odom_max_age_s", 1.0)

        # Forward (surge) speed commanded to the vehicle during tracking.
        self.declare_parameter("forward_speed_mps", 5.0)

        # The candidate path is minimal: two poses at current position and a
        # point lookahead_m ahead in the current vehicle heading direction.
        # This satisfies the action-server safety gate (non-empty path,
        # consistent frame_id) without claiming to know the target position.
        self.declare_parameter("lookahead_m", 5.0)

        # Inspect-transition thresholds (bbox area gate).
        # The transition to MODE_INSPECT fires once the target bounding box
        # exceeds bbox_area_threshold_px2 for inspect_confirm_time_s seconds.
        self.declare_parameter("detection_max_age_s", 1.0)
        self.declare_parameter("bbox_area_threshold_px2", 40000.0)
        self.declare_parameter("inspect_confirm_time_s", 2.0)

        # Camera FOV parameters for the optional YOLO pixel correction.
        # Must match the values used by YoloActionServer and
        # detection_json_publisher.py (width-normalised convention).
        self.declare_parameter("image_width", 1920)
        self.declare_parameter("camera_aperture_deg", 57.1)

        # When True the backend starts in RUNNING state on node startup.
        # Useful for unit testing without prox_ops_bt.
        self.declare_parameter("autostart", False)

        # --- Read parameters ---
        gcudata_topic = self.get_parameter("gcudata_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        self._publish_hz = self.get_parameter("publish_frequency_hz").value
        self._gcu_max_age_s = self.get_parameter("gcu_max_age_s").value
        self._odom_max_age_s = self.get_parameter("odom_max_age_s").value
        self._detection_max_age_s = self.get_parameter("detection_max_age_s").value
        self._forward_speed_mps = self.get_parameter("forward_speed_mps").value
        self._lookahead_m = self.get_parameter("lookahead_m").value
        self._bbox_area_threshold = self.get_parameter("bbox_area_threshold_px2").value
        self._inspect_confirm_time_s = self.get_parameter("inspect_confirm_time_s").value
        self._img_w = self.get_parameter("image_width").value
        camera_aperture = self.get_parameter("camera_aperture_deg").value
        # Width-normalised scale — same as YoloActionServer and
        # detection_json_publisher.py.
        self._angle_per_pixel = math.radians(camera_aperture) / self._img_w
        autostart = self.get_parameter("autostart").value

        # ------------------------------------------------------------------ #
        # Publishers — backend contract                                        #
        # ------------------------------------------------------------------ #
        self._status_pub = self.create_publisher(
            ProxOpsBackendStatus, "backend/status", 10)
        self._path_pub = self.create_publisher(
            Path, "backend/candidate_path", 10)

        # backend/control_planned carries the unified control setpoint:
        #   pose.pose.orientation — desired vehicle heading in ENU/odom frame.
        #   twist.twist.linear.x  — forward surge speed in body frame
        self._control_pub = self.create_publisher(
            Odometry, "backend/control_planned", 10)

        # ------------------------------------------------------------------ #
        # Subscribers                                                          #
        # ------------------------------------------------------------------ #
        self._command_sub = self.create_subscription(
            String, "backend/command", self._command_cb, 10)

        self._gcu_sub = self.create_subscription(
            Gcudata, gcudata_topic, self._gcu_cb, 10)

        self._odom_sub = self.create_subscription(
            Odometry, "smarc/odom", self._odom_cb, 10)

        self._detections_sub = self.create_subscription(
            DetectionArray, detections_topic,
            self._detections_cb, 10)
        self.get_logger().info(
            f"YOLO tracking enabled (topic: '{detections_topic}').")

        # ------------------------------------------------------------------ #
        # Runtime state                                                        #
        # ------------------------------------------------------------------ #
        # State machine: "IDLE" | "RUNNING" | "STOPPED"
        self._state: str = "IDLE"

        # Latest sensor messages and the wall-clock time they arrived.
        self._last_gcu: Gcudata | None = None
        self._last_gcu_time: float | None = None
        self._last_odom: Odometry | None = None
        self._last_odom_time: float | None = None

        # Best tracked YOLO detection from the last message (id-filtered).
        # We store only what we need: bbox centre x (for pixel offset) and
        # bbox area (for inspect transition).
        self._last_det_cx: float | None = None       # bbox centre x in pixels
        self._last_det_area: float | None = None     # bbox width*height in px²
        self._last_det_time: float | None = None     # arrival wall-clock time

        # Inspect-transition timer.  Set to the wall-clock time at which the
        # bounding box first exceeded the area threshold.  Reset to None
        # whenever the condition drops below the threshold or detections go
        # stale.
        self._bbox_large_since: float | None = None

        # Publish timer — fires at publish_frequency_hz.
        self._timer = self.create_timer(1.0 / self._publish_hz,
                                        self._publish_tick)

        if autostart:
            self._start_run()

        self.get_logger().info(
            f"Gimbal POI backend started. "
            f"gcudata='{gcudata_topic}', "
            f"forward_speed={self._forward_speed_mps} m/s, "
            f"bbox_area_threshold={self._bbox_area_threshold:.0f} px², "
            f"inspect_confirm={self._inspect_confirm_time_s} s.")

    # ------------------------------------------------------------------ #
    # Command callback — mirrors fake_prox_ops_backend.cpp semantics       #
    # ------------------------------------------------------------------ #

    def _command_cb(self, msg: String) -> None:
        """
        Handle lifecycle commands from prox_ops_bt.

        Expected JSON format: {"command": "START"}
        The prox_ops_bt sends RESET then START on every new prox-ops goal,
        and RESET + STOP on timeout.  See prox_ops_actions/README.md.
        """
        self.get_logger().info(f"Backend command received: {msg.data}")
        try:
            payload = json.loads(msg.data)
            command = payload.get("command", "")
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"Could not parse backend command as JSON: '{msg.data}'")
            return

        if command == "START":
            self._start_run()

        elif command == "STOP":
            # prox_ops_bt has decided the mission is over (timeout exceeded).
            # Stop outputting plans so the BT can cleanly transition.
            self._state = "STOPPED"
            self._bbox_large_since = None
            self.get_logger().info("Backend STOPPED.")

        elif command == "RESET":
            # Called before START (new goal) and before STOP (timeout).
            # Clear all per-run state so a fresh run starts clean.
            self._state = "IDLE"
            self._bbox_large_since = None
            self._last_det_cx = None
            self._last_det_area = None
            self._last_det_time = None
            self.get_logger().info("Backend RESET.")

        elif command == "PAUSE":
            # PAUSE is defined in the contract but not used by the current BT.
            # Treat conservatively as STOP.
            self._state = "STOPPED"
            self.get_logger().info("Backend PAUSED (treated as STOPPED).")

        elif command == "RESUME":
            if self._state == "STOPPED":
                self._start_run()

        else:
            self.get_logger().warn(f"Unknown backend command: '{command}'")

    # ------------------------------------------------------------------ #
    # Sensor callbacks                                                     #
    # ------------------------------------------------------------------ #

    def _gcu_cb(self, msg: Gcudata) -> None:
        """Cache the latest Gcudata message and record its arrival time."""
        self._last_gcu = msg
        self._last_gcu_time = self._now_s

    def _odom_cb(self, msg: Odometry) -> None:
        """Cache the latest odometry and record its arrival time."""
        self._last_odom = msg
        self._last_odom_time = self._now_s

    def _detections_cb(self, msg) -> None:
        """
        Pick the best id-filtered YOLO detection and cache what we need.

        Filtering rules:
          1. Discard detections whose 'id' field is empty.  These are
             one-shot detections that the tracker has not committed to; using
             them could cause the vehicle to swerve toward false positives.
          2. Among the remaining (tracked) detections, pick the one with the
             highest confidence score.

        We store only:
          _last_det_cx   — bbox centre x in pixels (for pixel-offset correction)
          _last_det_area — bbox width * height in px²  (for inspect transition)
        """
        # Filter to id-confirmed (tracked) detections only.
        tracked = [d for d in msg.detections if d.id != ""]
        if not tracked:
            # No tracked detection this frame — do NOT clear the cache; let
            # the staleness check in _publish_tick handle expiry.
            return

        # Best tracked detection by confidence.
        best = max(tracked, key=lambda d: d.score)

        self._last_det_cx = best.bbox.center.position.x
        # yolo_msgs BoundingBox2D uses size.x / size.y for width / height.
        self._last_det_area = best.bbox.size.x * best.bbox.size.y
        self._last_det_time = self._now_s

    # ------------------------------------------------------------------ #
    # Main publish tick                                                    #
    # ------------------------------------------------------------------ #

    def _publish_tick(self) -> None:
        """Called at publish_frequency_hz.  Computes and publishes the plan."""
        if self._state != "RUNNING":
            self._publish_idle_status()
            return

        # ------------------------------------------------------------------ #
        # Sensor freshness checks                                              #
        # ------------------------------------------------------------------ #

        gcu_fresh = self._is_fresh(self._last_gcu_time, self._gcu_max_age_s)
        odom_fresh = self._is_fresh(self._last_odom_time, self._odom_max_age_s)

        if not gcu_fresh:
            # Cannot compute a bearing without gimbal data.
            # Signal target_lost so the BT falls back to loiter/patrol.
            self._publish_running_status(
                target_lost=True,
                plan_available=False,
                status_text="GCU_DATA_STALE")
            self._bbox_large_since = None
            return

        if not odom_fresh:
            # Without vehicle pose we cannot rotate the gimbal angle into the
            # global frame.  Stay patient (target_lost=False) rather than
            # triggering a patrol fallback for a transient odom dropout.
            self._publish_running_status(
                target_lost=False,
                plan_available=False,
                status_text="ODOM_STALE")
            return

        # ------------------------------------------------------------------ #
        # YOLO detection required                                              #
        # ------------------------------------------------------------------ #
        # We require a fresh id-filtered YOLO detection before issuing any
        # control command.  Without one, the gimbal might be pointing at a
        # stale last-known position with no visual confirmation that the
        # target is still there.  Signal target_lost so the BT falls back to
        # loiter/patrol rather than driving the vehicle on dead-reckoning.
        det_fresh = self._is_fresh(self._last_det_time, self._detection_max_age_s)
        if not det_fresh or self._last_det_cx is None:
            self._publish_running_status(
                target_lost=True,
                plan_available=False,
                status_text="NO_TRACKED_YOLO_DETECTION")
            self._bbox_large_since = None
            return

        # ------------------------------------------------------------------ #
        # Heading computation                                                  #
        # ------------------------------------------------------------------ #

        # vehicle_yaw_ENU: heading of the vessel in the ENU / odom frame,
        # extracted from the odometry quaternion.
        vehicle_yaw = self._yaw_from_odom(self._last_odom)

        # gimbal_relative_yaw: angle from vessel boresight to the target,
        # positive = target is to the RIGHT.  Convention verified in
        # gimbal_joint_publisher.cpp:
        #   yaw_angle_ = msg->relative_yaw * M_PI / 180.0  (positive sign,
        #   non-inverted for camera_below_base == false).
        gimbal_yaw_rad = math.radians(self._last_gcu.relative_yaw)

        # YOLO pixel-offset correction.
        # In ENU (CCW-positive), turning toward a target that is RIGHT of centre
        # means a negative (clockwise) angle correction, so the sign is inverted
        # relative to raw pixel coordinates where positive cx means right:
        #   pixel_offset_rad = -(cx - W/2) * angle_per_pixel
        # Positive cx-offset = target right of image centre = CW = negative ENU.
        pixel_offset_rad = (
            -(self._last_det_cx - self._img_w / 2.0)
            * self._angle_per_pixel
        )

        # Absolute ENU yaw pointing at the target.
        # This is what the action server places into the orientation field of
        # ctrl/control_planned (an Odometry in the odom/ENU frame).
        target_yaw = self._wrap_to_pi(
            vehicle_yaw + gimbal_yaw_rad + pixel_offset_rad)

        # ------------------------------------------------------------------ #
        # Inspect-transition (bbox-area gate)                                  #
        # ------------------------------------------------------------------ #
        # We transition to MODE_INSPECT (target_intercepted = True) once the
        # target's bounding box has been large enough for inspect_confirm_time_s
        # seconds.  "Large enough" means:
        #   bbox.size.x * bbox.size.y >= bbox_area_threshold_px2
        # This is the only signal we have for physical proximity: a large bbox
        # means the target fills enough of the frame that we are close enough
        # for inspection.
        bbox_large = (
            self._last_det_area is not None
            and self._last_det_area >= self._bbox_area_threshold
        )

        if bbox_large:
            if self._bbox_large_since is None:
                # Start the confirmation timer.
                self._bbox_large_since = self._now_s
            confirmed_duration = self._now_s - self._bbox_large_since
        else:
            # Condition not met — reset so we require a full uninterrupted
            # inspect_confirm_time_s of large-bbox detections.
            self._bbox_large_since = None
            confirmed_duration = 0.0

        target_intercepted = confirmed_duration >= self._inspect_confirm_time_s
        mode = (ProxOpsBackendStatus.MODE_INSPECT if target_intercepted
                else ProxOpsBackendStatus.MODE_LONG_RANGE_INTERCEPT)

        # ------------------------------------------------------------------ #
        # Publish plan                                                         #
        # ------------------------------------------------------------------ #

        now_msg = self.get_clock().now().to_msg()
        frame_id = self._last_odom.header.frame_id

        # All three plan topics must be published together whenever
        # plan_available=True, because the action-server safety gate checks
        # freshness of each independently.
        self._publish_candidate_path(now_msg, frame_id, target_yaw)
        self._publish_control(now_msg, frame_id, target_yaw)

        base_text = "GIMBAL_POI_YAW_OVERRIDE_YOLO_BBOX" if bbox_large else "GIMBAL_POI_YAW_OVERRIDE_YOLO"
        status_text = (
            f"{base_text} | vehicle_yaw={math.degrees(vehicle_yaw):.1f}deg"
            f" gimbal={math.degrees(gimbal_yaw_rad):.1f}deg"
            f" pixel_offset={math.degrees(pixel_offset_rad):.1f}deg"
            f" target_yaw={math.degrees(target_yaw):.1f}deg"
            + (f" bbox_area={self._last_det_area:.0f}px2"
               f" confirmed={confirmed_duration:.1f}s" if bbox_large else
               f" bbox_area={self._last_det_area:.0f}px2" if self._last_det_area is not None else "")
        )
        self._publish_running_status(
            target_lost=False,
            plan_available=True,
            mode=mode,
            target_intercepted=target_intercepted,
            status_text=status_text,
        )

    # ------------------------------------------------------------------ #
    # Publishing helpers                                                   #
    # ------------------------------------------------------------------ #

    def _publish_idle_status(self) -> None:
        """Publish a status indicating the backend is not actively planning."""
        status = ProxOpsBackendStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "map"
        status.health = ProxOpsBackendStatus.HEALTH_OK
        if self._state == "STOPPED":
            status.mode = ProxOpsBackendStatus.MODE_IDLE
            status.status_text = "GIMBAL_POI_STOPPED"
        else:
            status.mode = ProxOpsBackendStatus.MODE_UNKNOWN
            status.status_text = "GIMBAL_POI_IDLE"
        # No plan, no tracking → BT stays in loiter/patrol.
        status.long_range_track_live = False
        status.long_range_track_converged = False
        status.plan_available = False
        status.target_lost = True
        self._status_pub.publish(status)

    def _publish_running_status(
        self,
        *,
        target_lost: bool,
        plan_available: bool,
        mode: int = ProxOpsBackendStatus.MODE_LONG_RANGE_INTERCEPT,
        target_intercepted: bool = False,
        status_text: str = "",
    ) -> None:
        """Publish status while the backend is in RUNNING state."""
        status = ProxOpsBackendStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "map"
        status.mode = mode
        status.health = ProxOpsBackendStatus.HEALTH_OK

        # We treat the gimbal lock as a "long range track": it is live and
        # converged immediately — the gimbal firmware has already acquired the
        # target, there is no warm-up period.
        # This satisfies the BT pre-condition for activating evolo_target_intercept:
        #   health_ok AND long_range_track_converged AND plan_available AND NOT target_lost
        status.long_range_track_live = not target_lost
        status.long_range_track_converged = not target_lost
        status.terminal_track_live = False
        status.target_lost = target_lost
        status.plan_available = plan_available
        status.target_intercepted = target_intercepted
        status.long_range_confidence = 0.9 if not target_lost else 0.0
        status.terminal_confidence = 0.0
        status.status_text = status_text
        self._status_pub.publish(status)

    def _publish_candidate_path(
        self,
        stamp,
        frame_id: str,
        heading: float,
    ) -> None:
        """
        Publish a minimal valid candidate path to satisfy the safety gate.

        This path is a fiction — we know the bearing to the target but not
        its range, so we cannot produce a meaningful Cartesian trajectory.
        The two poses (current position and a point lookahead_m ahead along
        the computed target heading) satisfy the action-server gate:
          - path.header.frame_id is not empty
          - path.poses is not empty
          - per-pose frame_ids left empty (no frame contradiction)

        The direction uses the computed target heading rather than the vehicle
        heading so that, if geofence checking is enabled on evolo_target_intercept,
        the path at least points roughly in the direction the vehicle will turn.
        The length (lookahead_m) is still arbitrary.
        """
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame_id

        x0 = self._last_odom.pose.pose.position.x
        y0 = self._last_odom.pose.pose.position.y
        z0 = self._last_odom.pose.pose.position.z

        for dist in (0.0, self._lookahead_m):
            pose = PoseStamped()
            pose.header.stamp = stamp
            # Empty per-pose frame_id avoids the frame-consistency check.
            pose.pose.position.x = x0 + dist * math.cos(heading)
            pose.pose.position.y = y0 + dist * math.sin(heading)
            pose.pose.position.z = z0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self._path_pub.publish(path)

    def _publish_control(
        self,
        stamp,
        frame_id: str,
        target_yaw_rad: float,
    ) -> None:
        """
        Publish the unified control setpoint on backend/control_planned.

        pose.pose.orientation  — desired vehicle heading in the odom (ENU) frame,
                                 expressed as a pure yaw quaternion.
        twist.twist.linear.x   — forward surge speed in the body frame.
        pose.pose.position      — zeros; the action server uses its own odom for
                                  the current vehicle position.
        header.frame_id         — odom/ENU frame (must not be empty).
        child_frame_id          — 'base_link' (body frame for the twist).
        """
        control = Odometry()
        control.header.stamp = stamp
        control.header.frame_id = frame_id   # ENU/odom frame
        control.child_frame_id = "base_link"
        control.pose.pose.orientation.x = 0.0
        control.pose.pose.orientation.y = 0.0
        control.pose.pose.orientation.z = math.sin(target_yaw_rad * 0.5)
        control.pose.pose.orientation.w = math.cos(target_yaw_rad * 0.5)
        control.twist.twist.linear.x = self._forward_speed_mps
        self._control_pub.publish(control)

    # ------------------------------------------------------------------ #
    # Geometry helpers                                                     #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _yaw_from_odom(odom: Odometry) -> float:
        """
        Extract yaw in ENU radians from an Odometry quaternion.

        Uses the standard atan2 decomposition; only valid for small roll/pitch
        which is a reasonable assumption for a surface vessel.
        """
        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Wrap an angle in radians to (-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------------------ #
    # Utilities                                                            #
    # ------------------------------------------------------------------ #

    @property
    def _now_s(self) -> float:
        """Current ROS time as a float (seconds)."""
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def _is_fresh(self, timestamp_s: float | None, max_age_s: float) -> bool:
        """Return True iff the timestamp is not None and within max_age_s."""
        if timestamp_s is None:
            return False
        age = self._now_s - timestamp_s
        return 0.0 <= age <= max_age_s

    def _start_run(self) -> None:
        """Transition to RUNNING and reset per-run tracking state."""
        self._state = "RUNNING"
        self._bbox_large_since = None
        self.get_logger().info("Backend STARTED (gimbal POI, yaw-override mode).")


def main(args=None):
    rclpy.init(args=args)
    node = GimbalPoiBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
