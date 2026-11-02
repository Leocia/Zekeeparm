from __future__ import annotations

import sys
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from pynput import keyboard
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .tracking import (TargetLock, detect_red_blocks, estimate_camera_point,
                       stabilize_camera_error, track_base_target)


class ShadowVision(Node):
    def __init__(self):
        super().__init__("zekeep_shadow_vision")
        default_root = Path(__file__).resolve().parents[2] / "zekeep_grasp"
        self.declare_parameter("perception_root", str(Path(os.environ["ZKEEP_WS"]) / "src/zekeep_grasp") if os.environ.get("ZKEEP_WS") else str(default_root))
        self.declare_parameter("config", "config/default.yaml")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("desired_depth_m", 0.15)
        self.root = Path(self.get_parameter("perception_root").value).expanduser().resolve()
        sys.path.insert(0, str(self.root))
        from utils.camera_utils import (compose_cam_to_base_transform, load_config,
                                        load_hand_eye)
        from utils.calibration_identity import validate_calibration_identity
        from drivers.camera import CameraFrameError, make_camera

        config_path = Path(self.get_parameter("config").value).expanduser()
        if not config_path.is_absolute():
            config_path = self.root / config_path
        self.cfg = load_config(config_path)
        self.desired_depth = float(self.get_parameter("desired_depth_m").value)
        if not 0.10 <= self.desired_depth <= 0.5:
            raise ValueError("desired_depth_m must be in [0.10, 0.5]")
        self.compose_cam_to_base_transform = compose_cam_to_base_transform
        self.camera_frame_error = CameraFrameError
        self.camera = make_camera(self.cfg)
        self.camera.open()
        expected_serial = str(self.cfg.get("camera", {}).get("serial", ""))
        try:
            if expected_serial and self.camera.serial != expected_serial:
                raise RuntimeError(f"camera serial mismatch: expected {expected_serial}, got {self.camera.serial}")
            validate_calibration_identity(self.root, self.cfg, self.camera.serial)
            camera_cfg = self.cfg.get("camera", {})
            robot_cfg = self.cfg.get("robot", {})
            self.reference_frame = str(robot_cfg.get("end_effector_frame", "")).strip()
            self.hand_eye, mode = load_hand_eye(
                self.root, str(camera_cfg.get("type", "")),
                expected_reference_frame=self.reference_frame)
            if self.hand_eye is None or mode != "eye_in_hand":
                raise RuntimeError("valid eye-in-hand calibration is required")
        except Exception:
            self.camera.close()
            raise
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.lock = TargetLock()
        self.filtered_error_camera = None
        self.error_axes_moving = np.zeros(3, dtype=bool)
        self.target_anchor_base = None
        self.detections = []
        self.points = self.create_publisher(PointStamped, "~/target_point", 10)
        self.errors = self.create_publisher(PointStamped, "~/target_error", 10)
        self.status = self.create_publisher(String, "~/status", 10)
        self.g_held = False
        self.held_pub = self.create_publisher(Bool, "~/g_held", 10)
        self.create_timer(0.05, self._publish_held)
        self.follower_status = "WAITING_FOR_SELECTION"
        self.last_tf_warning = 0.0
        self.last_frame_warning = 0.0
        follower_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/zekeep_shadow_follower/status", self._follower_status,
            follower_status_qos,
        )
        self.follow_services = {
            key: self.create_client(Trigger, f"/zekeep_shadow_follower/{key}")
            for key in ("return",)
        }
        self.get_logger().info("Click red block; hold g to follow, release g to hold; h return after loss, c clear, q quit.")

    def run(self):
        cv2.namedWindow("zekeep_shadow")
        cv2.setMouseCallback("zekeep_shadow", self.on_click)
        listener = None
        try:
            listener = keyboard.Listener(on_press=self._key_down, on_release=self._key_up)
            listener.start()
            while rclpy.ok():
                try:
                    color, depth = self.camera.get_frame()
                except self.camera_frame_error as exc:
                    color, depth = None, None
                    if time.monotonic() - self.last_frame_warning >= 2.0:
                        self.get_logger().error(str(exc))
                        self.last_frame_warning = time.monotonic()
                if color is None or depth is None:
                    self._status("NO_FRAME")
                    display = color.copy() if color is not None else np.zeros((360, 640, 3), np.uint8)
                    cv2.putText(display, "NO RGB-D FRAME", (15, 35),
                                cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2)
                    reason = getattr(self.camera, "last_frame_failure_reason", "")
                    cv2.putText(display, reason[:75], (15, 65),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 200, 255), 1)
                    cv2.imshow("zekeep_shadow", display)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    continue
                frame_stamp = self.get_clock().now().to_msg()
                raw = detect_red_blocks(color)
                try:
                    stamp_ns = frame_stamp.sec * 1_000_000_000 + frame_stamp.nanosec
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            self.base_frame, self.reference_frame,
                            rclpy.time.Time(nanoseconds=stamp_ns),
                            timeout=Duration(seconds=0.05))
                    except TransformException:
                        transform = self.tf_buffer.lookup_transform(
                            self.base_frame, self.reference_frame, rclpy.time.Time())
                        tf_stamp = transform.header.stamp
                        tf_ns = tf_stamp.sec * 1_000_000_000 + tf_stamp.nanosec
                        if abs(tf_ns - stamp_ns) > 100_000_000:
                            raise RuntimeError("latest camera transform differs from frame time by over 0.1 s")
                    T_ref_base = self._transform_matrix(transform.transform)
                    T_camera_base = self.compose_cam_to_base_transform(T_ref_base, self.hand_eye, self.cfg)
                except Exception as exc:
                    T_camera_base = None
                    self._status("TF_SYNC_UNAVAILABLE")
                    if time.monotonic() - self.last_tf_warning >= 2.0:
                        self.get_logger().warning(f"camera transform unavailable: {exc}")
                        self.last_tf_warning = time.monotonic()
                self.detections = []
                if self.follower_status.startswith(("TARGET_LOST", "RETURNING_TO_READY",
                                                    "WAITING_FOR_CONFIRMATION", "RETURN_FAILED")):
                    self.lock.selected = None
                    self.filtered_error_camera = None
                    self.error_axes_moving[:] = False
                    self.target_anchor_base = None
                for item in raw:
                    if T_camera_base is not None:
                        camera_point = estimate_camera_point(item, depth, self.camera.K)
                        if camera_point is not None:
                            item.position_base = T_camera_base[:3, :3] @ camera_point + T_camera_base[:3, 3]
                    self.detections.append(item)
                target = self.lock.update(self.detections) if self.lock.selected else None
                display = color.copy()
                for det in self.detections:
                    x1, y1, x2, y2 = det.bbox
                    color_box = (0, 255, 0) if det is target else (0, 165, 255)
                    cv2.rectangle(display, (x1, y1), (x2, y2), color_box, 2)
                    cv2.putText(display, f"{det.name} {det.confidence:.2f}", (x1, max(20, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, .55, color_box, 2)
                if self.lock.selected:
                    if target is not None and target.position_base is not None:
                        msg = PointStamped()
                        msg.header.stamp = frame_stamp
                        msg.header.frame_id = self.base_frame
                        msg.point.x, msg.point.y, msg.point.z = map(float, target.position_base)
                        self.points.publish(msg)
                        camera_point = estimate_camera_point(target, depth, self.camera.K)
                        if camera_point is not None:
                            if not self.g_held:
                                self.target_anchor_base = None
                            self.target_anchor_base = track_base_target(
                                target.position_base, self.target_anchor_base)
                            desired_base = T_camera_base[:3, :3] @ np.array(
                                [0.0, 0.0, self.desired_depth]) + T_camera_base[:3, 3]
                            anchor_error_camera = T_camera_base[:3, :3].T @ (
                                self.target_anchor_base - desired_base)
                            camera_error, self.filtered_error_camera, self.error_axes_moving = (
                                stabilize_camera_error(
                                    anchor_error_camera,
                                    self.filtered_error_camera, self.error_axes_moving))
                            error = T_camera_base[:3, :3] @ camera_error
                            error_msg = PointStamped()
                            error_msg.header = msg.header
                            error_msg.point.x, error_msg.point.y, error_msg.point.z = map(float, error)
                            self.errors.publish(error_msg)
                        self._status("LOCKED")
                        label = f"LOCKED base XYZ {target.position_base[0]:.3f} {target.position_base[1]:.3f} {target.position_base[2]:.3f} m"
                    else:
                        self._status("TF_SYNC_UNAVAILABLE" if T_camera_base is None else "LOST_OR_INVALID_DEPTH")
                        label = ("TF UNAVAILABLE" if T_camera_base is None
                                 else "AMBIGUOUS — click to reselect" if self.lock.failed
                                 else "LOCK LOST / INVALID DEPTH — click to reselect")
                    cv2.putText(display, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65,
                                (0, 255, 255), 2)
                else:
                    self._status("WAITING_FOR_SELECTION")
                cv2.putText(display, self.follower_status[:90], (12, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 0), 2)
                cv2.imshow("zekeep_shadow", display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("c"):
                    self.lock.selected = None
                    self.filtered_error_camera = None
                    self.error_axes_moving[:] = False
                    self.target_anchor_base = None
                if key == ord("h"):
                    self._call_follow("return")
        finally:
            self.g_held = False
            try:
                if rclpy.ok():
                    self._publish_held()
            finally:
                if listener is not None:
                    listener.stop()
                try:
                    self.camera.close()
                finally:
                    cv2.destroyAllWindows()
                    if rclpy.ok():
                        self.tf_listener.executor.shutdown()
                        self.tf_listener.dedicated_listener_thread.join(timeout=1.0)
                        self.tf_listener.unregister()

    def on_click(self, event, x, y, flags, userdata):
        del flags, userdata
        if event == cv2.EVENT_LBUTTONDOWN:
            self.target_anchor_base = None
            if self.lock.select(self.detections, x, y):
                self.filtered_error_camera = None
                self.error_axes_moving[:] = False
                self.get_logger().info("Red block selected")
            else:
                self.get_logger().warning("No red block at clicked location")

    def _status(self, value):
        msg = String(); msg.data = value
        self.status.publish(msg)

    def _follower_status(self, msg):
        self.follower_status = msg.data

    def _key_down(self, key):
        if (getattr(key, "char", None) or "").lower() == "g":
            self.g_held = True

    def _key_up(self, key):
        if (getattr(key, "char", None) or "").lower() == "g":
            self.g_held = False

    def _publish_held(self):
        msg = Bool()
        msg.data = self.g_held
        self.held_pub.publish(msg)

    def _call_follow(self, key):
        client = self.follow_services[key]
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(f"{key} service unavailable")
            return False
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._report_follow_result(done))
        return True

    def _report_follow_result(self, future):
        if future.result() is None:
            self.get_logger().error("follower request failed")
            return False
        result = future.result()
        (self.get_logger().info if result.success else self.get_logger().warning)(result.message)
        return bool(result.success)

    @staticmethod
    def _transform_matrix(transform):
        t, q = transform.translation, transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        matrix = np.eye(4)
        matrix[:3, :3] = [
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ]
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix


def main(args=None):
    rclpy.init(args=args)
    node = ShadowVision()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
