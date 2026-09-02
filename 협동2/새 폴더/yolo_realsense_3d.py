#!/usr/bin/env python3
"""YOLO best.pt + RealSense aligned depth + TF 3D object localization."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO


class YoloRealSense3D(Node):
    def __init__(self) -> None:
        super().__init__("yolo_realsense_3d")

        # YOLO
        self.declare_parameter("model_path", "best.pt")
        self.declare_parameter("confidence", 0.50)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("inference_rate_hz", 10.0)

        # Target selection
        self.declare_parameter("target_class", "")
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("target_policy", "highest_confidence")

        # ROS / TF
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info",
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.08)
        self.declare_parameter("tf_timeout_sec", 0.20)

        # Depth
        self.declare_parameter("depth_roi_ratio", 0.30)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 2.00)
        self.declare_parameter("min_valid_depth_pixels", 10)

        # Display
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "YOLO RealSense 3D")

        self.model_path = str(self.get_parameter("model_path").value).strip()
        self.confidence = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value).strip()
        rate = max(0.1, float(self.get_parameter("inference_rate_hz").value))
        self.min_period = 1.0 / rate

        self.target_class = str(self.get_parameter("target_class").value).strip()
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_policy = str(
            self.get_parameter("target_policy").value
        ).strip().lower()
        if self.target_policy not in {"highest_confidence", "nearest", "largest"}:
            raise ValueError(
                "target_policy must be highest_confidence, nearest, or largest"
            )

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.base_frame = self._clean_frame(
            str(self.get_parameter("base_frame").value)
        )
        self.camera_frame_override = self._clean_frame(
            str(self.get_parameter("camera_frame").value)
        )
        sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        sync_slop_sec = float(self.get_parameter("sync_slop_sec").value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout_sec").value)
        )

        self.depth_roi_ratio = float(
            self.get_parameter("depth_roi_ratio").value
        )
        self.depth_roi_ratio = float(np.clip(self.depth_roi_ratio, 0.05, 1.0))
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_valid_depth_pixels = int(
            self.get_parameter("min_valid_depth_pixels").value
        )
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = str(self.get_parameter("window_name").value)

        model_path = Path(self.model_path).expanduser()
        if not model_path.is_absolute():
            model_path = Path.cwd() / model_path
        if not model_path.is_file():
            raise FileNotFoundError(
                f"best.pt not found: {model_path}. Set model_path to an absolute path."
            )
        self.model_path = str(model_path.resolve())

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.class_names = self.model.names

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.last_inference_time = 0.0
        self.last_tf_warning_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_point_pub = self.create_publisher(
            PointStamped, "/yolo_object_3d/camera_point", 10
        )
        self.base_point_pub = self.create_publisher(
            PointStamped, "/yolo_object_3d/base_point", 10
        )
        self.detections_pub = self.create_publisher(
            String, "/yolo_object_3d/detections", 10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            self.color_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=sync_queue_size,
            slop=sync_slop_sec,
        )
        self.sync.registerCallback(self._synced_callback)

        self.get_logger().info("YOLO + RealSense 3D node started")
        self.get_logger().info(f"base_frame={self.base_frame}")

    @staticmethod
    def _clean_frame(frame_id: str) -> str:
        return frame_id.strip().lstrip("/")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _class_name(self, class_id: int) -> str:
        if isinstance(self.class_names, dict):
            return str(self.class_names.get(class_id, class_id))
        if isinstance(self.class_names, Sequence) and 0 <= class_id < len(self.class_names):
            return str(self.class_names[class_id])
        return str(class_id)

    def _matches_target(self, class_id: int, class_name: str) -> bool:
        if self.target_class_id >= 0:
            return class_id == self.target_class_id
        if self.target_class:
            return class_name.casefold() == self.target_class.casefold()
        return True

    def _depth_to_meters(self, msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding in {"16UC1", "mono16"} or depth.dtype == np.uint16:
            return depth.astype(np.float32) * 0.001
        return depth.astype(np.float32)

    def _intrinsics(self) -> Optional[Tuple[float, float, float, float]]:
        if self.camera_info is None:
            return None
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        return fx, fy, cx, cy

    def _camera_frame(self, color_msg: Image) -> str:
        if self.camera_frame_override:
            return self.camera_frame_override
        if self.camera_info is not None:
            frame = self._clean_frame(self.camera_info.header.frame_id)
            if frame:
                return frame
        return self._clean_frame(color_msg.header.frame_id)

    def _estimate_depth(
        self,
        depth_m: np.ndarray,
        bbox: Tuple[int, int, int, int],
        color_shape: Tuple[int, int],
    ) -> Optional[Tuple[float, int, int]]:
        x1, y1, x2, y2 = bbox
        color_h, color_w = color_shape
        depth_h, depth_w = depth_m.shape[:2]
        sx = depth_w / float(color_w)
        sy = depth_h / float(color_h)

        dx1 = int(np.clip(round(x1 * sx), 0, depth_w - 1))
        dx2 = int(np.clip(round(x2 * sx), 0, depth_w - 1))
        dy1 = int(np.clip(round(y1 * sy), 0, depth_h - 1))
        dy2 = int(np.clip(round(y2 * sy), 0, depth_h - 1))
        if dx2 <= dx1 or dy2 <= dy1:
            return None

        center_x = (dx1 + dx2) // 2
        center_y = (dy1 + dy2) // 2
        roi_w = max(3, int((dx2 - dx1) * self.depth_roi_ratio))
        roi_h = max(3, int((dy2 - dy1) * self.depth_roi_ratio))
        rx1 = max(0, center_x - roi_w // 2)
        rx2 = min(depth_w, center_x + roi_w // 2 + 1)
        ry1 = max(0, center_y - roi_h // 2)
        ry2 = min(depth_h, center_y + roi_h // 2 + 1)

        roi = depth_m[ry1:ry2, rx1:rx2]
        valid = roi[
            np.isfinite(roi)
            & (roi >= self.min_depth_m)
            & (roi <= self.max_depth_m)
        ]
        if valid.size < self.min_valid_depth_pixels:
            roi = depth_m[dy1 : dy2 + 1, dx1 : dx2 + 1]
            valid = roi[
                np.isfinite(roi)
                & (roi >= self.min_depth_m)
                & (roi <= self.max_depth_m)
            ]
        if valid.size < self.min_valid_depth_pixels:
            return None

        z = float(np.median(valid))
        u = int(np.clip(round(center_x / max(sx, 1e-9)), 0, color_w - 1))
        v = int(np.clip(round(center_y / max(sy, 1e-9)), 0, color_h - 1))
        return z, u, v

    @staticmethod
    def _deproject(
        u: int,
        v: int,
        z: float,
        intrinsics: Tuple[float, float, float, float],
    ) -> np.ndarray:
        fx, fy, cx, cy = intrinsics
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return np.array([x, y, z], dtype=np.float64)

    @staticmethod
    def _point_msg(
        xyz: Sequence[float], frame_id: str, stamp: Any
    ) -> PointStamped:
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        return msg

    def _transform_to_base(self, camera_point: PointStamped) -> Optional[PointStamped]:
        try:
            image_time = rclpy.time.Time.from_msg(camera_point.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_point.header.frame_id,
                image_time,
                timeout=self.tf_timeout,
            )
            return do_transform_point(camera_point, transform)
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    camera_point.header.frame_id,
                    rclpy.time.Time(),
                    timeout=self.tf_timeout,
                )
                return do_transform_point(camera_point, transform)
            except TransformException as exc:
                now = time.monotonic()
                if now - self.last_tf_warning_time > 2.0:
                    self.get_logger().warning(
                        f"TF unavailable: {self.base_frame} <- "
                        f"{camera_point.header.frame_id}: {exc}"
                    )
                    self.last_tf_warning_time = now
                return None

    def _select(self, detections: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        valid = [d for d in detections if d["camera_xyz_m"] is not None]
        if not valid:
            return None
        if self.target_policy == "nearest":
            return min(valid, key=lambda d: d["depth_m"])
        if self.target_policy == "largest":
            return max(valid, key=lambda d: d["bbox_area_px"])
        return max(valid, key=lambda d: d["confidence"])

    def _synced_callback(self, color_msg: Image, depth_msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_inference_time < self.min_period:
            return
        self.last_inference_time = now

        intrinsics = self._intrinsics()
        if intrinsics is None:
            self.get_logger().warning("Waiting for CameraInfo...")
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_m = self._depth_to_meters(depth_msg)
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return

        kwargs: Dict[str, Any] = {
            "source": bgr,
            "conf": self.confidence,
            "iou": self.iou,
            "imgsz": self.imgsz,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device

        try:
            result = self.model.predict(**kwargs)[0]
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        image_h, image_w = bgr.shape[:2]
        camera_frame = self._camera_frame(color_msg)
        detections: List[Dict[str, Any]] = []

        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.detach().cpu().numpy()
            confs = result.boxes.conf.detach().cpu().numpy()
            classes = result.boxes.cls.detach().cpu().numpy().astype(int)

            for box, conf, class_id in zip(xyxy, confs, classes):
                class_id = int(class_id)
                class_name = self._class_name(class_id)
                if not self._matches_target(class_id, class_name):
                    continue

                x1, y1, x2, y2 = [int(round(v)) for v in box.tolist()]
                x1 = int(np.clip(x1, 0, image_w - 1))
                x2 = int(np.clip(x2, 0, image_w - 1))
                y1 = int(np.clip(y1, 0, image_h - 1))
                y2 = int(np.clip(y2, 0, image_h - 1))
                if x2 <= x1 or y2 <= y1:
                    continue

                det: Dict[str, Any] = {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": float(conf),
                    "bbox_xyxy_px": [x1, y1, x2, y2],
                    "bbox_area_px": int((x2 - x1) * (y2 - y1)),
                    "center_uv_px": None,
                    "depth_m": None,
                    "camera_frame": camera_frame,
                    "camera_xyz_m": None,
                    "base_frame": self.base_frame,
                    "base_xyz_m": None,
                    "selected": False,
                }

                depth_result = self._estimate_depth(
                    depth_m, (x1, y1, x2, y2), (image_h, image_w)
                )
                if depth_result is not None:
                    z, u, v = depth_result
                    camera_xyz = self._deproject(u, v, z, intrinsics)
                    camera_point = self._point_msg(
                        camera_xyz, camera_frame, color_msg.header.stamp
                    )
                    base_point = self._transform_to_base(camera_point)
                    det["center_uv_px"] = [u, v]
                    det["depth_m"] = z
                    det["camera_xyz_m"] = [float(value) for value in camera_xyz]
                    if base_point is not None:
                        det["base_xyz_m"] = [
                            float(base_point.point.x),
                            float(base_point.point.y),
                            float(base_point.point.z),
                        ]

                detections.append(det)

        selected = self._select(detections)
        if selected is not None:
            selected["selected"] = True
            camera_point = self._point_msg(
                selected["camera_xyz_m"],
                selected["camera_frame"],
                color_msg.header.stamp,
            )
            self.camera_point_pub.publish(camera_point)
            if selected["base_xyz_m"] is not None:
                base_point = self._point_msg(
                    selected["base_xyz_m"],
                    self.base_frame,
                    color_msg.header.stamp,
                )
                self.base_point_pub.publish(base_point)

        payload = {
            "stamp": {
                "sec": int(color_msg.header.stamp.sec),
                "nanosec": int(color_msg.header.stamp.nanosec),
            },
            "target_policy": self.target_policy,
            "target_class": self.target_class,
            "target_class_id": self.target_class_id,
            "count": len(detections),
            "detections": detections,
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.detections_pub.publish(out)

        if self.show_window:
            self._draw(bgr, detections)
            cv2.imshow(self.window_name, bgr)
            key = cv2.waitKey(1) & 0xFF
            if key in {ord("q"), 27}:
                rclpy.shutdown()

    def _draw(self, image: np.ndarray, detections: List[Dict[str, Any]]) -> None:
        for det in detections:
            x1, y1, x2, y2 = det["bbox_xyxy_px"]
            color = (0, 255, 0) if det["selected"] else (255, 180, 0)
            thickness = 3 if det["selected"] else 2
            cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)

            label = f"{det['class_name']} {det['confidence']:.2f}"
            if det["depth_m"] is not None:
                label += f" Z={det['depth_m']:.3f}m"
            else:
                label += " depth=N/A"
            cv2.putText(
                image,
                label,
                (x1, max(24, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

            if det["center_uv_px"] is not None:
                u, v = det["center_uv_px"]
                cv2.circle(image, (u, v), 5, color, -1)

            if det["camera_xyz_m"] is not None:
                x, y, z = det["camera_xyz_m"]
                text = f"CAM X={x*1000:.1f} Y={y*1000:.1f} Z={z*1000:.1f} mm"
                cv2.putText(
                    image,
                    text,
                    (x1, min(image.shape[0] - 28, y2 + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    color,
                    1,
                    cv2.LINE_AA,
                )

            if det["base_xyz_m"] is not None:
                x, y, z = det["base_xyz_m"]
                text = f"BASE X={x*1000:.1f} Y={y*1000:.1f} Z={z*1000:.1f} mm"
                cv2.putText(
                    image,
                    text,
                    (x1, min(image.shape[0] - 8, y2 + 40)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        cv2.putText(
            image,
            f"detections={len(detections)} base={self.base_frame}",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def destroy_node(self) -> bool:
        if self.show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[YoloRealSense3D] = None
    try:
        node = YoloRealSense3D()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
