#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLO 객체 인식 + RealSense 3D 좌표 + DB Bridge + Service/Action 통합 노드.

테스트 실행 방식
-------------
- 실행 직후 별도의 Service/Action 신호 없이 자동 객체 인식을 시작한다.
- 기본 1초 간격으로 최신 RGB-D 프레임을 추론한다.
- 탐지 결과를 /assistive/object_detection 토픽으로 계속 발행한다.
- 기존 Service/Action 인터페이스도 그대로 사용할 수 있다.

좌표 단위
---------
- base 좌표 x/y/z: mm
- Service/Action coordinate: mm
- camera_depth_z: m

좌표 변환
---------
요청된 직접 행렬식을 사용한다.

    p_base =
        T_base_gripper(robot pose)
        @ T_gripper_camera(NPY)
        @ p_camera

NPY에는 역행렬을 적용하지 않는다.

중요:
- calibration_frame 기본값은 link_6이다.
- robot_pos는 TF에서 읽은 calibration_frame의 현재 자세를 mm/deg로 변환한다.

DB 저장 형식
------------
- 객체 이름을 record_name으로 사용한다.
- data.pose에는 다음 6개 값을 저장한다.

    [object_x_mm, object_y_mm, object_z_mm, robot_rx_deg, robot_ry_deg, robot_rz_deg]

- 객체 좌표는 base_link 기준이며, 회전값은 객체를 인식한 시점의 link_6 자세다.
- 기존 DB 필드는 replace=True로 제거하고 pose 필드만 남긴다.
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from hey_doopal_msg.action import FindOrder
from hey_doopal_msg.srv import GripBoundingBox, ScanRequest
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO


def quaternion_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError("Quaternion norm is zero")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def bbox_center_xyxy(
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
) -> Tuple[int, int]:
    return (
        int(round((float(x_min) + float(x_max)) / 2.0)),
        int(round((float(y_min) + float(y_max)) / 2.0)),
    )


class CameraToBaseTransformer:
    """NPY + TF를 이용해 RGB-D 픽셀을 base_link 좌표로 변환한다."""

    def __init__(
        self,
        *,
        node: Node,
        tf_buffer: tf2_ros.Buffer,
        transform_path: str,
        base_frame: str = "base_link",
        calibration_frame: str = "link_6",
        transform_direction: str = "gripper_to_camera",
        transform_translation_unit: str = "auto",
        tf_timeout_sec: float = 0.20,
        depth_scale_16u_m: float = 0.001,
        depth_roi_radius: int = 5,
        min_valid_depth_m: float = 0.15,
        max_valid_depth_m: float = 2.0,
        min_valid_depth_pixels: int = 8,
    ) -> None:
        self.node = node
        self.tf_buffer = tf_buffer
        self.base_frame = str(base_frame).strip().lstrip("/")
        self.calibration_frame = (
            str(calibration_frame).strip().lstrip("/")
        )
        self.tf_timeout = Duration(seconds=float(tf_timeout_sec))

        self.depth_scale_16u_m = float(depth_scale_16u_m)
        self.depth_roi_radius = max(1, int(depth_roi_radius))
        self.min_valid_depth_m = float(min_valid_depth_m)
        self.max_valid_depth_m = float(max_valid_depth_m)
        self.min_valid_depth_pixels = max(
            1,
            int(min_valid_depth_pixels),
        )

        if self.calibration_frame == "gripper_tcp":
            self.node.get_logger().warning(
                "calibration_frame=gripper_tcp이면 TCP 250 mm가 중복될 수 있습니다. "
                "현재 NPY에는 link_6를 사용해야 합니다."
            )

        self.gripper2cam_path = str(
            Path(transform_path).expanduser().resolve()
        )
        self.transform_translation_unit = str(
            transform_translation_unit
        ).strip().lower()
        self._validate_gripper2cam_file()

    def _validate_gripper2cam_file(self) -> None:
        path = Path(self.gripper2cam_path)
        if not path.is_file():
            raise FileNotFoundError(f"Transform file not found: {path}")
        matrix = np.asarray(np.load(str(path)), dtype=np.float64)
        if matrix.shape not in {(3, 4), (4, 4)}:
            raise ValueError(f"Transform must be 4x4 or 3x4: {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("Transform contains NaN or Inf")
        self.node.get_logger().info(
            "Coordinate transform: base2gripper @ gripper2cam @ camera_point "
            "(NPY inverse is not used)"
        )

    def _load_gripper2cam_mm(self, gripper2cam_path: str) -> np.ndarray:
        gripper2cam = np.asarray(
            np.load(str(Path(gripper2cam_path).expanduser().resolve())),
            dtype=np.float64,
        )
        if gripper2cam.shape == (3, 4):
            gripper2cam = np.vstack(
                (gripper2cam, np.array([[0.0, 0.0, 0.0, 1.0]]))
            )
        if gripper2cam.shape != (4, 4):
            raise ValueError(
                f"Transform must be 4x4 or 3x4: {gripper2cam.shape}"
            )
        if not np.all(np.isfinite(gripper2cam)):
            raise ValueError("Transform contains NaN or Inf")
        if abs(float(gripper2cam[3, 3])) < 1.0e-12:
            raise ValueError("Invalid homogeneous transform")

        gripper2cam = gripper2cam / float(gripper2cam[3, 3])
        unit = self.transform_translation_unit
        translation_norm = float(np.linalg.norm(gripper2cam[:3, 3]))
        if unit == "auto":
            unit = "m" if translation_norm < 2.0 else "mm"
        if unit == "m":
            gripper2cam[:3, 3] *= 1000.0
        elif unit != "mm":
            raise ValueError(
                "transform_translation_unit must be auto, m, or mm"
            )
        return gripper2cam

    @staticmethod
    def get_robot_pose_matrix(
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
    ) -> np.ndarray:
        """Create T_base_gripper from a Doosan-style Z-Y'-Z'' pose [mm, deg]."""
        a, b, c = np.deg2rad([rx, ry, rz])
        ca, sa = math.cos(a), math.sin(a)
        cb, sb = math.cos(b), math.sin(b)
        cc, sc = math.cos(c), math.sin(c)

        rot_z_a = np.array(
            [[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        rot_y_b = np.array(
            [[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]],
            dtype=np.float64,
        )
        rot_z_c = np.array(
            [[cc, -sc, 0.0], [sc, cc, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = rot_z_a @ rot_y_b @ rot_z_c
        matrix[:3, 3] = [float(x), float(y), float(z)]
        return matrix

    @staticmethod
    def _rotation_matrix_to_zyz_deg(rotation: np.ndarray) -> np.ndarray:
        r = np.asarray(rotation, dtype=np.float64)
        beta = math.acos(float(np.clip(r[2, 2], -1.0, 1.0)))
        sin_beta = math.sin(beta)
        if abs(sin_beta) > 1.0e-9:
            alpha = math.atan2(float(r[1, 2]), float(r[0, 2]))
            gamma = math.atan2(float(r[2, 1]), float(-r[2, 0]))
        else:
            alpha = math.atan2(float(r[1, 0]), float(r[0, 0]))
            gamma = 0.0
        return np.rad2deg([alpha, beta, gamma]).astype(np.float64)

    def transform_to_base(
        self,
        camera_coords: Sequence[float],
        gripper2cam_path: str,
        robot_pos: Sequence[float],
    ) -> np.ndarray:
        """Convert camera coordinates to the robot base frame using the requested formula."""
        gripper2cam = self._load_gripper2cam_mm(gripper2cam_path)
        coord = np.append(np.asarray(camera_coords, dtype=np.float64), 1.0)

        x, y, z, rx, ry, rz = [float(value) for value in robot_pos]
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)
        if abs(float(td_coord[3])) < 1.0e-12:
            raise ValueError("Invalid transformed homogeneous coordinate")
        return td_coord[:3] / float(td_coord[3])

    def _median_depth_m(
        self,
        *,
        depth_image: np.ndarray,
        depth_encoding: str,
        u: int,
        v: int,
        color_width: int,
        color_height: int,
    ) -> Optional[float]:
        if depth_image is None or depth_image.ndim < 2:
            return None

        depth_height, depth_width = depth_image.shape[:2]
        u_depth = int(round(u * depth_width / float(color_width)))
        v_depth = int(round(v * depth_height / float(color_height)))
        u_depth = int(np.clip(u_depth, 0, depth_width - 1))
        v_depth = int(np.clip(v_depth, 0, depth_height - 1))

        radius = self.depth_roi_radius
        x1 = max(0, u_depth - radius)
        x2 = min(depth_width, u_depth + radius + 1)
        y1 = max(0, v_depth - radius)
        y2 = min(depth_height, v_depth + radius + 1)

        patch = np.asarray(depth_image[y1:y2, x1:x2])
        if patch.size == 0:
            return None

        if (
            depth_encoding in {"16UC1", "mono16"}
            or patch.dtype == np.uint16
        ):
            patch_m = patch.astype(np.float32) * self.depth_scale_16u_m
        else:
            patch_m = patch.astype(np.float32)

        valid = patch_m[
            np.isfinite(patch_m)
            & (patch_m >= self.min_valid_depth_m)
            & (patch_m <= self.max_valid_depth_m)
        ]

        if valid.size < self.min_valid_depth_pixels:
            return None

        return float(np.median(valid))

    @staticmethod
    def _deproject_camera_m(
        *,
        u: int,
        v: int,
        depth_m: float,
        camera_info: CameraInfo,
    ) -> Optional[np.ndarray]:
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            return None

        return np.array(
            [
                (float(u) - cx) * depth_m / fx,
                (float(v) - cy) * depth_m / fy,
                depth_m,
            ],
            dtype=np.float64,
        )

    def _lookup_robot_pos_mm(
        self,
        stamp: Any,
    ) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.calibration_frame,
                Time.from_msg(stamp),
                timeout=self.tf_timeout,
            )
        except Exception:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.calibration_frame,
                    Time(),
                    timeout=self.tf_timeout,
                )
            except Exception as error:
                self.node.get_logger().warning(
                    f"TF unavailable: {self.base_frame} <- "
                    f"{self.calibration_frame}: {error}",
                    throttle_duration_sec=2.0,
                )
                return None

        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        rotation = quaternion_to_rotation_matrix(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        zyz_deg = self._rotation_matrix_to_zyz_deg(rotation)
        return np.array(
            [
                float(translation.x) * 1000.0,
                float(translation.y) * 1000.0,
                float(translation.z) * 1000.0,
                float(zyz_deg[0]),
                float(zyz_deg[1]),
                float(zyz_deg[2]),
            ],
            dtype=np.float64,
        )

    def pixel_to_base(
        self,
        *,
        u: int,
        v: int,
        depth_image: np.ndarray,
        depth_encoding: str,
        camera_info: CameraInfo,
        color_width: int,
        color_height: int,
        stamp: Any,
    ) -> Optional[Dict[str, np.ndarray | float]]:
        depth_m = self._median_depth_m(
            depth_image=depth_image,
            depth_encoding=depth_encoding,
            u=int(u),
            v=int(v),
            color_width=int(color_width),
            color_height=int(color_height),
        )
        if depth_m is None:
            return None

        camera_point_m = self._deproject_camera_m(
            u=int(u),
            v=int(v),
            depth_m=depth_m,
            camera_info=camera_info,
        )
        if camera_point_m is None:
            return None

        robot_pos = self._lookup_robot_pos_mm(stamp)
        if robot_pos is None:
            return None

        camera_point_mm = np.asarray(camera_point_m, dtype=np.float64) * 1000.0
        try:
            base_point_mm = self.transform_to_base(
                camera_point_mm,
                self.gripper2cam_path,
                robot_pos,
            )
            gripper2cam = self._load_gripper2cam_mm(self.gripper2cam_path)
        except (OSError, ValueError) as error:
            self.node.get_logger().error(
                f"Coordinate transform failed: {error}"
            )
            return None

        point_camera_h = np.append(camera_point_mm, 1.0)
        point_calibration_h = gripper2cam @ point_camera_h
        if abs(float(point_calibration_h[3])) < 1.0e-12:
            return None
        point_calibration_mm = (
            point_calibration_h[:3] / float(point_calibration_h[3])
        )

        if not np.all(np.isfinite(base_point_mm)):
            return None

        # 프로젝트 기준 보정:
        # link_6 기준으로 계산된 객체 Z에서 TCP 오프셋 250 mm를 제거하고
        # 지면 아래 음수 좌표는 0 mm로 제한한다.
        base_point_mm = np.asarray(base_point_mm, dtype=np.float64).copy()
        base_point_mm[2] = max(0.0, float(base_point_mm[2]) - 250.0)

        return {
            "depth_m": depth_m,
            "camera_point_m": camera_point_mm / 1000.0,
            "calibration_point_m": point_calibration_mm / 1000.0,
            "base_point_m": base_point_mm / 1000.0,
            # 객체 좌표 계산에 실제로 사용된 동일 시점의 link_6 pose [mm, deg]
            "robot_pose_mm_deg": np.asarray(robot_pos, dtype=np.float64).copy(),
        }


class ObjectDetectionNode(Node):
    def __init__(
        self,
        model: YOLO,
        color_topic: str = "/camera/camera/color/image_raw",
        depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw",
        camera_info_topic: str = "/camera/camera/color/camera_info",
        hand_eye_transform_path: str = str(
            Path(__file__).resolve().parent / "T_gripper2camera.npy"
        ),
        base_frame: str = "base_link",
        calibration_frame: str = "link_6",
        full_scan_conf: float = 0.6,
        find_target_conf: float = 0.5,
        find_target_timeout: float = 15.0,
        find_target_interval: float = 0.15,
        grip_retry_attempts: int = 3,
        grip_retry_interval: float = 0.15,
        per_class_conf_threshold: Optional[Dict[str, float]] = None,
        auto_test_mode: bool = True,
        auto_test_interval_sec: float = 1.0,
    ) -> None:
        super().__init__("object_detection_node")

        self.model = model
        self.class_names = model.names
        self.bridge = CvBridge()

        self.full_scan_conf = float(full_scan_conf)
        self.find_target_conf = float(find_target_conf)
        self.find_target_timeout = float(find_target_timeout)
        self.find_target_interval = float(find_target_interval)
        self.grip_retry_attempts = int(grip_retry_attempts)
        self.grip_retry_interval = float(grip_retry_interval)
        self.per_class_conf_threshold = per_class_conf_threshold or {}

        # 테스트 모드: 서비스/액션 신호 없이 주기적으로 객체를 탐지하고
        # /assistive/object_detection 토픽으로 DB Bridge 메시지를 발행한다.
        self.auto_test_mode = bool(auto_test_mode)
        self.auto_test_interval_sec = max(0.1, float(auto_test_interval_sec))
        self.auto_test_scan_lock = threading.Lock()
        self.last_auto_test_wait_log_time = 0.0

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.coordinate_transformer = CameraToBaseTransformer(
            node=self,
            tf_buffer=self.tf_buffer,
            transform_path=hand_eye_transform_path,
            base_frame=base_frame,
            calibration_frame=calibration_frame,
            transform_direction="gripper_to_camera",
            transform_translation_unit="auto",
            tf_timeout_sec=0.20,
            depth_scale_16u_m=0.001,
            depth_roi_radius=5,
            min_valid_depth_m=0.15,
            max_valid_depth_m=2.0,
            min_valid_depth_pixels=8,
        )

        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_color: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_encoding = ""
        self.latest_stamp = None

        self.frame_lock = threading.Lock()
        self.inference_lock = threading.Lock()

        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        color_sub = message_filters.Subscriber(
            self,
            Image,
            color_topic,
            qos_profile=qos_profile_sensor_data,
        )
        depth_sub = message_filters.Subscriber(
            self,
            Image,
            depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=10,
            slop=0.08,
        )
        self.sync.registerCallback(self.frame_callback)

        self.object_detection_pub = self.create_publisher(
            String,
            "/assistive/object_detection",
            10,
        )

        service_cb_group = ReentrantCallbackGroup()
        self.scan_srv = self.create_service(
            ScanRequest,
            "yolo_scan_request",
            self.handle_scan_request,
            callback_group=service_cb_group,
        )
        self.grip_srv = self.create_service(
            GripBoundingBox,
            "grip_bounding_box",
            self.handle_grip_bounding_box,
            callback_group=service_cb_group,
        )

        action_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FindOrder,
            "find_target_order",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_cb_group,
        )

        self.auto_test_timer = None
        if self.auto_test_mode:
            self.auto_test_timer = self.create_timer(
                self.auto_test_interval_sec,
                self._auto_test_scan_callback,
                callback_group=ReentrantCallbackGroup(),
            )

        self.get_logger().info(
            "ObjectDetectionNode ready: "
            "yolo_scan_request, grip_bounding_box, find_target_order"
        )
        self.get_logger().info(
            "Coordinate chain: base2gripper(robot pose) @ "
            "gripper2cam(NPY direct) @ camera_point"
        )
        self.get_logger().info(
            "DB object format: pose=[x,y,z,rx,ry,rz] "
            "(mm,mm,mm,deg,deg,deg), replace=True"
        )
        self.get_logger().warning(
            "Base Z correction enabled: z=max(0, z-250.0 mm)"
        )
        if self.auto_test_mode:
            self.get_logger().warning(
                "AUTO TEST MODE enabled: no service/action signal is required. "
                f"Detection and DB Bridge publishing run every "
                f"{self.auto_test_interval_sec:.2f} sec."
            )

    def _auto_test_scan_callback(self) -> None:
        """신호 없이 최신 RGB-D 프레임을 주기적으로 탐지해 DB Bridge로 보낸다."""
        if not self.auto_test_mode:
            return

        # MultiThreadedExecutor에서 이전 추론이 끝나기 전에 다음 타이머가
        # 중첩 실행되는 것을 막는다.
        if not self.auto_test_scan_lock.acquire(blocking=False):
            return

        try:
            detections = self.run_inference_on_latest_frame(
                self.full_scan_conf
            )

            if detections:
                self.publish_to_db_bridge(detections)
                summary = ", ".join(
                    f"{item['class_name']}="
                    f"({item['x']:.1f}, {item['y']:.1f}, {item['z']:.1f}) mm"
                    for item in detections
                )
                self.get_logger().info(
                    f"[자동 테스트] {len(detections)}개 탐지/발행: {summary}"
                )
            else:
                now = time.monotonic()
                if now - self.last_auto_test_wait_log_time >= 3.0:
                    self.get_logger().warning(
                        "[자동 테스트] 탐지 결과 없음. RGB/Depth/CameraInfo, "
                        "YOLO confidence, TF와 NPY 변환을 확인하세요."
                    )
                    self.last_auto_test_wait_log_time = now
        except Exception as error:
            self.get_logger().error(
                f"[자동 테스트] 추론 또는 DB 발행 중 오류: {error}"
            )
        finally:
            self.auto_test_scan_lock.release()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def frame_callback(self, color_msg: Image, depth_msg: Image) -> None:
        try:
            color_img = self.bridge.imgmsg_to_cv2(
                color_msg,
                desired_encoding="bgr8",
            )
            depth_img = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
        except CvBridgeError as error:
            self.get_logger().error(f"cv_bridge error: {error}")
            return

        with self.frame_lock:
            self.latest_color = color_img
            self.latest_depth = depth_img
            self.latest_depth_encoding = depth_msg.encoding
            self.latest_stamp = color_msg.header.stamp

    def _class_name(self, class_id: int) -> str:
        if isinstance(self.class_names, dict):
            return str(self.class_names.get(class_id, f"class_{class_id}"))
        try:
            return str(self.class_names[class_id])
        except (IndexError, TypeError):
            return f"class_{class_id}"

    def run_inference_on_latest_frame(
        self,
        conf_threshold: float,
        target_label: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        with self.frame_lock:
            if self.latest_color is None or self.latest_depth is None:
                return []

            color_img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()
            depth_encoding = self.latest_depth_encoding
            stamp = self.latest_stamp

        camera_info = self.latest_camera_info
        if camera_info is None or stamp is None:
            return []

        with self.inference_lock:
            results = self.model(color_img, verbose=False)

        detections: List[Dict[str, Any]] = []
        color_height, color_width = color_img.shape[:2]

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                confidence = float(box.conf[0].item())
                if confidence < float(conf_threshold):
                    continue

                class_id = int(box.cls[0].item())
                label = self._class_name(class_id)

                effective_threshold = float(
                    self.per_class_conf_threshold.get(
                        label,
                        conf_threshold,
                    )
                )
                if confidence < effective_threshold:
                    continue

                if target_label and label != target_label:
                    continue

                x1, y1, x2, y2 = [
                    int(round(value))
                    for value in box.xyxy[0].detach().cpu().tolist()
                ]
                center_u, center_v = bbox_center_xyxy(x1, y1, x2, y2)

                coordinate = self.coordinate_transformer.pixel_to_base(
                    u=center_u,
                    v=center_v,
                    depth_image=depth_img,
                    depth_encoding=depth_encoding,
                    camera_info=camera_info,
                    color_width=color_width,
                    color_height=color_height,
                    stamp=stamp,
                )
                if coordinate is None:
                    continue

                base_point_mm = (
                    np.asarray(coordinate["base_point_m"], dtype=np.float64)
                    * 1000.0
                )
                camera_point_mm = (
                    np.asarray(coordinate["camera_point_m"], dtype=np.float64)
                    * 1000.0
                )
                link6_point_mm = (
                    np.asarray(
                        coordinate["calibration_point_m"],
                        dtype=np.float64,
                    )
                    * 1000.0
                )
                robot_pose_mm_deg = np.asarray(
                    coordinate["robot_pose_mm_deg"],
                    dtype=np.float64,
                )

                detections.append(
                    {
                        "class_name": label,
                        "class_id": class_id,
                        "confidence": round(confidence, 4),
                        "coordinate_unit": "mm",
                        "frame_id": self.coordinate_transformer.base_frame,
                        "x": round(float(base_point_mm[0]), 2),
                        "y": round(float(base_point_mm[1]), 2),
                        "z": round(float(base_point_mm[2]), 2),
                        # 객체를 인식한 동일 이미지 시점의 M0609 link_6 회전 자세 [deg]
                        "rx": round(float(robot_pose_mm_deg[3]), 2),
                        "ry": round(float(robot_pose_mm_deg[4]), 2),
                        "rz": round(float(robot_pose_mm_deg[5]), 2),
                        "camera_x_mm": round(float(camera_point_mm[0]), 2),
                        "camera_y_mm": round(float(camera_point_mm[1]), 2),
                        "camera_z_mm": round(float(camera_point_mm[2]), 2),
                        "link6_x_mm": round(float(link6_point_mm[0]), 2),
                        "link6_y_mm": round(float(link6_point_mm[1]), 2),
                        "link6_z_mm": round(float(link6_point_mm[2]), 2),
                        "camera_depth_z": round(
                            float(coordinate["depth_m"]),
                            4,
                        ),
                        "bbox_width": int(x2 - x1),
                        "bbox_height": int(y2 - y1),
                        "bbox_center_u": int(center_u),
                        "bbox_center_v": int(center_v),
                    }
                )

        return detections

    def publish_to_db_bridge(
        self,
        detections: Sequence[Dict[str, Any]],
    ) -> None:
        for detection in detections:
            class_name = detection.get("class_name")
            if not class_name:
                self.get_logger().error(
                    "[DB Bridge] class_name missing"
                )
                continue

            # Redis 객체 레코드는 다음 한 필드만 유지한다.
            # pose = [object_x, object_y, object_z, robot_rx, robot_ry, robot_rz]
            pose = [
                round(float(detection["x"]), 2),
                round(float(detection["y"]), 2),
                round(float(detection["z"]), 2),
                round(float(detection["rx"]), 2),
                round(float(detection["ry"]), 2),
                round(float(detection["rz"]), 2),
            ]

            payload = {
                "record_name": class_name,
                "data": {"pose": pose},
                # 과거 x/y/z/saved_at 등의 필드가 남지 않도록 전체 교체한다.
                "replace": True,
            }

            message = String()
            message.data = json.dumps(
                payload,
                ensure_ascii=False,
            )
            self.object_detection_pub.publish(message)
            self.get_logger().info(
                f'[DB Bridge] published: {class_name} -> {pose}'
            )

    # ------------------------------------------------------------------
    # Scan service
    # ------------------------------------------------------------------
    def handle_scan_request(self, request, response):
        waypoint_id = getattr(request, "waypoint_id", "")
        self.get_logger().info(
            f'[전체 스캔] 요청 수신 (waypoint_id="{waypoint_id}")'
        )

        detections = self.run_inference_on_latest_frame(
            self.full_scan_conf
        )

        if detections:
            self.publish_to_db_bridge(detections)
            response.success = True
            response.message = (
                f"{len(detections)}개 객체 스캔 및 DB Bridge 전송 완료"
            )
            response.detected_count = len(detections)
        else:
            response.success = True
            response.message = (
                "탐지된 객체 없음 또는 depth/TF 좌표 변환 실패"
            )
            response.detected_count = 0

        return response

    # ------------------------------------------------------------------
    # Grip confirmation service
    # ------------------------------------------------------------------
    def handle_grip_bounding_box(self, request, response):
        target_label = str(request.target).strip()
        self.get_logger().info(
            f'[그립 확인] 요청 수신: target="{target_label}"'
        )

        best = None
        for attempt in range(1, self.grip_retry_attempts + 1):
            detections = self.run_inference_on_latest_frame(
                self.find_target_conf,
                target_label=target_label,
            )

            if detections:
                best = max(
                    detections,
                    key=lambda item: item["confidence"],
                )
                break

            self.get_logger().warning(
                f'[그립 확인] "{target_label}" '
                f"{attempt}/{self.grip_retry_attempts} 실패"
            )
            if attempt < self.grip_retry_attempts:
                time.sleep(self.grip_retry_interval)

        if best is not None:
            self.publish_to_db_bridge([best])
            response.coordinate = [
                float(best["x"]),
                float(best["y"]),
                float(best["z"]),
            ]
            response.bbox_width = float(best["bbox_width"])
            response.bbox_height = float(best["bbox_height"])
            response.camera_depth_z = float(best["camera_depth_z"])
            response.is_find = True
        else:
            response.coordinate = [0.0, 0.0, 0.0]
            response.bbox_width = 0.0
            response.bbox_height = 0.0
            response.camera_depth_z = 0.0
            response.is_find = False

        return response

    # ------------------------------------------------------------------
    # Find target action
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request):
        target_name = str(goal_request.target_name).strip()
        if not target_name:
            self.get_logger().warning(
                "[타겟 찾기] empty target_name rejected"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_label = str(goal_handle.request.target_name).strip()
        feedback_msg = FindOrder.Feedback()
        result = FindOrder.Result()

        start = time.monotonic()
        attempts = 0
        found_detection = None

        while (
            rclpy.ok()
            and time.monotonic() - start < self.find_target_timeout
        ):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.found = False
                result.coordinate = [0.0, 0.0, 0.0]
                result.message = "canceled"
                return result

            attempts += 1
            feedback_msg.state = "searching"
            goal_handle.publish_feedback(feedback_msg)

            detections = self.run_inference_on_latest_frame(
                self.find_target_conf,
                target_label=target_label,
            )

            if detections:
                found_detection = max(
                    detections,
                    key=lambda item: item["confidence"],
                )
                break

            time.sleep(self.find_target_interval)

        if found_detection is not None:
            feedback_msg.state = "calculating"
            goal_handle.publish_feedback(feedback_msg)

            self.publish_to_db_bridge([found_detection])
            goal_handle.succeed()

            result.found = True
            result.coordinate = [
                float(found_detection["x"]),
                float(found_detection["y"]),
                float(found_detection["z"]),
            ]
            result.message = str(found_detection["class_name"])

            self.get_logger().info(
                f'[타겟 찾기] "{target_label}" '
                f"{attempts}번 시도 만에 발견: "
                f"{result.coordinate} mm"
            )
        else:
            goal_handle.abort()
            result.found = False
            result.coordinate = [0.0, 0.0, 0.0]
            result.message = ""

            self.get_logger().warning(
                f'[타겟 찾기] "{target_label}" '
                f"{self.find_target_timeout:.1f}초 동안 못 찾음"
            )

        return result

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()


def main() -> None:
    model_path = Path(__file__).resolve().parent / "my_seg_best.pt"

    if not model_path.is_file():
        print(f"File not found: {model_path}", file=sys.stderr)
        sys.exit(1)

    suffix = model_path.suffix.lower()
    if suffix == ".pt":
        model = YOLO(str(model_path))
    elif suffix in {".onnx", ".engine"}:
        model = YOLO(str(model_path), task="detect")
    else:
        print(f"Unsupported model format: {suffix}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = ObjectDetectionNode(
        model,
        per_class_conf_threshold={"airpods": 0.9},
        auto_test_mode=True,
        auto_test_interval_sec=1.0,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
