#!/usr/bin/env python3
"""RealSense 손바닥 3D 기반 M0609 SpeedL 추적 노드.

통신 구조
---------
Robot Control -> Hand Tracking
  /arrived_goal  std_srvs/srv/Trigger

Hand Tracking -> VLA
  /hand_tracking_request  std_msgs/msg/Bool
      Trigger 서비스 요청을 수락하고 추적 세션을 시작했을 때 True 1회 발행
  /hand_arrived           std_msgs/msg/Bool
      TCP가 손바닥 목표 좌표에 안정적으로 도착했을 때 True 1회 발행

좌표 단위
---------
이 파일에서 카메라 점, 손바닥 점, TCP 점, 목표 점, SpeedL 선속도는 모두 mm 기준이다.
PointStamped 디버그 토픽도 프로젝트 요구에 맞춰 mm 값을 담는다.

NPY 해석
--------
요청된 직접 행렬식을 사용한다.
p_base = T_base_gripper(robot pose) @ T_gripper_camera(NPY) @ p_camera
NPY에는 역행렬을 적용하지 않는다.
기본 calibration_frame과 tcp_frame은 모두 gripper_tcp이다.

주의
----
목표는 손바닥 좌표 자체다. 실제 접촉 직전에는 힘/토크 감시와 낮은 속도가 필요하다.
기본 최대 속도와 도착 허용 오차는 보수적으로 설정되어 있다.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional, Sequence, Tuple

import cv2
import mediapipe as mp
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from dsr_msgs2.msg import SpeedlStream
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


PALM_LANDMARK_INDICES = (0, 5, 9, 13, 17)
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def quaternion_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError("Invalid zero-length quaternion")

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


class HandSpeedLTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__("realsense_hand_speedl_tracking")

        script_dir = Path(__file__).resolve().parent
        self._declare_parameters(script_dir)
        self._read_parameters()
        self._validate_parameters()

        self.bridge = CvBridge()
        self.gripper2cam_path = str(Path(self.transform_path).expanduser().resolve())
        self._validate_gripper2cam_file()

        self.latest_depth_image: Optional[np.ndarray] = None
        self.latest_depth_encoding = ""
        self.latest_depth_stamp_sec = 0.0
        self.latest_camera_info: Optional[CameraInfo] = None

        self.filtered_palm_base_mm: Optional[np.ndarray] = None
        self.latest_valid_palm_time = 0.0
        self.latest_palm_depth_mm: Optional[float] = None
        self.latest_tcp_mm: Optional[np.ndarray] = None
        self.latest_target_tcp_mm: Optional[np.ndarray] = None

        self.stable_frame_count = 0
        self.arrival_stable_count = 0
        self.tracking_enabled = False
        self.tracking = False
        self.arrival_published = False
        self.last_speed_nonzero = False

        self.hand_ok = False
        self.depth_sync_ok = False
        self.depth_ok = False
        self.calibration_tf_ok = False
        self.tcp_tf_ok = False
        self.base_transform_ok = False
        self.workspace_target_clamped = False
        self.status_text = "Waiting for /arrived_goal"

        self.last_inference_time = 0.0
        self.last_log_time = 0.0
        self.window_name = "Palm 3D Direct Tracking -> M0609 SpeedL"

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=self.model_complexity,
            min_detection_confidence=self.min_hand_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.speedl_publisher = self.create_publisher(
            SpeedlStream,
            self.speedl_topic,
            10,
        )
        self.palm_point_publisher = self.create_publisher(
            PointStamped,
            "/hand_follow/palm_point_base_mm",
            10,
        )
        self.target_point_publisher = self.create_publisher(
            PointStamped,
            "/hand_follow/target_point_base_mm",
            10,
        )

        # Hand Tracking -> VLA
        self.vla_signal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.tracking_started_publisher = self.create_publisher(
            Bool,
            "/hand_tracking_request",
            self.vla_signal_qos,
        )
        self.hand_arrived_publisher = self.create_publisher(
            Bool,
            "/hand_arrived",
            self.vla_signal_qos,
        )

        # Robot Control -> Hand Tracking
        self.start_tracking_service = self.create_service(
            Trigger,
            "/arrived_goal",
            self.start_tracking_callback,
        )

        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            qos_profile_sensor_data,
        )

        self.create_timer(1.0 / self.control_rate_hz, self.control_callback)

        self.get_logger().info(
            f"Calibration TF: {self.base_frame} <- {self.calibration_frame}"
        )
        self.get_logger().info(
            f"TCP TF: {self.base_frame} <- {self.tcp_frame}"
        )
        self.get_logger().info(
            "Tracking starts only after /arrived_goal Trigger service"
        )
        self.get_logger().warning(
            "Direct palm targeting enabled. Use conservative speed and robot safety limits."
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self, script_dir: Path) -> None:
        defaults = {
            "transform_path": str(script_dir / "T_gripper2camera.npy"),
            "transform_direction": "gripper_to_camera",
            "transform_translation_unit": "mm",
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "speedl_topic": "/dsr01/speedl_stream",
            "base_frame": "base_link",
            "calibration_frame": "gripper_tcp",
            "tcp_frame": "gripper_tcp",
            "model_complexity": 1,
            "min_hand_detection_confidence": 0.60,
            "min_tracking_confidence": 0.60,
            "max_inference_hz": 15.0,
            "depth_scale_16u_mm": 1.0,
            "depth_roi_radius": 5,
            "min_valid_depth_mm": 150.0,
            "max_valid_depth_mm": 2000.0,
            "max_rgb_depth_time_diff_sec": 0.20,
            "position_filter_alpha": 0.25,
            "max_hand_jump_mm": 250.0,
            "hand_timeout_sec": 0.30,
            "auto_start_stable_frames": 3,
            "control_rate_hz": 20.0,
            "kp": 0.10,
            "arrival_tolerance_mm": 10.0,
            "arrival_stable_cycles": 6,
            "max_linear_speed_mm_s": 45.0,
            "linear_acc_mm_s2": 80.0,
            "angular_acc_deg_s2": 100.0,
            "command_time_sec": 0.0,
            "max_tracking_distance_mm": 2000.0,
            "workspace_x_min_mm": 150.0,
            "workspace_x_max_mm": 750.0,
            "workspace_y_min_mm": -500.0,
            "workspace_y_max_mm": 500.0,
            "workspace_z_min_mm": 100.0,
            "workspace_z_max_mm": 850.0,
            "show_window": True,
            "draw_landmarks": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        names = [
            "transform_path",
            "transform_direction",
            "transform_translation_unit",
            "color_topic",
            "depth_topic",
            "camera_info_topic",
            "speedl_topic",
            "base_frame",
            "calibration_frame",
            "tcp_frame",
            "model_complexity",
            "min_hand_detection_confidence",
            "min_tracking_confidence",
            "max_inference_hz",
            "depth_scale_16u_mm",
            "depth_roi_radius",
            "min_valid_depth_mm",
            "max_valid_depth_mm",
            "max_rgb_depth_time_diff_sec",
            "position_filter_alpha",
            "max_hand_jump_mm",
            "hand_timeout_sec",
            "auto_start_stable_frames",
            "control_rate_hz",
            "kp",
            "arrival_tolerance_mm",
            "arrival_stable_cycles",
            "max_linear_speed_mm_s",
            "linear_acc_mm_s2",
            "angular_acc_deg_s2",
            "command_time_sec",
            "max_tracking_distance_mm",
            "workspace_x_min_mm",
            "workspace_x_max_mm",
            "workspace_y_min_mm",
            "workspace_y_max_mm",
            "workspace_z_min_mm",
            "workspace_z_max_mm",
            "show_window",
            "draw_landmarks",
        ]
        for name in names:
            setattr(self, name, self.get_parameter(name).value)

        self.transform_direction = str(self.transform_direction).strip().lower()
        self.transform_translation_unit = (
            str(self.transform_translation_unit).strip().lower()
        )

    def _validate_parameters(self) -> None:
        if self.control_rate_hz <= 0 or self.max_inference_hz <= 0:
            raise ValueError("control_rate_hz and max_inference_hz must be > 0")
        if not 0.0 < self.position_filter_alpha <= 1.0:
            raise ValueError("position_filter_alpha must be in (0, 1]")
        if self.auto_start_stable_frames < 1:
            raise ValueError("auto_start_stable_frames must be >= 1")
        if self.arrival_stable_cycles < 1:
            raise ValueError("arrival_stable_cycles must be >= 1")
        if self.arrival_tolerance_mm <= 0:
            raise ValueError("arrival_tolerance_mm must be > 0")

        for minimum, maximum, axis in (
            (self.workspace_x_min_mm, self.workspace_x_max_mm, "X"),
            (self.workspace_y_min_mm, self.workspace_y_max_mm, "Y"),
            (self.workspace_z_min_mm, self.workspace_z_max_mm, "Z"),
        ):
            if minimum >= maximum:
                raise ValueError(f"Invalid workspace {axis} range")

    # ------------------------------------------------------------------
    # Trigger service and VLA signals
    # ------------------------------------------------------------------
    def start_tracking_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.tracking_enabled:
            response.success = False
            response.message = "hand tracking is already active"
            return response

        self.filtered_palm_base_mm = None
        self.latest_valid_palm_time = 0.0
        self.latest_target_tcp_mm = None
        self.stable_frame_count = 0
        self.arrival_stable_count = 0
        self.arrival_published = False
        self.workspace_target_clamped = False
        self.tracking_enabled = True
        self.tracking = False
        self.status_text = "Tracking session started; searching hand"

        started = Bool()
        started.data = True
        self.tracking_started_publisher.publish(started)

        response.success = True
        response.message = "hand tracking started"
        self.get_logger().info(
            "Hand tracking started. Published /hand_tracking_request=True"
        )
        return response

    def _finish_tracking(self) -> None:
        self._send_zero_speed(force=True)
        self.tracking = False
        self.tracking_enabled = False
        self.arrival_published = True
        self.status_text = "Palm target reached"

        arrived = Bool()
        arrived.data = True
        self.hand_arrived_publisher.publish(arrived)
        self.get_logger().info(
            "Palm target reached. Published /hand_arrived=True"
        )

    # ------------------------------------------------------------------
    # Requested direct coordinate transform
    # ------------------------------------------------------------------
    def _validate_gripper2cam_file(self) -> None:
        path = Path(self.gripper2cam_path)
        if not path.is_file():
            raise FileNotFoundError(f"Transform file not found: {path}")
        matrix = np.asarray(np.load(str(path)), dtype=np.float64)
        if matrix.shape not in {(3, 4), (4, 4)}:
            raise ValueError(f"Transform must be 4x4 or 3x4: {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("Transform contains NaN or Inf")
        self.get_logger().info(
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
        unit = str(self.transform_translation_unit).strip().lower()
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

    # ------------------------------------------------------------------
    # Camera callbacks
    # ------------------------------------------------------------------
    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as error:
            self.status_text = f"Depth conversion failed: {error}"
            return

        if depth is None or depth.size == 0:
            return
        self.latest_depth_image = np.asarray(depth)
        self.latest_depth_encoding = msg.encoding
        self.latest_depth_stamp_sec = stamp_to_seconds(msg.header.stamp)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def color_callback(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_inference_time < 1.0 / self.max_inference_hz:
            return
        self.last_inference_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.status_text = f"Color conversion failed: {error}"
            return
        if frame is None or frame.size == 0:
            return

        self._reset_frame_flags()

        if not self.tracking_enabled:
            self.status_text = "Waiting for /arrived_goal"
            self._draw_status(frame)
            self._show_frame(frame)
            return

        depth = self.latest_depth_image
        camera_info = self.latest_camera_info
        color_stamp = stamp_to_seconds(msg.header.stamp)
        self.depth_sync_ok = (
            depth is not None
            and camera_info is not None
            and abs(color_stamp - self.latest_depth_stamp_sec)
            <= self.max_rgb_depth_time_diff_sec
        )

        if not self.depth_sync_ok:
            self.status_text = "RGB-Depth sync or CameraInfo missing"

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        result = self.hands.process(rgb)
        rgb.flags.writeable = True

        if result.multi_hand_landmarks:
            self.hand_ok = True
            hand_landmarks = result.multi_hand_landmarks[0]
            landmarks = hand_landmarks.landmark
            height, width = frame.shape[:2]
            palm_u, palm_v = self._palm_pixel(landmarks, width, height)

            if self.draw_landmarks:
                self._draw_hand(frame, landmarks)
            cv2.circle(frame, (palm_u, palm_v), 9, (0, 0, 255), 2, cv2.LINE_AA)

            if self.depth_sync_ok:
                depth_mm = self._median_depth_mm(
                    depth,
                    self.latest_depth_encoding,
                    palm_u,
                    palm_v,
                )
                if depth_mm is not None:
                    self.depth_ok = True
                    self.latest_palm_depth_mm = depth_mm
                    camera_point_mm = self._deproject_mm(
                        palm_u,
                        palm_v,
                        depth_mm,
                        camera_info,
                    )
                    base_point_mm = self._camera_to_base_mm(camera_point_mm)
                    if base_point_mm is not None:
                        self.base_transform_ok = True
                        if self._filter_palm(base_point_mm, now):
                            self.latest_valid_palm_time = now
                            self.stable_frame_count = min(
                                self.stable_frame_count + 1,
                                self.auto_start_stable_frames,
                            )
                            self._publish_point_mm(
                                self.palm_point_publisher,
                                base_point_mm,
                            )
                            self.status_text = "Palm coordinate valid"
                        else:
                            self.stable_frame_count = 0
                else:
                    self.status_text = "No valid palm depth"
                    self.stable_frame_count = 0
        else:
            self.status_text = "Hand not detected"
            self.stable_frame_count = 0

        self._draw_status(frame)
        self._show_frame(frame)

    def _show_frame(self, frame: np.ndarray) -> None:
        if not self.show_window:
            return
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            self.tracking_enabled = False
            self._send_zero_speed(force=True)
            if rclpy.ok():
                rclpy.shutdown()

    def _reset_frame_flags(self) -> None:
        self.hand_ok = False
        self.depth_sync_ok = False
        self.depth_ok = False
        self.base_transform_ok = False

    # ------------------------------------------------------------------
    # 3D processing in mm
    # ------------------------------------------------------------------
    @staticmethod
    def _palm_pixel(
        landmarks: Sequence,
        width: int,
        height: int,
    ) -> Tuple[int, int]:
        u = int(
            round(
                np.mean([float(landmarks[index].x) for index in PALM_LANDMARK_INDICES])
                * width
            )
        )
        v = int(
            round(
                np.mean([float(landmarks[index].y) for index in PALM_LANDMARK_INDICES])
                * height
            )
        )
        return max(0, min(width - 1, u)), max(0, min(height - 1, v))

    def _median_depth_mm(
        self,
        depth: np.ndarray,
        encoding: str,
        u: int,
        v: int,
    ) -> Optional[float]:
        height, width = depth.shape[:2]
        radius = int(self.depth_roi_radius)
        roi = depth[
            max(0, v - radius):min(height, v + radius + 1),
            max(0, u - radius):min(width, u + radius + 1),
        ]
        if roi.size == 0:
            return None

        values = roi.astype(np.float64)
        if roi.dtype == np.uint16 or encoding.lower() in ("16uc1", "mono16"):
            values *= float(self.depth_scale_16u_mm)
        else:
            # 32FC1 RealSense depth는 일반적으로 m이므로 mm로 변환한다.
            values *= 1000.0

        valid = values[
            np.isfinite(values)
            & (values >= self.min_valid_depth_mm)
            & (values <= self.max_valid_depth_mm)
        ]
        if valid.size < 5:
            return None
        return float(np.median(valid))

    @staticmethod
    def _deproject_mm(
        u: int,
        v: int,
        depth_mm: float,
        info: CameraInfo,
    ) -> np.ndarray:
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx <= 0 or fy <= 0:
            raise ValueError("Invalid CameraInfo intrinsics")
        return np.array(
            [
                (float(u) - cx) * depth_mm / fx,
                (float(v) - cy) * depth_mm / fy,
                depth_mm,
            ],
            dtype=np.float64,
        )

    def _lookup_pose_mm(
        self,
        child_frame: str,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            self.status_text = f"TF missing: {self.base_frame}<-{child_frame}"
            self.get_logger().warning(
                f"{self.status_text}: {error}",
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        position_mm = np.array(
            [
                float(translation.x) * 1000.0,
                float(translation.y) * 1000.0,
                float(translation.z) * 1000.0,
            ],
            dtype=np.float64,
        )
        rotation_matrix = quaternion_to_rotation_matrix(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        return position_mm, rotation_matrix

    def _camera_to_base_mm(
        self,
        camera_point_mm: np.ndarray,
    ) -> Optional[np.ndarray]:
        pose = self._lookup_pose_mm(self.calibration_frame)
        if pose is None:
            self.calibration_tf_ok = False
            return None

        self.calibration_tf_ok = True
        calibration_position_mm, calibration_rotation = pose
        zyz_deg = self._rotation_matrix_to_zyz_deg(calibration_rotation)
        robot_pos = np.concatenate((calibration_position_mm, zyz_deg))

        try:
            base_point_mm = self.transform_to_base(
                camera_point_mm,
                self.gripper2cam_path,
                robot_pos,
            )
        except (OSError, ValueError) as error:
            self.status_text = f"Coordinate transform failed: {error}"
            self.get_logger().error(self.status_text)
            return None

        if not np.all(np.isfinite(base_point_mm)):
            self.status_text = "Base transform returned NaN/Inf"
            return None
        return base_point_mm

    def _filter_palm(self, new_point_mm: np.ndarray, now: float) -> bool:
        if not np.all(np.isfinite(new_point_mm)):
            return False

        stale = (
            self.filtered_palm_base_mm is None
            or now - self.latest_valid_palm_time > self.hand_timeout_sec
        )
        if stale:
            self.filtered_palm_base_mm = new_point_mm.copy()
            return True

        jump_mm = float(np.linalg.norm(new_point_mm - self.filtered_palm_base_mm))
        if jump_mm > self.max_hand_jump_mm:
            self.status_text = f"Palm jump rejected: {jump_mm:.1f} mm"
            return False

        alpha = float(self.position_filter_alpha)
        self.filtered_palm_base_mm = (
            alpha * new_point_mm
            + (1.0 - alpha) * self.filtered_palm_base_mm
        )
        return True

    # ------------------------------------------------------------------
    # SpeedL control
    # ------------------------------------------------------------------
    def control_callback(self) -> None:
        if not self.tracking_enabled:
            self.tracking = False
            self._send_zero_speed()
            return

        now = time.monotonic()
        if (
            self.filtered_palm_base_mm is None
            or now - self.latest_valid_palm_time > self.hand_timeout_sec
            or self.stable_frame_count < self.auto_start_stable_frames
        ):
            self.tracking = False
            self.arrival_stable_count = 0
            self._send_zero_speed()
            return

        tcp_pose = self._lookup_pose_mm(self.tcp_frame)
        if tcp_pose is None:
            self.tcp_tf_ok = False
            self.tracking = False
            self.arrival_stable_count = 0
            self._send_zero_speed()
            return

        self.tcp_tf_ok = True
        tcp_mm, _ = tcp_pose
        self.latest_tcp_mm = tcp_mm

        palm_mm = self.filtered_palm_base_mm
        hand_distance_mm = float(np.linalg.norm(palm_mm - tcp_mm))
        if hand_distance_mm > self.max_tracking_distance_mm:
            self.tracking = False
            self.arrival_stable_count = 0
            self.status_text = (
                f"TCP-hand distance blocked: {hand_distance_mm:.1f} mm"
            )
            self._send_zero_speed()
            return

        # 사용자 요구: standoff 없이 손바닥 좌표 자체를 TCP 목표로 사용한다.
        target_mm, target_clamped = self._clamp_workspace(palm_mm)
        self.latest_target_tcp_mm = target_mm
        self.workspace_target_clamped = target_clamped
        self._publish_point_mm(self.target_point_publisher, target_mm)

        error_vector = target_mm - tcp_mm
        error_norm_mm = float(np.linalg.norm(error_vector))
        self.tracking = True

        if error_norm_mm <= self.arrival_tolerance_mm:
            self._send_zero_speed()
            if target_clamped:
                self.arrival_stable_count = 0
                self.status_text = "Palm target is outside workspace"
                return

            self.arrival_stable_count += 1
            self.status_text = (
                f"Arrival stabilizing {self.arrival_stable_count}/"
                f"{self.arrival_stable_cycles}"
            )
            if (
                self.arrival_stable_count >= self.arrival_stable_cycles
                and not self.arrival_published
            ):
                self._finish_tracking()
            return

        self.arrival_stable_count = 0
        target_direction = error_vector / error_norm_mm
        speed = clamp(
            self.kp * error_norm_mm,
            0.0,
            self.max_linear_speed_mm_s,
        )
        linear_velocity = self._limit_workspace_velocity(
            tcp_mm,
            target_direction * speed,
        )
        if float(np.linalg.norm(linear_velocity)) < 1.0e-6:
            self.status_text = "Workspace velocity limited"
            self._send_zero_speed()
            return

        self._publish_speedl(
            [
                float(linear_velocity[0]),
                float(linear_velocity[1]),
                float(linear_velocity[2]),
                0.0,
                0.0,
                0.0,
            ]
        )
        self.last_speed_nonzero = True
        self.status_text = (
            "Tracking clamped palm target"
            if target_clamped
            else "Tracking direct palm target"
        )

        if now - self.last_log_time >= 1.0:
            self.get_logger().info(
                f"hand_distance={hand_distance_mm:.1f} mm, "
                f"target_error={error_norm_mm:.1f} mm, "
                f"velocity={np.array2string(linear_velocity, precision=1)}"
            )
            self.last_log_time = now

    def _clamp_workspace(
        self,
        point_mm: np.ndarray,
    ) -> Tuple[np.ndarray, bool]:
        clamped = np.array(
            [
                clamp(
                    float(point_mm[0]),
                    self.workspace_x_min_mm,
                    self.workspace_x_max_mm,
                ),
                clamp(
                    float(point_mm[1]),
                    self.workspace_y_min_mm,
                    self.workspace_y_max_mm,
                ),
                clamp(
                    float(point_mm[2]),
                    self.workspace_z_min_mm,
                    self.workspace_z_max_mm,
                ),
            ],
            dtype=np.float64,
        )
        return clamped, not np.allclose(clamped, point_mm, atol=1.0e-6)

    def _limit_workspace_velocity(
        self,
        tcp_mm: np.ndarray,
        velocity: np.ndarray,
    ) -> np.ndarray:
        limited = np.asarray(velocity, dtype=np.float64).copy()
        minimums = np.array(
            [
                self.workspace_x_min_mm,
                self.workspace_y_min_mm,
                self.workspace_z_min_mm,
            ]
        )
        maximums = np.array(
            [
                self.workspace_x_max_mm,
                self.workspace_y_max_mm,
                self.workspace_z_max_mm,
            ]
        )
        for axis in range(3):
            if tcp_mm[axis] <= minimums[axis] and limited[axis] < 0:
                limited[axis] = 0.0
            elif tcp_mm[axis] >= maximums[axis] and limited[axis] > 0:
                limited[axis] = 0.0
        return limited

    def _publish_speedl(self, velocity: Sequence[float]) -> None:
        message = SpeedlStream()
        message.vel = [float(value) for value in velocity]
        message.acc = [
            float(self.linear_acc_mm_s2),
            float(self.angular_acc_deg_s2),
        ]
        message.time = float(self.command_time_sec)
        self.speedl_publisher.publish(message)

    def _send_zero_speed(self, force: bool = False) -> None:
        if not force and not self.last_speed_nonzero:
            return
        self._publish_speedl([0.0] * 6)
        self.last_speed_nonzero = False

    # ------------------------------------------------------------------
    # Display/debug
    # ------------------------------------------------------------------
    @staticmethod
    def _draw_hand(frame: np.ndarray, landmarks: Sequence) -> None:
        height, width = frame.shape[:2]
        points = [
            (
                max(0, min(width - 1, int(float(landmark.x) * width))),
                max(0, min(height - 1, int(float(landmark.y) * height))),
            )
            for landmark in landmarks
        ]
        for start, end in HAND_CONNECTIONS:
            cv2.line(
                frame,
                points[start],
                points[end],
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        for point in points:
            cv2.circle(frame, point, 3, (0, 255, 0), -1, cv2.LINE_AA)

    def _draw_status(self, frame: np.ndarray) -> None:
        lines = [
            f"Session: {'ACTIVE' if self.tracking_enabled else 'WAITING SERVICE'}",
            f"Motion: {'TRACKING' if self.tracking else 'STOPPED'}",
            f"Stable hand: {self.stable_frame_count}/{self.auto_start_stable_frames}",
            f"Arrival: {self.arrival_stable_count}/{self.arrival_stable_cycles}",
            f"Hand: {'OK' if self.hand_ok else 'NO'}",
            f"RGB-Depth: {'OK' if self.depth_sync_ok else 'NO'}",
            f"Base TF: {'OK' if self.base_transform_ok else 'NO'}",
            f"Target: {'CLAMPED' if self.workspace_target_clamped else 'PALM DIRECT'}",
            f"Status: {self.status_text}",
        ]
        if self.latest_palm_depth_mm is not None:
            lines.append(f"Palm depth: {self.latest_palm_depth_mm:.1f} mm")
        if self.latest_tcp_mm is not None:
            lines.append(
                "TCP base mm: "
                f"[{self.latest_tcp_mm[0]:.1f}, "
                f"{self.latest_tcp_mm[1]:.1f}, "
                f"{self.latest_tcp_mm[2]:.1f}]"
            )
        if self.filtered_palm_base_mm is not None:
            lines.append(
                "Palm base mm: "
                f"[{self.filtered_palm_base_mm[0]:.1f}, "
                f"{self.filtered_palm_base_mm[1]:.1f}, "
                f"{self.filtered_palm_base_mm[2]:.1f}]"
            )

        for index, text in enumerate(lines):
            cv2.putText(
                frame,
                text,
                (20, 32 + index * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def _publish_point_mm(
        self,
        publisher,
        point_mm: np.ndarray,
    ) -> None:
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.point.x = float(point_mm[0])
        message.point.y = float(point_mm[1])
        message.point.z = float(point_mm[2])
        publisher.publish(message)

    def destroy_node(self) -> bool:
        self.tracking_enabled = False
        self.tracking = False
        try:
            for _ in range(3):
                self._send_zero_speed(force=True)
                time.sleep(0.02)
        except Exception:
            pass
        try:
            self.hands.close()
        finally:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[HandSpeedLTrackingNode] = None
    try:
        node = HandSpeedLTrackingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"[ERROR] {error}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
