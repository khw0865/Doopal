#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
from typing import Any, Mapping, Sequence

import rclpy
from hey_doopal_msg.srv import (
    GetDbData,
    GetFixedPose,
    GetObjectCoordinate,
    GetScanCase,
)
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from redis_store import RedisStore, utc_now
from runtime_log import append_runtime_log


class RedisObjectBridge(Node):
    """ROS 2와 Redis/Flask UI 사이 Bridge 노드.

    기존 기능:
    - 객체 인식 결과를 Redis에 저장
    - 일반 DB 조회: /assistive/get_db_data
    - VLA 상태/대화/런타임 로그 처리

    추가 기능:
    - 로봇 컨트롤 노드가 객체명을 보내면 Redis 객체 레코드를 조회
    - JSON/딕셔너리에서 x, y, z를 꺼내 [x, y, z] 리스트로 변환
    - /assistive/get_object_coordinate 서비스 응답의 float64[3]으로 전달

    좌표 출력 단위는 항상 mm이다.
    """

    USER_UI_STATE_KEY = "assistive_robot:user_ui_state"

    def __init__(self) -> None:
        super().__init__("assistive_robot_redis_bridge")

        self.store = RedisStore()
        self.store.ping()
        fixed_result = self.store.initialize_fixed_data()

        self.chat_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.create_subscription(
            String,
            "/assistive/object_detection",
            self.object_detection_callback,
            10,
        )
        self.create_subscription(
            String,
            "/assistive/object_moved",
            self.object_moved_callback,
            10,
        )
        self.create_subscription(
            String,
            "/assistive/vla_state",
            self.vla_state_callback,
            10,
        )
        self.create_subscription(
            String,
            "/ui_chat_log",
            self.ui_chat_log_callback,
            self.chat_qos,
        )
        self.create_subscription(
            Bool,
            "/hand_tracking_request",
            self.hand_tracking_started_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/hand_arrived",
            self.hand_arrived_callback,
            10,
        )
        self.create_subscription(
            String,
            "/assistive/system_log",
            self.system_log_callback,
            50,
        )

        filter_text = os.getenv(
            "ROSOUT_LOG_FILTER",
            "m0609,dsr,robot_control,hand,object_detection,redis_bridge,vla",
        )
        self.rosout_filters = {
            item.strip().lower()
            for item in filter_text.split(",")
            if item.strip()
        }
        self.rosout_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1000,
        )
        self.create_subscription(
            Log,
            "/rosout",
            self.rosout_callback,
            self.rosout_qos,
        )

        # 기존 범용 DB 조회 서비스
        self.get_db_data_service = self.create_service(
            GetDbData,
            "/assistive/get_db_data",
            self.get_db_data_callback,
        )

        # 객체 좌표 전용 서비스: [x, y, z]
        self.get_object_coordinate_service = self.create_service(
            GetObjectCoordinate,
            "/assistive/get_object_coordinate",
            self.get_object_coordinate_callback,
        )

        # 웨이포인트 전용 서비스:
        # [x, y, z, rx, ry, rz]
        self.get_fixed_pose_service = self.create_service(
            GetFixedPose,
            "/assistive/get_fixed_pose",
            self.get_fixed_pose_callback,
        )

        # CASE 전용 서비스:
        # DB의 waypoints 순서에 따라 두 개의 6D pose를 반환
        self.get_scan_case_service = self.create_service(
            GetScanCase,
            "/assistive/get_scan_case",
            self.get_scan_case_callback,
        )

        self.get_logger().info(f"fixed data: {fixed_result}")
        self.get_logger().info(
            "Redis bridge started: "
            "query=/assistive/get_db_data, "
            "coordinate=/assistive/get_object_coordinate, "
            "fixed_pose=/assistive/get_fixed_pose, "
            "scan_case=/assistive/get_scan_case"
        )
        self._runtime_log(
            source="redis_bridge",
            level="INFO",
            category="startup",
            message="Redis Bridge started",
            details={
                "fixed_data": fixed_result,
                "query_service": "/assistive/get_db_data",
                "coordinate_service": "/assistive/get_object_coordinate",
                "fixed_pose_service": "/assistive/get_fixed_pose",
                "scan_case_service": "/assistive/get_scan_case",
            },
        )

    # ------------------------------------------------------------------
    # Common helpers
    # ------------------------------------------------------------------
    def _runtime_log(
        self,
        *,
        source: str,
        level: str,
        message: str,
        category: str = "system",
        details: Mapping[str, Any] | None = None,
    ) -> None:
        try:
            append_runtime_log(
                source=source,
                level=level,
                message=message,
                category=category,
                details=details,
            )
        except OSError as error:
            self.get_logger().warning(
                f"Runtime log file write failed: {error}",
                throttle_duration_sec=5.0,
            )

    def _parse(
        self,
        message: String,
        *,
        source: str,
    ) -> dict[str, Any] | None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"Invalid JSON from {source}: {error}")
            self._runtime_log(
                source=source,
                level="ERROR",
                category="communication",
                message="Invalid JSON received",
                details={"error": str(error)},
            )
            return None

        if not isinstance(payload, dict):
            self.get_logger().error(
                f"JSON payload from {source} must be an object"
            )
            return None
        return payload

    @staticmethod
    def _record_name(payload: Mapping[str, Any]) -> str:
        value = (
            payload.get("record_name")
            or payload.get("object_name")
            or payload.get("class_name")
        )
        if value is None:
            raise ValueError(
                "record_name, object_name, or class_name is required"
            )
        return str(value)

    @staticmethod
    def _record_data(payload: Mapping[str, Any]) -> Mapping[str, Any]:
        if "data" in payload:
            data = payload.get("data")
            if not isinstance(data, Mapping):
                raise ValueError("data must be a JSON object")
            return data

        return {
            key: value
            for key, value in payload.items()
            if key not in {"record_name", "object_name", "replace"}
        }

    @staticmethod
    def _as_mapping(value: Any) -> Mapping[str, Any]:
        """Redis 조회 결과가 JSON 문자열이어도 dict로 변환한다."""
        if isinstance(value, Mapping):
            return value

        if isinstance(value, str):
            try:
                decoded = json.loads(value)
            except json.JSONDecodeError as error:
                raise ValueError(
                    f"DB JSON 해석 실패: {error}"
                ) from error

            if not isinstance(decoded, Mapping):
                raise ValueError("DB 객체 레코드는 JSON object여야 합니다")
            return decoded

        raise ValueError(
            f"지원하지 않는 DB 레코드 타입: {type(value).__name__}"
        )

    @classmethod
    def _extract_coordinate_mm(
        cls,
        record_value: Any,
    ) -> tuple[list[float], str]:
        """객체 레코드에서 좌표를 꺼내 항상 mm 리스트로 반환한다.

        허용 형식:
        1. {"data": {"x": 1, "y": 2, "z": 3, ...}}
        2. {"x": 1, "y": 2, "z": 3, ...}
        3. {"data": {"coordinate": [1, 2, 3], ...}}
        4. {"data": {"position": {"x": 1, "y": 2, "z": 3}}}
        5. {"data": {"position": [1, 2, 3]}}

        단위:
        - coordinate_unit 또는 unit이 "mm"이면 그대로 사용
        - "m"이면 1000을 곱함
        - 단위 필드가 없으면 안전을 위해 오류 처리
          (구형 m 데이터와 신형 mm 데이터가 섞이면 위험하기 때문)
        """
        record = cls._as_mapping(record_value)

        raw_data = record.get("data", record)
        data = cls._as_mapping(raw_data)

        unit_value = (
            data.get("coordinate_unit")
            or data.get("unit")
            or record.get("coordinate_unit")
            or record.get("unit")
        )
        unit = str(unit_value or "").strip().lower()

        coordinate: Sequence[Any] | None = None

        if all(axis in data for axis in ("x", "y", "z")):
            coordinate = [data["x"], data["y"], data["z"]]
        elif isinstance(data.get("coordinate"), Sequence) and not isinstance(
            data.get("coordinate"),
            (str, bytes, bytearray),
        ):
            coordinate = data["coordinate"]
        else:
            position = data.get("position")
            if isinstance(position, Mapping) and all(
                axis in position for axis in ("x", "y", "z")
            ):
                coordinate = [
                    position["x"],
                    position["y"],
                    position["z"],
                ]
            elif isinstance(position, Sequence) and not isinstance(
                position,
                (str, bytes, bytearray),
            ):
                coordinate = position

        if coordinate is None or len(coordinate) < 3:
            raise ValueError(
                "객체 레코드에 x/y/z 또는 coordinate[3] 좌표가 없습니다"
            )

        try:
            xyz = [
                float(coordinate[0]),
                float(coordinate[1]),
                float(coordinate[2]),
            ]
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"좌표를 float로 변환할 수 없습니다: {coordinate}"
            ) from error

        if not all(value == value and abs(value) != float("inf") for value in xyz):
            raise ValueError("좌표에 NaN 또는 Inf가 포함되어 있습니다")

        if unit in {"mm", "millimeter", "millimeters"}:
            coordinate_mm = xyz
        elif unit in {"m", "meter", "meters"}:
            coordinate_mm = [value * 1000.0 for value in xyz]
        else:
            raise ValueError(
                "coordinate_unit이 없습니다. "
                "객체 검출 노드가 coordinate_unit='mm'을 저장하도록 하세요"
            )

        frame_id = str(
            data.get("frame_id")
            or record.get("frame_id")
            or "base_link"
        ).strip()

        return coordinate_mm, frame_id or "base_link"

    @staticmethod
    def _is_sequence(value: Any) -> bool:
        return (
            isinstance(value, Sequence)
            and not isinstance(
                value,
                (str, bytes, bytearray),
            )
        )

    @staticmethod
    def _finite_float_list(
        values: Sequence[Any],
        *,
        expected_length: int,
        field_name: str,
    ) -> list[float]:
        if len(values) < expected_length:
            raise ValueError(
                f"{field_name}에는 {expected_length}개 값이 필요합니다: "
                f"{values}"
            )

        try:
            result = [
                float(values[index])
                for index in range(expected_length)
            ]
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{field_name} 값을 float로 변환할 수 없습니다: {values}"
            ) from error

        if not all(math.isfinite(value) for value in result):
            raise ValueError(
                f"{field_name}에 NaN 또는 Inf가 포함되어 있습니다"
            )

        return result

    @staticmethod
    def _mapping_xyz(
        mapping: Mapping[str, Any],
    ) -> list[Any] | None:
        key_sets = (
            ("x", "y", "z"),
            ("x_mm", "y_mm", "z_mm"),
        )
        for keys in key_sets:
            if all(key in mapping for key in keys):
                return [mapping[key] for key in keys]
        return None

    @staticmethod
    def _mapping_orientation(
        mapping: Mapping[str, Any],
    ) -> list[Any] | None:
        key_sets = (
            ("rx", "ry", "rz"),
            ("a", "b", "c"),
            ("roll", "pitch", "yaw"),
        )
        for keys in key_sets:
            if all(key in mapping for key in keys):
                return [mapping[key] for key in keys]
        return None

    @classmethod
    def _extract_pose6(
        cls,
        record_value: Any,
    ) -> tuple[list[float], str]:
        """고정 웨이포인트를 Doosan posx 6D 리스트로 변환한다.

        반환:
            [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg], frame_id

        허용 형식:
        - {"pose": [x,y,z,rx,ry,rz]}
        - {"position": [x,y,z,rx,ry,rz]}
        - {"x":..., "y":..., "z":..., "rx":..., "ry":..., "rz":...}
        - {
            "position": {"x":..., "y":..., "z":...},
            "orientation": {"rx":..., "ry":..., "rz":...}
          }

        웨이포인트는 두산 posx 데이터이므로 단위 필드가 없으면
        위치 mm, 각도 deg로 간주한다.
        """
        record = cls._as_mapping(record_value)
        raw_data = record.get("data", record)
        data = cls._as_mapping(raw_data)

        linear_unit = str(
            data.get("coordinate_unit")
            or data.get("position_unit")
            or data.get("linear_unit")
            or data.get("unit")
            or record.get("coordinate_unit")
            or record.get("position_unit")
            or record.get("linear_unit")
            or record.get("unit")
            or "mm"
        ).strip().lower()

        angle_unit = str(
            data.get("angle_unit")
            or data.get("orientation_unit")
            or record.get("angle_unit")
            or record.get("orientation_unit")
            or "deg"
        ).strip().lower()

        pose_values: Sequence[Any] | None = None

        # 1) 한 배열에 6개 값이 저장된 형식
        for key in (
            "pose",
            "posx",
            "coordinates",
            "coordinate",
            "values",
        ):
            candidate = data.get(key)
            if cls._is_sequence(candidate) and len(candidate) >= 6:
                pose_values = candidate
                break

        # position 자체가 6개 배열인 형식
        if pose_values is None:
            candidate = data.get("position")
            if cls._is_sequence(candidate) and len(candidate) >= 6:
                pose_values = candidate

        # 2) x/y/z/rx/ry/rz가 직접 저장된 형식
        if pose_values is None:
            xyz = cls._mapping_xyz(data)
            orientation = cls._mapping_orientation(data)
            if xyz is not None and orientation is not None:
                pose_values = xyz + orientation

        # 3) position/orientation이 각각 dict인 형식
        if pose_values is None:
            position = data.get("position")
            orientation_data = (
                data.get("orientation")
                or data.get("rotation")
                or data.get("rpy")
            )

            if isinstance(position, Mapping):
                xyz = cls._mapping_xyz(position)
            else:
                xyz = None

            if isinstance(orientation_data, Mapping):
                orientation = cls._mapping_orientation(
                    orientation_data
                )
            elif (
                cls._is_sequence(orientation_data)
                and len(orientation_data) >= 3
            ):
                orientation = list(orientation_data[:3])
            else:
                orientation = None

            if xyz is not None and orientation is not None:
                pose_values = xyz + orientation

        if pose_values is None:
            raise ValueError(
                "웨이포인트에 6D pose가 없습니다. "
                "pose=[x,y,z,rx,ry,rz] 또는 "
                "x/y/z/rx/ry/rz 형식으로 저장하세요"
            )

        pose = cls._finite_float_list(
            pose_values,
            expected_length=6,
            field_name="pose",
        )

        if linear_unit in {
            "m",
            "meter",
            "meters",
        }:
            pose[0:3] = [
                value * 1000.0
                for value in pose[0:3]
            ]
        elif linear_unit not in {
            "mm",
            "millimeter",
            "millimeters",
        }:
            raise ValueError(
                f"지원하지 않는 위치 단위입니다: {linear_unit}"
            )

        if angle_unit in {
            "rad",
            "radian",
            "radians",
        }:
            pose[3:6] = [
                math.degrees(value)
                for value in pose[3:6]
            ]
        elif angle_unit not in {
            "deg",
            "degree",
            "degrees",
        }:
            raise ValueError(
                f"지원하지 않는 각도 단위입니다: {angle_unit}"
            )

        frame_id = str(
            data.get("frame_id")
            or record.get("frame_id")
            or "base_link"
        ).strip()

        return pose, frame_id or "base_link"

    @classmethod
    def _waypoint_reference(
        cls,
        value: Any,
        *,
        default_name: str,
    ) -> tuple[str, Any | None]:
        """CASE waypoint에서 이름 또는 inline pose를 추출한다."""
        if isinstance(value, str):
            name = value.strip()
            if not name:
                raise ValueError("CASE waypoint 이름이 비어 있습니다")
            return name, None

        if not isinstance(value, Mapping):
            raise ValueError(
                "CASE waypoint는 pose 이름 문자열 또는 JSON object여야 합니다"
            )

        for key in (
            "pose_name",
            "waypoint_name",
            "fixed_point",
            "name",
        ):
            candidate = value.get(key)
            if isinstance(candidate, str) and candidate.strip():
                return candidate.strip(), None

        pose_field = value.get("pose")
        if isinstance(pose_field, str) and pose_field.strip():
            return pose_field.strip(), None

        # pose가 실제 6개 배열이거나, waypoint 자체에 6D 값이 있는 형식
        return default_name, value

    def _extract_case_two_poses(
        self,
        case_record: Any,
    ) -> tuple[
        str,
        list[float],
        str,
        list[float],
        str,
    ]:
        """CASE의 두 waypoint를 DB 순서 그대로 6D pose로 변환한다."""
        record = self._as_mapping(case_record)
        raw_data = record.get("data", record)
        data = self._as_mapping(raw_data)

        waypoints = (
            data.get("waypoints")
            or data.get("poses")
            or data.get("sequence")
            or data.get("route")
        )

        # first/second 필드 형식도 허용
        if not self._is_sequence(waypoints):
            first = (
                data.get("first_pose")
                or data.get("pose1")
                or data.get("start_pose")
            )
            second = (
                data.get("second_pose")
                or data.get("pose2")
                or data.get("end_pose")
            )
            if first is not None and second is not None:
                waypoints = [first, second]

        if not self._is_sequence(waypoints):
            raise ValueError(
                "CASE에 waypoints 배열이 없습니다"
            )

        if len(waypoints) != 2:
            raise ValueError(
                "GetScanCase 서비스는 정확히 2개의 waypoint를 반환합니다. "
                f"DB CASE에는 {len(waypoints)}개가 들어 있습니다"
            )

        resolved_names: list[str] = []
        resolved_poses: list[list[float]] = []
        frame_ids: list[str] = []

        for index, waypoint in enumerate(waypoints):
            pose_name, inline_record = self._waypoint_reference(
                waypoint,
                default_name=f"waypoint_{index + 1}",
            )

            if inline_record is None:
                fixed_record = self.store.get_fixed_point(pose_name)
                if fixed_record is None:
                    raise ValueError(
                        f"CASE가 참조한 웨이포인트를 찾을 수 없습니다: "
                        f"{pose_name}"
                    )
                pose, frame_id = self._extract_pose6(fixed_record)
            else:
                pose, frame_id = self._extract_pose6(inline_record)

            resolved_names.append(pose_name)
            resolved_poses.append(pose)
            frame_ids.append(frame_id)

        if frame_ids[0] != frame_ids[1]:
            raise ValueError(
                "CASE의 두 waypoint frame_id가 서로 다릅니다: "
                f"{frame_ids[0]}, {frame_ids[1]}"
            )

        return (
            resolved_names[0],
            resolved_poses[0],
            resolved_names[1],
            resolved_poses[1],
            frame_ids[0],
        )

    # ------------------------------------------------------------------
    # 고정 웨이포인트 6D pose 조회 서비스
    # ------------------------------------------------------------------
    def get_fixed_pose_callback(
        self,
        request: GetFixedPose.Request,
        response: GetFixedPose.Response,
    ) -> GetFixedPose.Response:
        pose_name = request.pose_name.strip()

        response.success = False
        response.pose = [0.0] * 6
        response.coordinate_unit = "mm"
        response.angle_unit = "deg"
        response.frame_id = "base_link"
        response.json_data = ""

        if not pose_name:
            response.message = "pose_name이 비어 있습니다"
            return response

        try:
            record = self.store.get_fixed_point(pose_name)
            if record is None:
                response.message = (
                    f"고정 웨이포인트를 찾을 수 없습니다: {pose_name}"
                )
                return response

            pose, frame_id = self._extract_pose6(record)

            response.success = True
            response.pose = [float(value) for value in pose]
            response.frame_id = frame_id
            response.json_data = json.dumps(
                record,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
            response.message = "웨이포인트 조회 성공"

            self.get_logger().info(
                f"Fixed pose query success: {pose_name} -> "
                f"{list(response.pose)}"
            )
        except (KeyError, TypeError, ValueError) as error:
            response.message = str(error)
            self.get_logger().warning(
                f"Fixed pose query failed ({pose_name}): {error}"
            )
        except Exception as error:
            response.message = (
                f"웨이포인트 조회 중 오류가 발생했습니다: {error}"
            )
            self.get_logger().error(response.message)

        return response

    # ------------------------------------------------------------------
    # CASE 순서형 두 개 6D pose 조회 서비스
    # ------------------------------------------------------------------
    def get_scan_case_callback(
        self,
        request: GetScanCase.Request,
        response: GetScanCase.Response,
    ) -> GetScanCase.Response:
        case_name = request.case_name.strip()

        response.success = False
        response.first_pose_name = ""
        response.first_pose = [0.0] * 6
        response.second_pose_name = ""
        response.second_pose = [0.0] * 6
        response.coordinate_unit = "mm"
        response.angle_unit = "deg"
        response.frame_id = "base_link"
        response.json_data = ""

        if not case_name:
            response.message = "case_name이 비어 있습니다"
            return response

        try:
            case_record = self.store.get_scan_case(case_name)
            if case_record is None:
                response.message = (
                    f"스캔 CASE를 찾을 수 없습니다: {case_name}"
                )
                return response

            (
                first_name,
                first_pose,
                second_name,
                second_pose,
                frame_id,
            ) = self._extract_case_two_poses(case_record)

            response.success = True
            response.first_pose_name = first_name
            response.first_pose = [
                float(value)
                for value in first_pose
            ]
            response.second_pose_name = second_name
            response.second_pose = [
                float(value)
                for value in second_pose
            ]
            response.frame_id = frame_id
            response.json_data = json.dumps(
                case_record,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
            response.message = "CASE 조회 성공"

            self.get_logger().info(
                f"Scan case query success: {case_name} -> "
                f"{first_name} {list(response.first_pose)} -> "
                f"{second_name} {list(response.second_pose)}"
            )
        except (KeyError, TypeError, ValueError) as error:
            response.message = str(error)
            self.get_logger().warning(
                f"Scan case query failed ({case_name}): {error}"
            )
        except Exception as error:
            response.message = (
                f"CASE 조회 중 오류가 발생했습니다: {error}"
            )
            self.get_logger().error(response.message)

        return response

    # ------------------------------------------------------------------
    # 객체 좌표 전용 조회 서비스
    # ------------------------------------------------------------------
    def get_object_coordinate_callback(
        self,
        request: GetObjectCoordinate.Request,
        response: GetObjectCoordinate.Response,
    ) -> GetObjectCoordinate.Response:
        object_name = request.object_name.strip()

        # 모든 실패 응답의 기본값
        response.success = False
        response.has_coordinate = False
        response.coordinate = [0.0, 0.0, 0.0]
        response.coordinate_unit = "mm"
        response.frame_id = "base_link"
        response.json_data = ""

        if not object_name:
            response.message = "object_name이 비어 있습니다"
            return response

        try:
            # RedisStore가 dict를 반환해도 되고, JSON 문자열을 반환해도 됨.
            record = self.store.get_object_record(object_name)
            if record is None:
                response.message = (
                    f"객체를 찾을 수 없습니다: {object_name}"
                )
                return response

            coordinate_mm, frame_id = self._extract_coordinate_mm(record)

            response.success = True
            response.has_coordinate = True
            response.coordinate = [
                float(coordinate_mm[0]),
                float(coordinate_mm[1]),
                float(coordinate_mm[2]),
            ]
            response.coordinate_unit = "mm"
            response.frame_id = frame_id
            response.json_data = json.dumps(
                record,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
            response.message = "객체 좌표 조회 성공"

            self.get_logger().info(
                f"Coordinate query success: {object_name} -> "
                f"{list(response.coordinate)} mm, frame={frame_id}"
            )
            self._runtime_log(
                source="redis_bridge",
                level="INFO",
                category="communication",
                message=f"Coordinate query success: {object_name}",
                details={
                    "coordinate_mm": list(response.coordinate),
                    "frame_id": frame_id,
                },
            )
        except (KeyError, TypeError, ValueError) as error:
            response.message = str(error)
            self.get_logger().warning(
                f"Coordinate query failed ({object_name}): {error}"
            )
            self._runtime_log(
                source="redis_bridge",
                level="WARN",
                category="communication",
                message=f"Coordinate query failed: {object_name}",
                details={"error": str(error)},
            )
        except Exception as error:
            response.message = (
                f"객체 좌표 조회 중 오류가 발생했습니다: {error}"
            )
            self.get_logger().error(response.message)
            self._runtime_log(
                source="redis_bridge",
                level="ERROR",
                category="communication",
                message=response.message,
            )

        return response

    # ------------------------------------------------------------------
    # 기존 Redis 범용 조회 서비스
    # ------------------------------------------------------------------
    def get_db_data_callback(
        self,
        request: GetDbData.Request,
        response: GetDbData.Response,
    ) -> GetDbData.Response:
        data_type = request.data_type.strip().lower()
        name = request.name.strip()

        aliases = {
            "object_list": "objects",
            "list_objects": "objects",
            "fixed": "fixed_point",
            "waypoint": "fixed_point",
            "waypoints": "fixed_points",
            "list_fixed_points": "fixed_points",
            "case": "scan_case",
            "cases": "scan_cases",
            "list_scan_cases": "scan_cases",
            "conversation": "conversations",
        }
        data_type = aliases.get(data_type, data_type)

        try:
            if data_type == "object":
                if not name:
                    raise ValueError("object 조회에는 name이 필요합니다")
                result = self.store.get_object_record(name)
                if result is None:
                    return self._not_found(
                        response,
                        f"객체를 찾을 수 없습니다: {name}",
                    )
            elif data_type == "objects":
                result = self.store.list_objects()
            elif data_type == "fixed_point":
                if not name:
                    raise ValueError(
                        "fixed_point 조회에는 name이 필요합니다"
                    )
                result = self.store.get_fixed_point(name)
                if result is None:
                    return self._not_found(
                        response,
                        f"고정 좌표를 찾을 수 없습니다: {name}",
                    )
            elif data_type == "fixed_points":
                result = self.store.list_fixed_points()
            elif data_type == "scan_case":
                if not name:
                    raise ValueError(
                        "scan_case 조회에는 name이 필요합니다"
                    )
                result = self.store.get_scan_case(name)
                if result is None:
                    return self._not_found(
                        response,
                        f"스캔 CASE를 찾을 수 없습니다: {name}",
                    )
            elif data_type == "scan_cases":
                result = self.store.list_scan_cases()
            elif data_type == "conversations":
                limit = 100
                if name:
                    try:
                        limit = int(name)
                    except ValueError as error:
                        raise ValueError(
                            "conversations의 name에는 조회 개수만 "
                            "입력할 수 있습니다"
                        ) from error
                result = self.store.list_conversations(limit=limit)
            else:
                raise ValueError(
                    "지원하지 않는 data_type입니다. 사용 가능: "
                    "object, objects, fixed_point, fixed_points, "
                    "scan_case, scan_cases, conversations"
                )

            response.success = True
            response.json_data = json.dumps(
                result,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
            response.message = "조회 성공"

            label = f"{data_type}:{name}" if name else data_type
            self.get_logger().info(f"DB query success: {label}")
            self._runtime_log(
                source="redis_bridge",
                level="INFO",
                category="communication",
                message=f"DB query success: {label}",
            )
        except (KeyError, TypeError, ValueError) as error:
            response.success = False
            response.json_data = ""
            response.message = str(error)
            self.get_logger().warning(f"DB query failed: {error}")
            self._runtime_log(
                source="redis_bridge",
                level="WARN",
                category="communication",
                message=f"DB query failed: {error}",
            )
        except Exception as error:
            response.success = False
            response.json_data = ""
            response.message = (
                f"DB 조회 중 오류가 발생했습니다: {error}"
            )
            self.get_logger().error(response.message)
            self._runtime_log(
                source="redis_bridge",
                level="ERROR",
                category="communication",
                message=response.message,
            )

        return response

    @staticmethod
    def _not_found(
        response: GetDbData.Response,
        message: str,
    ) -> GetDbData.Response:
        response.success = False
        response.json_data = ""
        response.message = message
        return response

    # ------------------------------------------------------------------
    # Redis save callbacks
    # ------------------------------------------------------------------
    def object_detection_callback(self, message: String) -> None:
        payload = self._parse(message, source="object_detection")
        if payload is None:
            return

        try:
            record_name = self._record_name(payload)
            item = self.store.save_object_record(
                record_name=record_name,
                data=self._record_data(payload),
                replace=bool(payload.get("replace", False)),
            )
            field_count = len(item.get("data", {}))
            log_message = (
                f"Object updated: {record_name} ({field_count} fields)"
            )
            self.get_logger().info(log_message)
            self._runtime_log(
                source="object_detection",
                level="INFO",
                category="communication",
                message=log_message,
            )
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f"Object payload error: {error}")
            self._runtime_log(
                source="object_detection",
                level="ERROR",
                category="communication",
                message=f"Object payload error: {error}",
            )

    def object_moved_callback(self, message: String) -> None:
        payload = self._parse(message, source="object_moved")
        if payload is None:
            return

        try:
            record_name = self._record_name(payload)
            if isinstance(payload.get("data"), Mapping):
                fields = payload["data"]
            else:
                fields = {
                    "last_moved": {
                        "destination": payload.get("destination"),
                        "position": payload.get("position"),
                        "timestamp": payload.get("timestamp", utc_now()),
                    }
                }

            self.store.update_object_fields(
                record_name=record_name,
                fields=fields,
            )
            log_message = (
                f"Object moved data updated: {record_name}"
            )
            self.get_logger().info(log_message)
            self._runtime_log(
                source="robot_control",
                level="INFO",
                category="communication",
                message=log_message,
            )
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f"Moved payload error: {error}")
            self._runtime_log(
                source="robot_control",
                level="ERROR",
                category="communication",
                message=f"Moved payload error: {error}",
            )

    def vla_state_callback(self, message: String) -> None:
        payload = self._parse(message, source="vla_state")
        if payload is None:
            return

        state = (
            str(payload.get("state", "idle")).strip().lower()
            or "idle"
        )
        status_message = str(payload.get("message", ""))

        self.store.redis.hset(
            self.USER_UI_STATE_KEY,
            mapping={
                "state": state,
                "message": status_message,
            },
        )
        self._runtime_log(
            source="vla",
            level="INFO",
            category="state",
            message=f"VLA state changed: {state}",
            details={"message": status_message},
        )

    def ui_chat_log_callback(self, message: String) -> None:
        payload = self._parse(message, source="ui_chat_log")
        if payload is None:
            return

        speaker = str(payload.get("speaker", "")).strip().upper()
        text = str(payload.get("text", "")).strip()
        session_id = (
            str(payload.get("session_id", "default")).strip()
            or "default"
        )

        role_mapping = {
            "USER": "user",
            "ASSISTANT": "assistant",
        }
        role = role_mapping.get(speaker)
        if role is None:
            self.get_logger().warning(
                "ui_chat_log speaker must be USER or ASSISTANT"
            )
            return
        if not text:
            self.get_logger().warning("ui_chat_log text is empty")
            return

        self.store.append_conversation(
            role=role,
            text=text,
            session_id=session_id,
            source="ui_chat_log",
            state="chat",
        )

        ui_field = (
            "user_text"
            if role == "user"
            else "assistant_text"
        )
        self.store.redis.hset(
            self.USER_UI_STATE_KEY,
            mapping={ui_field: text},
        )
        self._runtime_log(
            source="vla",
            level="INFO",
            category="communication",
            message=f"UI chat received: {speaker}",
        )

    # ------------------------------------------------------------------
    # Runtime-only log callbacks
    # ------------------------------------------------------------------
    def hand_tracking_started_callback(
        self,
        message: Bool,
    ) -> None:
        if not message.data:
            return
        self._runtime_log(
            source="hand_tracking",
            level="INFO",
            category="communication",
            message="Hand tracking started signal received",
            details={
                "topic": "/hand_tracking_request",
                "data": True,
            },
        )

    def hand_arrived_callback(self, message: Bool) -> None:
        if not message.data:
            return
        self._runtime_log(
            source="hand_tracking",
            level="INFO",
            category="communication",
            message="Hand arrived signal received",
            details={
                "topic": "/hand_arrived",
                "data": True,
            },
        )

    def system_log_callback(self, message: String) -> None:
        payload = self._parse(message, source="system_log")
        if payload is None:
            return

        self._runtime_log(
            source=str(payload.get("source", "external_node")),
            level=str(payload.get("level", "INFO")),
            category=str(
                payload.get("category", "communication")
            ),
            message=str(payload.get("message", "")),
            details=(
                payload.get("details")
                if isinstance(
                    payload.get("details"),
                    Mapping,
                )
                else None
            ),
        )

    def rosout_callback(self, message: Log) -> None:
        node_name = str(message.name)
        message_text = str(message.msg)
        searchable = f"{node_name} {message_text}".lower()

        if self.rosout_filters and not any(
            keyword in searchable
            for keyword in self.rosout_filters
        ):
            return

        level_mapping = {
            10: "DEBUG",
            20: "INFO",
            30: "WARN",
            40: "ERROR",
            50: "FATAL",
        }
        level = level_mapping.get(
            int(message.level),
            str(message.level),
        )
        timestamp = (
            f"{int(message.stamp.sec)}."
            f"{int(message.stamp.nanosec):09d}"
        )

        try:
            append_runtime_log(
                source=node_name or "rosout",
                level=level,
                category="rosout",
                message=message_text,
                timestamp=timestamp,
                details={
                    "file": str(message.file),
                    "function": str(message.function),
                    "line": int(message.line),
                },
            )
        except OSError:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RedisObjectBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
