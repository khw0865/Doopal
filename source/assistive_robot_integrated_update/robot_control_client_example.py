#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import rclpy
from hey_doopal_msg.srv import GetObjectCoordinate
from rclpy.node import Node


class RobotControlCoordinateClient(Node):
    """로봇 컨트롤 노드에 옮겨 사용할 수 있는 좌표 조회 예제."""

    def __init__(self) -> None:
        super().__init__("robot_control_coordinate_client")

        self.coordinate_client = self.create_client(
            GetObjectCoordinate,
            "/assistive/get_object_coordinate",
        )

    def request_coordinate(self, object_name: str) -> None:
        if not self.coordinate_client.service_is_ready():
            self.get_logger().warning(
                "/assistive/get_object_coordinate 서비스가 "
                "아직 준비되지 않았습니다"
            )
            return

        request = GetObjectCoordinate.Request()
        request.object_name = object_name

        future = self.coordinate_client.call_async(request)
        future.add_done_callback(
            self._coordinate_response_callback
        )

    def _coordinate_response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"좌표 서비스 호출 실패: {error}"
            )
            return

        if (
            response is None
            or not response.success
            or not response.has_coordinate
        ):
            message = (
                response.message
                if response is not None
                else "응답 없음"
            )
            self.get_logger().warning(
                f"객체 좌표 조회 실패: {message}"
            )
            return

        # ROS float64[3]를 명시적으로 Python list로 변환
        coordinate_mm = [
            float(value)
            for value in response.coordinate
        ]

        self.get_logger().info(
            f"수신 좌표: {coordinate_mm} "
            f"{response.coordinate_unit}, "
            f"frame={response.frame_id}"
        )

        # Doosan 제어에서 사용 예:
        #
        # rx, ry, rz = current_orientation
        # target_pose = posx(
        #     coordinate_mm
        #     + [rx, ry, rz]
        # )
        # movel(target_pose, vel=..., acc=..., ref=DR_BASE)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotControlCoordinateClient()

    try:
        if not node.coordinate_client.wait_for_service(
            timeout_sec=5.0
        ):
            node.get_logger().error(
                "/assistive/get_object_coordinate 서비스를 "
                "찾을 수 없습니다"
            )
            return

        node.request_coordinate("cup")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
