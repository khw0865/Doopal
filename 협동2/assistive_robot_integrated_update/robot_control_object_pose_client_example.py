#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import rclpy
from hey_doopal_msg.srv import GetObjectPose
from rclpy.node import Node


class RobotControlObjectPoseClient(Node):
    """Redis에서 객체의 [x,y,z,rx,ry,rz]를 조회하는 예제."""

    def __init__(self) -> None:
        super().__init__("robot_control_object_pose_client")
        self.object_pose_client = self.create_client(
            GetObjectPose,
            "/assistive/get_object_pose",
        )

    def request_object_pose(self, object_name: str) -> None:
        if not self.object_pose_client.service_is_ready():
            self.get_logger().warning(
                "/assistive/get_object_pose 서비스가 아직 준비되지 않았습니다"
            )
            return

        request = GetObjectPose.Request()
        request.object_name = object_name
        future = self.object_pose_client.call_async(request)
        future.add_done_callback(self._response_callback)

    def _response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"객체 pose 서비스 호출 실패: {error}")
            return

        if response is None or not response.success or not response.has_pose:
            message = response.message if response is not None else "응답 없음"
            self.get_logger().warning(f"객체 pose 조회 실패: {message}")
            return

        pose = [float(value) for value in response.pose]
        self.get_logger().info(
            f"수신 pose: {pose}, position={response.coordinate_unit}, "
            f"angle={response.angle_unit}, frame={response.frame_id}"
        )

        # Doosan 제어 예:
        # target_pose = posx(pose)
        # movel(target_pose, vel=..., acc=..., ref=DR_BASE)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotControlObjectPoseClient()
    try:
        if not node.object_pose_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(
                "/assistive/get_object_pose 서비스를 찾을 수 없습니다"
            )
            return
        node.request_object_pose("airpods")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
