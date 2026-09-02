#!/usr/bin/env python3
from __future__ import annotations

import sys
from typing import Optional

import rclpy
from hey_doopal_msg.srv import GetFixedPose, GetScanCase
from rclpy.node import Node


class RobotControlWaypointCaseClient(Node):
    """로봇 컨트롤 노드에 통합할 수 있는 서비스 클라이언트 예제."""

    def __init__(self) -> None:
        super().__init__("robot_control_waypoint_case_client")

        self.fixed_pose_client = self.create_client(
            GetFixedPose,
            "/assistive/get_fixed_pose",
        )
        self.scan_case_client = self.create_client(
            GetScanCase,
            "/assistive/get_scan_case",
        )

    def request_fixed_pose(self, pose_name: str) -> None:
        request = GetFixedPose.Request()
        request.pose_name = pose_name

        future = self.fixed_pose_client.call_async(request)
        future.add_done_callback(self._fixed_pose_done)

    def _fixed_pose_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"웨이포인트 서비스 호출 실패: {error}"
            )
            return

        if response is None or not response.success:
            message = (
                response.message
                if response is not None
                else "응답 없음"
            )
            self.get_logger().warning(message)
            return

        pose = [
            float(value)
            for value in response.pose
        ]

        self.get_logger().info(
            f"웨이포인트: {pose} "
            f"[{response.coordinate_unit}, "
            f"{response.angle_unit}]"
        )

        # Doosan:
        # target = posx(pose)
        # movel(target, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    def request_scan_case(self, case_name: str) -> None:
        request = GetScanCase.Request()
        request.case_name = case_name

        future = self.scan_case_client.call_async(request)
        future.add_done_callback(self._scan_case_done)

    def _scan_case_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"CASE 서비스 호출 실패: {error}"
            )
            return

        if response is None or not response.success:
            message = (
                response.message
                if response is not None
                else "응답 없음"
            )
            self.get_logger().warning(message)
            return

        first_pose = [
            float(value)
            for value in response.first_pose
        ]
        second_pose = [
            float(value)
            for value in response.second_pose
        ]

        # DB에 저장된 CASE 순서를 그대로 유지한다.
        case_poses = [
            first_pose,
            second_pose,
        ]

        self.get_logger().info(
            f"{response.first_pose_name}: {first_pose}"
        )
        self.get_logger().info(
            f"{response.second_pose_name}: {second_pose}"
        )
        self.get_logger().info(
            f"CASE 실행 리스트: {case_poses}"
        )

        # Doosan:
        # for pose in case_poses:
        #     movel(
        #         posx(pose),
        #         vel=VELOCITY,
        #         acc=ACC,
        #         ref=DR_BASE,
        #     )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotControlWaypointCaseClient()

    try:
        mode = sys.argv[1] if len(sys.argv) >= 2 else "case"
        name = sys.argv[2] if len(sys.argv) >= 3 else "CASE_1"

        if mode == "pose":
            if not node.fixed_pose_client.wait_for_service(
                timeout_sec=5.0
            ):
                node.get_logger().error(
                    "/assistive/get_fixed_pose 서비스를 찾을 수 없습니다"
                )
                return
            node.request_fixed_pose(name)
        else:
            if not node.scan_case_client.wait_for_service(
                timeout_sec=5.0
            ):
                node.get_logger().error(
                    "/assistive/get_scan_case 서비스를 찾을 수 없습니다"
                )
                return
            node.request_scan_case(name)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
