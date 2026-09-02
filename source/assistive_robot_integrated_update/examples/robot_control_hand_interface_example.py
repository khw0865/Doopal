#!/usr/bin/env python3
"""Robot Control 노드에 추가할 Hand Detection/Tracking 통신 예제.

실제 로봇 이동 함수는 프로젝트의 movel/posx 코드에 연결해야 한다.
"""

from __future__ import annotations

from typing import Callable, Optional

import rclpy
from hey_doopal_msg.action import FindOrder
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class RobotControlHandClient(Node):
    def __init__(self) -> None:
        super().__init__("robot_control_hand_client_example")

        self.find_hand_client = ActionClient(
            self,
            FindOrder,
            "/find_hand_order",
        )
        self.start_tracking_client = self.create_client(
            Trigger,
            "/start_hand_tracking",
        )

        # Robot Control도 최종 도착을 알아야 한다면 구독한다.
        self.create_subscription(
            Bool,
            "/hand_arrived",
            self.hand_arrived_callback,
            10,
        )

        self.last_hand_coordinate_mm: Optional[list[float]] = None

    def request_hand_coordinate(self) -> None:
        if not self.find_hand_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("/find_hand_order Action Server unavailable")
            return

        goal = FindOrder.Goal()
        goal.target_name = "hand"
        future = self.find_hand_client.send_goal_async(
            goal,
            feedback_callback=self.find_hand_feedback_callback,
        )
        future.add_done_callback(self.find_hand_goal_response_callback)

    def find_hand_feedback_callback(self, feedback_msg) -> None:
        self.get_logger().info(
            f"Hand detection feedback: {feedback_msg.feedback.state}"
        )

    def find_hand_goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Hand detection goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.find_hand_result_callback)

    def find_hand_result_callback(self, future) -> None:
        result = future.result().result
        if not result.found:
            self.get_logger().warning(result.message)
            return

        # 이미 base_link 기준 mm 단위다. 다시 1000을 곱하지 않는다.
        self.last_hand_coordinate_mm = [
            float(result.coordinate[0]),
            float(result.coordinate[1]),
            float(result.coordinate[2]),
        ]
        self.get_logger().info(
            f"Hand coordinate [mm]: {self.last_hand_coordinate_mm}"
        )

        # 이 지점에서 실제 로봇 제어 코드가 다음 순서를 수행한다.
        # 1) 물건 Grip
        # 2) 위 좌표 근처의 사전 접근 위치까지 movel
        # 3) 사전 접근 이동 완료 후 start_hand_tracking() 호출

    def start_hand_tracking(self) -> None:
        """로봇이 기존 손 좌표의 사전 접근 위치에 도착한 후 호출한다."""
        if not self.start_tracking_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/start_hand_tracking service unavailable")
            return

        request = Trigger.Request()
        future = self.start_tracking_client.call_async(request)
        future.add_done_callback(self.start_tracking_response_callback)

    def start_tracking_response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Start tracking call failed: {error}")
            return

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warning(response.message)

    def hand_arrived_callback(self, message: Bool) -> None:
        if not message.data:
            return
        self.get_logger().info("Hand tracking completed: /hand_arrived=True")
        # 필요 시 여기서 그리퍼를 열어 물건을 놓는 로직을 호출한다.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotControlHandClient()
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
