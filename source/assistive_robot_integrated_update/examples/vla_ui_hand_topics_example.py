#!/usr/bin/env python3
"""VLA에 추가할 UI 채팅 및 Hand Tracking 상태 통신 예제."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class VlaRosInterface(Node):
    def __init__(self) -> None:
        super().__init__("vla_ros_interface_example")

        chat_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.chat_publisher = self.create_publisher(
            String,
            "/ui_chat_log",
            chat_qos,
        )
        self.state_publisher = self.create_publisher(
            String,
            "/assistive/vla_state",
            10,
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

    def publish_chat(self, speaker: str, text: str) -> None:
        speaker = speaker.strip().upper()
        if speaker not in {"USER", "ASSISTANT"}:
            raise ValueError("speaker must be USER or ASSISTANT")

        message = String()
        message.data = json.dumps(
            {
                "speaker": speaker,
                "text": text,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.chat_publisher.publish(message)

    def publish_state(self, state: str, message_text: str) -> None:
        # user_text/assistant_text는 넣지 않는다.
        # 대화는 반드시 /ui_chat_log로만 보낸다.
        message = String()
        message.data = json.dumps(
            {
                "state": state,
                "message": message_text,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.state_publisher.publish(message)

    def hand_tracking_started_callback(self, message: Bool) -> None:
        if message.data:
            self.get_logger().info("Hand tracking started")
            self.publish_state("working", "손을 따라 이동하고 있어요.")

    def hand_arrived_callback(self, message: Bool) -> None:
        if message.data:
            self.get_logger().info("Hand tracking completed")
            self.publish_state("speaking", "손 위치에 도착했어요.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlaRosInterface()
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
