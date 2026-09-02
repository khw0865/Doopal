#!/usr/bin/env python3
"""Robot Control에서 관리자 UI 런타임 로그를 명시적으로 남기는 예제.

이 로그는 Redis에 저장되지 않고 UI 프로젝트의 runtime_logs.jsonl에만 기록된다.
"""

import json

from std_msgs.msg import String


def create_system_log_publisher(node):
    return node.create_publisher(String, "/assistive/system_log", 50)


def publish_system_log(
    publisher,
    *,
    source: str,
    level: str,
    message_text: str,
    category: str = "communication",
    details: dict | None = None,
) -> None:
    message = String()
    message.data = json.dumps(
        {
            "source": source,
            "level": level,
            "category": category,
            "message": message_text,
            "details": details or {},
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )
    publisher.publish(message)
