# 서비스 정리 내역

## 제거

```text
GetObjectCoordinate.srv
/assistive/get_object_coordinate
robot_control_client_example.py
```

이 인터페이스는 객체의 `[x,y,z]`만 반환했기 때문에 현재 `pose[6]` DB 계약과 맞지 않아 제거했습니다.

## 추가

```text
GetObjectPose.srv
/assistive/get_object_pose
robot_control_object_pose_client_example.py
```

응답 pose:

```text
[object_x_mm, object_y_mm, object_z_mm, robot_rx_deg, robot_ry_deg, robot_rz_deg]
```

## 유지 이유

- `GripBoundingBox.srv`: DB 조회가 아니라 실시간 그립 대상 확인용
- `ScanRequest.srv`: 전체 스캔 트리거용
- `GetDbData.srv`: 관리자/UI 범용 JSON 조회용
- `GetFixedPose.srv`: 사전 등록된 로봇 pose 조회용
- `GetScanCase.srv`: 스캔 경로 pose 묶음 조회용
