# Assistive Robot Integrated Update — Object Pose 6D

객체 DB와 ROS 2 조회 서비스를 다음 6D pose 계약으로 통일한 버전입니다.

```text
pose = [object_x_mm, object_y_mm, object_z_mm, robot_rx_deg, robot_ry_deg, robot_rz_deg]
```

- 앞 3개 값: `base_link` 기준 객체 위치, mm
- 뒤 3개 값: 객체 인식 시점의 M0609 `link_6` 회전 자세, deg
- 객체 인식 노드는 Redis 레코드를 `replace=true`로 교체합니다.
- 구형 `GetObjectCoordinate.srv`와 `/assistive/get_object_coordinate`는 제거했습니다.

## 객체 pose 조회 서비스

```text
Service: /assistive/get_object_pose
Type: hey_doopal_msg/srv/GetObjectPose
```

요청:

```text
string object_name
```

응답 핵심:

```text
bool success
bool has_pose
float64[6] pose
string coordinate_unit   # mm
string angle_unit        # deg
string frame_id          # base_link
```

테스트:

```bash
ros2 service call \
  /assistive/get_object_pose \
  hey_doopal_msg/srv/GetObjectPose \
  "{object_name: 'airpods'}"
```

정상 응답 예:

```text
success: true
has_pose: true
pose: [565.93, 50.48, 36.21, 179.98, 0.12, 89.74]
coordinate_unit: mm
angle_unit: deg
frame_id: base_link
message: 객체 pose 조회 성공
```

## 인터페이스 빌드

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
colcon build --packages-select hey_doopal_msg --symlink-install
source install/setup.bash
ros2 interface show hey_doopal_msg/srv/GetObjectPose
```

## 유지하는 서비스

- `GetObjectPose.srv`: Redis 객체 6D pose 조회
- `GetDbData.srv`: 범용 JSON 조회
- `GetFixedPose.srv`: 고정 로봇 pose 조회
- `GetScanCase.srv`: CASE별 스캔 pose 조회
- `ScanRequest.srv`: YOLO 전체 스캔 요청
- `GripBoundingBox.srv`: 현재 영상에서 그립 대상 재확인
- `VoiceKeyword.srv`: 음성 키워드 서비스

`GripBoundingBox.srv`의 `[x,y,z]`는 Redis 조회 서비스가 아니라, 현재 카메라에서 그립 대상을 즉시 재확인하기 위한 결과이므로 유지합니다.

## 중요

기존 Redis에 `x/y/z`만 저장된 객체는 새 서비스에서 거부됩니다. 객체 인식 노드로 다시 스캔해 `pose[6]` 형식으로 갱신해야 합니다.
