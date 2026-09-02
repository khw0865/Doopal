# 시각장애인 보조 로봇 통합 수정본

## 1. 최종 통신 구조

### VLA → UI/DB Bridge

- `/ui_chat_log`
  - 타입: `std_msgs/msg/String`
  - QoS: RELIABLE / VOLATILE / depth 10
  - JSON:

```json
{"speaker":"USER","text":"에어팟 손에 줘"}
```

또는:

```json
{"speaker":"ASSISTANT","text":"에어팟을 가져다드릴게요."}
```

- `/assistive/vla_state`
  - 타입: `std_msgs/msg/String`
  - 상태 전용 JSON:

```json
{"state":"listening","message":"말씀을 듣고 있어요."}
```

`/assistive/vla_state`에는 `user_text`, `assistant_text`를 넣지 않습니다. 대화는 `/ui_chat_log`로만 보냅니다.

### Robot Control → Hand Detection

- Action 이름: `/find_hand_order`
- 타입: `hey_doopal_msg/action/FindOrder`
- 요청: `target_name="hand"`
- 성공 결과:
  - `found=true`
  - `coordinate=[x_mm,y_mm,z_mm]`
  - `message="hand detected"`

### Robot Control → Hand Tracking

- Service 이름: `/start_hand_tracking`
- 타입: `std_srvs/srv/Trigger`
- 로봇이 물건을 집고 기존 손 좌표의 사전 접근 위치에 도착한 뒤 호출합니다.

### Hand Tracking → VLA

- `/hand_tracking_request`, `std_msgs/msg/Bool`
  - Trigger 서비스를 수락하고 추적을 시작할 때 `True` 1회 발행
- `/hand_arrived`, `std_msgs/msg/Bool`
  - TCP가 손바닥 목표에 안정적으로 도착했을 때 `True` 1회 발행

## 2. 좌표 기준

- 객체 좌표: `base_link` 기준 mm
- Hand Detection Action 결과: `base_link` 기준 mm
- Hand Tracking 내부 손바닥/TCP/목표 좌표: mm
- SpeedL 선속도: mm/s
- `T_gripper2camera.npy`는 이름 그대로 `gripper -> camera` 변환으로 해석합니다.
- 카메라 점을 gripper 좌표로 변환할 때 두 손 노드 모두 역행렬을 사용합니다.
- 두 손 노드의 기본 calibration frame은 `gripper_tcp`입니다.

필수 TF:

```text
base_link -> gripper_tcp
```

`gripper_tcp`가 `link_6`의 static child라면 해당 static TF도 먼저 발행해야 합니다.

## 3. 파일 배치

ROS Python 패키지가 `rokey`라고 가정합니다.

```bash
cp hand_nodes/mediapipe_palm_3d_action_server.py \
  ~/ws_cobot_pjt/ws_dsr/src/rokey/rokey/

cp hand_nodes/realsense_hand_speedl_follow_service.py \
  ~/ws_cobot_pjt/ws_dsr/src/rokey/rokey/

cp hand_nodes/T_gripper2camera.npy \
  ~/ws_cobot_pjt/ws_dsr/src/rokey/rokey/
```

객체 인식 노드도 mm 버전으로 교체할 경우:

```bash
cp object_detection_node_bridge_mm.py \
  ~/ws_cobot_pjt/ws_dsr/src/rokey/rokey/object_detection_node_bridge.py
```

`my_seg_best.pt`도 해당 Python 파일과 같은 패키지 폴더에 둡니다.

## 4. rokey/setup.py

`console_scripts`에 추가합니다.

```python
entry_points={
    'console_scripts': [
        'hand_detection = rokey.mediapipe_palm_3d_action_server:main',
        'hand_tracking = rokey.realsense_hand_speedl_follow_service:main',
        'obj_dect = rokey.object_detection_node_bridge:main',
    ],
},
```

NPY를 install 공간에도 포함하려면 `setup()`에 다음을 넣습니다.

```python
package_data={
    'rokey': ['*.npy'],
},
include_package_data=True,
```

## 5. rokey/package.xml

최소 의존성:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>std_srvs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>cv_bridge</depend>
<depend>tf2_ros</depend>
<depend>message_filters</depend>
<depend>dsr_msgs2</depend>
<depend>hey_doopal_msg</depend>
```

`mediapipe`, `opencv-python`, `numpy`, `ultralytics`는 Python 환경에 설치되어 있어야 합니다.

## 6. Interface 패키지 교체 및 빌드

```bash
rm -rf ~/ws_cobot_pjt/ws_dsr/src/hey_doopal_msg
cp -r hey_doopal_msg ~/ws_cobot_pjt/ws_dsr/src/

cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
colcon build --packages-select hey_doopal_msg rokey --symlink-install
source install/setup.bash
```

확인:

```bash
ros2 interface show hey_doopal_msg/action/FindOrder
ros2 interface show hey_doopal_msg/srv/GetDbData
```

`GripBoundingBox.srv`의 필드 불일치는 요청대로 수정하지 않았습니다. 좌표 단위 주석만 mm로 맞췄습니다.

## 7. UI/DB 프로젝트

기존 `.env`는 새 ZIP에 포함하지 않았습니다. 기존 `.env`를 `ui_db/.env`로 그대로 복사합니다.

```bash
cd ui_db
cp /기존/UI/프로젝트/.env .env
```

Redis:

```bash
docker compose up -d
```

Bridge:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
cd ui_db
source .venv/bin/activate
python3 ros_object_bridge.py
```

Flask UI:

```bash
cd ui_db
source .venv/bin/activate
python3 app.py
```

관리자 페이지에는 다음 두 로그가 분리되어 표시됩니다.

- VLA 대화 기록: Redis 저장
- M0609/ROS/통신 런타임 로그: `runtime_logs.jsonl` 파일 저장, Redis 미사용

M0609와 Robot Control 로그는 `/rosout`에서 필터링합니다. 별도 통신 로그를 명시적으로 남기려면 `/assistive/system_log`에 JSON String을 발행합니다.

## 8. 실행과 테스트

### Hand Detection 실행

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
ros2 run rokey hand_detection
```

Action 테스트:

```bash
ros2 action send_goal \
  /find_hand_order \
  hey_doopal_msg/action/FindOrder \
  "{target_name: 'hand'}" \
  --feedback
```

성공 예:

```text
found: true
coordinate: [445.2, -23.5, 533.6]
message: hand detected
```

### Hand Tracking 실행

```bash
ros2 run rokey hand_tracking
```

추적 시작:

```bash
ros2 service call \
  /start_hand_tracking \
  std_srvs/srv/Trigger \
  "{}"
```

VLA용 신호 확인:

```bash
ros2 topic echo /hand_tracking_request
ros2 topic echo /hand_arrived
```

### VLA 채팅 로그 테스트

```bash
ros2 topic pub --once \
  /ui_chat_log \
  std_msgs/msg/String \
  "{data: '{\"speaker\":\"USER\",\"text\":\"에어팟 손에 줘\"}'"
```

### VLA 상태 테스트

```bash
ros2 topic pub --once \
  /assistive/vla_state \
  std_msgs/msg/String \
  "{data: '{\"state\":\"listening\",\"message\":\"말씀을 듣고 있어요.\"}'"
```

## 9. 객체 DB pose[6] 형식 및 기존 데이터 주의

최신 Object Detection 노드는 객체 이름별 Redis 레코드를 다음 형식으로 전체 교체합니다.

```text
pose = [object_x_mm, object_y_mm, object_z_mm, robot_rx_deg, robot_ry_deg, robot_rz_deg]
```

- 앞의 세 값: `base_link` 기준 객체 좌표
- 뒤의 세 값: 객체를 인식한 동일 시점의 M0609 `link_6` 회전 자세
- `replace=True`이므로 해당 객체가 다시 인식되면 과거 `x/y/z/saved_at` 필드는 제거됩니다.
- 아직 다시 인식되지 않은 구형 객체 레코드는 Redis에 남을 수 있으므로 최초 적용 시 기존 객체 데이터를 삭제한 뒤 다시 스캔하는 것이 안전합니다.

## 10. 직접 손바닥 접근 안전 주의

수정된 Hand Tracking 목표는 손바닥 좌표 자체입니다. 다만 카메라 오차, TF 오차, 손의 갑작스러운 움직임 때문에 실제 사람에게 충돌할 위험이 있습니다.

기본값은 다음처럼 보수적으로 설정했습니다.

- 최대 선속도: 45 mm/s
- 도착 허용 오차: 10 mm
- 연속 도착 판정: 6 control cycles

실기 전에는 낮은 속도, 넓은 작업공간 여유, 비상정지 준비 상태에서 시험해야 합니다. 실제 접촉 안정성을 높이려면 최종 단계에 힘/토크 임계값 정지를 추가하는 것이 필요합니다.
