# Redis Bridge: DB 좌표를 float64[3] 리스트로 전달

## 통신 구조

```text
Robot Control Node
    │ GetObjectCoordinate 요청: object_name="cup"
    ▼
Redis Bridge
    │ Redis 객체 레코드 조회
    │ JSON/dict에서 x,y,z 추출
    │ m이면 mm로 변환
    ▼
GetObjectCoordinate 응답
    coordinate: [x, y, z]   # float64[3], mm
```

기존 `/assistive/get_db_data` 서비스는 그대로 유지됩니다.

새 서비스:

```text
/assistive/get_object_coordinate
hey_doopal_msg/srv/GetObjectCoordinate
```

## 1. 서비스 파일 복사

```bash
cp GetObjectCoordinate.srv \
  ~/ws_cobot_pjt/ws_dsr/src/hey_doopal_msg/srv/
```

## 2. hey_doopal_msg/CMakeLists.txt

`rosidl_generate_interfaces()`에 추가합니다.

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDbData.srv"
  "srv/GetObjectCoordinate.srv"
  # 기존 action/srv 파일들...
)
```

## 3. Bridge 파일 교체

UI/DB 프로젝트의 기존 bridge가 `ros_object_bridge.py`라면:

```bash
cp ros_object_bridge_coordinate_list.py \
  /UI_DB_프로젝트_경로/ros_object_bridge.py
```

`redis_store.py`, `runtime_log.py`는 기존 파일을 그대로 사용합니다.

## 4. 빌드

```bash
cd ~/ws_cobot_pjt/ws_dsr

source /opt/ros/humble/setup.bash

rm -rf \
  build/hey_doopal_msg \
  install/hey_doopal_msg

colcon build \
  --packages-select hey_doopal_msg \
  --symlink-install

source install/setup.bash
```

인터페이스 확인:

```bash
ros2 interface show \
  hey_doopal_msg/srv/GetObjectCoordinate
```

## 5. Bridge 실행

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash

cd /UI_DB_프로젝트_경로
source .venv/bin/activate

python3 ros_object_bridge.py
```

## 6. 터미널 테스트

```bash
ros2 service call \
  /assistive/get_object_coordinate \
  hey_doopal_msg/srv/GetObjectCoordinate \
  "{object_name: 'cup'}"
```

정상 예:

```text
success: true
has_coordinate: true
coordinate:
- 445.2
- -123.5
- 332.7
coordinate_unit: mm
frame_id: base_link
message: 객체 좌표 조회 성공
```

## 7. 로봇 컨트롤 노드

클라이언트에서:

```python
coordinate_mm = [
    float(value)
    for value in response.coordinate
]
```

로 받으면 결과는 바로:

```python
[445.2, -123.5, 332.7]
```

형태입니다.

## 단위 안전

Bridge는 DB 레코드의 `coordinate_unit`을 검사합니다.

- `coordinate_unit: "mm"`: 그대로 전송
- `coordinate_unit: "m"`: 1000을 곱해 mm로 변환
- 단위가 없음: 좌표 1000배 오류 방지를 위해 실패 처리

현재 객체 검출 노드는 반드시 다음 필드를 저장해야 합니다.

```json
{
  "coordinate_unit": "mm",
  "frame_id": "base_link",
  "x": 445.2,
  "y": -123.5,
  "z": 332.7
}
```
