# Assistive Robot UI — 자유 객체 데이터 + 고정 스캔 CASE 버전

## 이번 변경 사항

### 1. 객체 데이터는 완전한 자유 JSON

Redis 객체 키는 다음 형식만 유지합니다.

```text
assistive_robot:object:<record_name>
```

`record_name`은 객체 레코드를 구분하기 위한 이름이고, Hash 내부 필드는 객체마다 완전히 달라도 됩니다.
`x`, `y`, `z`, `confidence`, `visible` 같은 공통 필드를 더 이상 강제하지 않습니다.

예를 들어 컵은 다음처럼 저장할 수 있습니다.

```json
{
  "class_name": "cup",
  "coordinate": [420.5, -135.2, 85.0],
  "confidence": 0.91,
  "material": "plastic",
  "grasp": {
    "type": "side",
    "width_mm": 60
  }
}
```

칫솔은 전혀 다른 구조를 사용할 수 있습니다.

```json
{
  "label": "toothbrush",
  "pose": {
    "frame": "base_link",
    "x": 510.2,
    "y": 80.4,
    "z": 42.0
  },
  "color": "blue",
  "hygiene_status": "clean"
}
```

관리자 화면의 객체 목록도 정형 테이블이 아니라 객체별 JSON 카드로 표시됩니다.
수동 입력은 `객체 키 이름`과 `자유 JSON 데이터`만 사용합니다.

### 2. 새 고정 웨이포인트

```python
SCAN_WAYPOINT1 = [434.70,   21.07, 552.72,  63.44, -179.21,  62.54]
SCAN_WAYPOINT2 = [434.70, -187.14, 552.72,  63.44, -179.21,  62.54]
SCAN_WAYPOINT3 = [431.95, -392.07, 419.67, 147.77,  180.00, -33.61]
HAND_SCAN      = [445.29,  -23.52, 533.56,  90.00,  -90.00, -90.00]
```

필드 순서는 다음과 같습니다.

```text
[x, y, z, rx, ry, rz]
```

Redis 키:

```text
assistive_robot:fixed_point:SCAN_WAYPOINT1
assistive_robot:fixed_point:SCAN_WAYPOINT2
assistive_robot:fixed_point:SCAN_WAYPOINT3
assistive_robot:fixed_point:HAND_SCAN
```

### 3. 고정 이동 CASE

CASE는 좌표값을 중복 저장하지 않고, 이미 저장된 웨이포인트 이름을 이동 순서대로 담는 Redis List입니다.

```python
CASE_1 = ["SCAN_WAYPOINT1", "SCAN_WAYPOINT2"]
CASE_2 = ["SCAN_WAYPOINT1", "SCAN_WAYPOINT2", "SCAN_WAYPOINT3"]
```

Redis 키와 자료형:

```text
assistive_robot:scan_case:CASE_1    # Redis List
assistive_robot:scan_case:CASE_2    # Redis List
```

터미널 확인:

```bash
REDISCLI_AUTH="$REDIS_PASSWORD" redis-cli LRANGE assistive_robot:scan_case:CASE_1 0 -1
```

출력:

```text
1) "SCAN_WAYPOINT1"
2) "SCAN_WAYPOINT2"
```

## 기존 고정 데이터 교체 방식

고정 설정 버전 키:

```text
assistive_robot:fixed_config:version
```

새 코드가 최초 실행될 때 이전 버전이 감지되면 다음 작업을 자동 수행합니다.

1. 기존 `HAND_SCAN`, `TARGET_SCAN`을 포함한 이전 고정 좌표 키 삭제
2. 이전 스캔 CASE 키와 인덱스 삭제
3. 새 웨이포인트 4개 생성
4. `CASE_1`, `CASE_2` Redis List 생성
5. 새 설정 버전 저장

객체 데이터와 대화 기록은 삭제하지 않습니다.

마이그레이션은 다음 중 하나를 실행할 때 수행됩니다.

```bash
python3 seed_demo.py
```

또는:

```bash
python3 app.py
```

또는:

```bash
python3 ros_object_bridge.py
```

## 설치 및 실행

```bash
cd assistive_robot_ui_redis_fixed_points
cp .env.example .env
nano .env
```

최소 변경 항목:

```dotenv
REDIS_PASSWORD=충분히_긴_Redis_비밀번호
FLASK_SECRET_KEY=충분히_긴_랜덤_문자열
ADMIN_PASSWORD=관리자_UI_로그인_비밀번호
```

Redis 실행:

```bash
docker compose up -d
```

Python 환경:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

고정 설정 마이그레이션/초기화:

```bash
python3 seed_demo.py
```

UI 실행:

```bash
python3 app.py
```

접속:

```text
사용자 UI: http://127.0.0.1:5000/
관리자 UI: http://127.0.0.1:5000/admin/login
```

## 관리자 API로 자유 객체 저장

```bash
curl -X POST http://127.0.0.1:5000/api/admin/objects \
  -H 'Content-Type: application/json' \
  -b '<관리자 로그인 세션 쿠키>' \
  -d '{
    "record_name": "cup",
    "replace": true,
    "data": {
      "coordinate": [420.5, -135.2, 85.0],
      "material": "plastic",
      "grasp": {"type": "side", "width_mm": 60}
    }
  }'
```

관리자 UI에서는 로그인 세션을 자동 사용하므로 별도의 쿠키 입력 없이 폼으로 저장하면 됩니다.

## ROS 2 브리지로 자유 객체 저장

브리지 실행:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 ros_object_bridge.py
```

새 권장 메시지 형식:

```bash
ros2 topic pub --once /assistive/object_detection std_msgs/msg/String \
"{data: '{\"record_name\":\"cup\",\"replace\":false,\"data\":{\"coordinate\":[420.5,-135.2,85.0],\"confidence\":0.91,\"material\":\"plastic\",\"grasp\":{\"type\":\"side\",\"width_mm\":60}}}'}"
```

- `replace: false`: 기존 필드와 병합
- `replace: true`: 기존 Hash를 삭제하고 전달한 JSON으로 전체 교체

이전 메시지 형식도 지원합니다.

```json
{
  "class_name": "cup",
  "position": {"x_mm": 420.5, "y_mm": -135.2, "z_mm": 85.0},
  "confidence": 0.91,
  "attributes": {"material": "plastic"}
}
```

이 경우 `class_name`을 레코드 이름으로 사용하고 전체 payload를 자유 데이터로 저장합니다.

## Python에서 CASE 순서대로 로봇 이동

```python
from redis_store import RedisStore

store = RedisStore()
store.initialize_fixed_data()

case_1_poses = store.get_scan_case_poses("CASE_1")

# 반환값:
# [
#   [434.70, 21.07, 552.72, 63.44, -179.21, 62.54],
#   [434.70, -187.14, 552.72, 63.44, -179.21, 62.54],
# ]

for pose in case_1_poses:
    robot_pose = posx(pose)
    movel(robot_pose, vel=100, acc=100)
```

웨이포인트 이름까지 필요하면 다음을 사용합니다.

```python
case = store.get_scan_case("CASE_2")

for waypoint in case["waypoints"]:
    print(waypoint["order"], waypoint["name"], waypoint["pose"])
```

## Redis에서 직접 확인

```bash
set -a
source .env
set +a
export REDISCLI_AUTH="$REDIS_PASSWORD"

redis-cli SMEMBERS assistive_robot:fixed_points:index
redis-cli HGETALL assistive_robot:fixed_point:SCAN_WAYPOINT1
redis-cli LRANGE assistive_robot:scan_case:CASE_1 0 -1
redis-cli LRANGE assistive_robot:scan_case:CASE_2 0 -1
redis-cli SMEMBERS assistive_robot:objects:index

unset REDISCLI_AUTH
```

## 주의

고정 웨이포인트와 CASE는 Flask 관리자 API에 수정·삭제 API가 없으므로 UI에서는 읽기 전용입니다.
Redis CLI로 직접 값을 변경하는 것은 가능하지만, 다음 실행에서 구성 누락이나 CASE 순서 손상이 감지되면 코드에 정의된 고정 설정으로 다시 생성됩니다.
