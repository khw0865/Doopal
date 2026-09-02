# assistive_detection

시각장애인 보조 로봇 프로젝트의 인식·추적 기능을 ROS2 Humble용 `ament_python` 패키지로 구성한 제출본입니다.

## 포함 노드

| 실행 파일 | 역할 |
|---|---|
| `hand_detection` | MediaPipe + RealSense 기반 손 검출 및 3D 좌표 Action Server |
| `hand_tracking` | 영상 오차와 Depth를 이용한 SpeedL 손 추종 서비스 노드 |
| `object_detection` | YOLO 객체 인식, 3D 좌표 변환, 서비스·액션 제공 |

## 포함 데이터

- `assistive_detection/T_gripper2camera.npy`: Hand-Eye 캘리브레이션 행렬
- `assistive_detection/my_seg_best.pt`: 학습된 YOLO 모델

두 파일은 파이썬 패키지 내부에 설치되며, 각 노드는 `Path(__file__).resolve().parent`를 기준으로 자동 탐색합니다.

## 1. 제출 패키지 배치

```bash
mkdir -p ~/ws_cobot_pjt/ws_dsr/src
cd ~/ws_cobot_pjt/ws_dsr/src
unzip assistive_detection_submission.zip
```

압축을 풀었을 때 다음 경로가 만들어져야 합니다.

```text
~/ws_cobot_pjt/ws_dsr/src/assistive_detection_submission/package.xml
```

원한다면 폴더명을 `assistive_detection`으로 변경해도 됩니다. ROS2 패키지명은 `package.xml`에 정의된 `assistive_detection`입니다.

## 2. 파이썬 의존성 설치

```bash
python3 -m pip install -r ~/ws_cobot_pjt/ws_dsr/src/assistive_detection_submission/requirements.txt
```

ROS 의존성은 워크스페이스 루트에서 설치합니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr
rosdep install --from-paths src --ignore-src -r -y
```

`hey_doopal_msg`와 `dsr_msgs2`가 같은 워크스페이스에 있거나, 이미 빌드·source 된 환경이어야 합니다.

## 3. 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/ws_cobot_pjt/ws_dsr
colcon build --symlink-install --packages-select assistive_detection
source install/setup.bash
```

## 4. 개별 실행

```bash
ros2 run assistive_detection hand_detection
ros2 run assistive_detection hand_tracking
ros2 run assistive_detection object_detection
```

## 5. 전체 실행

RealSense와 두산 로봇 bringup을 먼저 실행한 뒤 다음 명령을 사용합니다.

```bash
ros2 launch assistive_detection detection.launch.py
```

## 6. 주요 선행 조건

- RealSense RGB, aligned depth, camera info 토픽이 발행 중이어야 합니다.
- `base_link -> link_6` TF를 조회할 수 있어야 합니다.
- `/dsr01/speedl_stream`을 처리하는 두산 로봇 드라이버가 실행 중이어야 합니다.
- `hey_doopal_msg`에 `FindOrder.action`, `ScanRequest.srv`, `GripBoundingBox.srv`가 존재해야 합니다.

## 7. 제출 전 확인

```bash
ros2 pkg executables assistive_detection
ros2 pkg prefix assistive_detection
```

정상이라면 세 실행 파일이 출력됩니다.

```text
assistive_detection hand_detection
assistive_detection hand_tracking
assistive_detection object_detection
```
