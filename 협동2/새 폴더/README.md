# YOLO best.pt + RealSense 3D 좌표

## 기능

- YOLO `best.pt`로 객체 검출
- 바운딩 박스 중앙 영역의 aligned depth 중앙값 계산
- 카메라 내부 파라미터로 Camera XYZ 계산
- TF로 `camera_color_optical_frame -> base_link` 변환
- 선택 객체와 전체 객체 좌표를 ROS 2 토픽으로 발행

## 복사

```bash
cp yolo_realsense_3d.py \
  ~/ws_cobot_pjt/ws_dsr/src/rokey/rokey/

cp yolo_realsense_3d_params.yaml \
  ~/ws_cobot_pjt/ws_dsr/src/rokey/config/
```

`setup.py`의 `console_scripts`에 추가:

```python
"yolo_3d = rokey.yolo_realsense_3d:main",
```

## 의존성

```bash
pip install ultralytics

sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-tf2-geometry-msgs
```

`package.xml`에 필요 시 추가:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
<exec_depend>message_filters</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_geometry_msgs</exec_depend>
```

## 빌드

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
colcon build --packages-select rokey --symlink-install
source install/setup.bash
```

## RealSense 실행

카메라 TF가 통합 URDF에서 발행되는 경우:

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  publish_tf:=false
```

카메라 내부 TF를 RealSense가 발행해야 하는 경우:

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  publish_tf:=true
```

## 실행

```bash
ros2 run rokey yolo_3d \
  --ros-args \
  --params-file \
  "$(ros2 pkg prefix rokey)/share/rokey/config/yolo_realsense_3d_params.yaml" \
  -p model_path:=/absolute/path/to/best.pt
```

특정 클래스만 사용:

```bash
ros2 run rokey yolo_3d \
  --ros-args \
  --params-file \
  "$(ros2 pkg prefix rokey)/share/rokey/config/yolo_realsense_3d_params.yaml" \
  -p model_path:=/absolute/path/to/best.pt \
  -p target_class:=toothbrush
```

## 출력 토픽

카메라 optical frame 기준 선택 객체 좌표:

```bash
ros2 topic echo /yolo_object_3d/camera_point
```

`base_link` 기준 선택 객체 좌표:

```bash
ros2 topic echo /yolo_object_3d/base_point
```

모든 검출 결과 JSON:

```bash
ros2 topic echo /yolo_object_3d/detections
```

PointStamped 단위는 m이고 OpenCV 화면에는 mm로 표시된다.
