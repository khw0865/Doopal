import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy # [ADD] QoS 임포트

from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from std_msgs.msg import String
from hey_doopal_msg.action import FindOrder
from hey_doopal_msg.srv import ScanRequest
from hey_doopal_msg.srv import VoiceKeyword
from hey_doopal_msg.srv import GripBoundingBox
from hey_doopal_msg.srv import GetFixedPose
from hey_doopal_msg.srv import GetScanCase

from robot_control.cone_scan import ConeScanner
from rclpy.executors import MultiThreadedExecutor

import DR_init

# =========================
# Gripper Configuration
# =========================

GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = 502

# =========================
# Robot Configuration
# =========================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY = 500
ACCELERATION = 60

# =========================
# Scan Configuration
# =========================

SCAN_TILT_ANGLE = 20.0
SCAN_POINT_COUNT = 8

SCAN_VELOCITY = [500, 400]
SCAN_ACCELERATION = [60, 50]

# YOLO 서비스가 나타날 때까지 기다리는 시간
YOLO_SERVICE_WAIT_TIMEOUT = 10.0

# YOLO 한 번의 스캔 응답을 기다리는 시간
YOLO_SCAN_RESPONSE_TIMEOUT = 30.0

# 1. DSR 정보 등록
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 2. ROS 초기화
rclpy.init()

# 3. DSR_ROBOT2가 사용할 노드 생성
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        movel,
        movej,
        get_current_posx,
        get_current_posj,
        task_compliance_ctrl,
        set_desired_force,
        get_tool_force,
        release_force,
        release_compliance_ctrl,
        set_ref_coord,
        DR_FC_MOD_REL,
        DR_BASE,
        )
    
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# ##############################################################################
# [ADD] GRIPPER CONTROL CLASS START
# ##############################################################################
class AdaptiveGripper:
    def __init__(self, ip, port=502):
        self.client = ModbusClient(ip, port=port, stopbits=1, bytesize=8, parity="E", baudrate=115200, timeout=1)
        self.client.connect()
        self.max_width = 1100

    def move_gripper(self, width_val, force_val):
        params = [force_val, width_val, 16] 
        self.client.write_registers(address=0, values=params, unit=65)

    def get_status(self):
        result = self.client.read_holding_registers(
            address=268,
            count=1,
            unit=65
        )

        if result.isError():
            return None

        raw_value = int(result.registers[0])
        binary_value = f"{raw_value:016b}"

        # bit0 또는 bit1이 1이면 파지 성공으로 판정
        grip_detected = 1 if raw_value in (2, 4) else 0

        return raw_value, binary_value, grip_detected
    
    def close_connection(self):
        self.client.close()
# ##############################################################################
# [ADD] GRIPPER CONTROL CLASS END
# ##############################################################################

class TargetScanNode(Node):

    def __init__(self):
        super().__init__("robot_control_node")

        self.gripper = AdaptiveGripper(GRIPPER_IP,GRIPPER_PORT) 

        self.scan_thread = None
        self.detected_coordinate = None

        #topic

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # publisher 설정
        self.pub_table_scan = self.create_publisher(Bool, '/table_scan_finished', qos_profile)
        self.pub_hand_start = self.create_publisher(Bool, '/hand_scan_start', qos_profile)
        self.pub_hand_finish = self.create_publisher(Bool, '/hand_scan_finished', qos_profile)
        self.pub_task_completed = self.create_publisher(Bool, '/task_completed', qos_profile)
        self.pub_table_rescan_start = self.create_publisher(Bool, '/table_rescan_started', qos_profile)
        self.pub_table_rescan_finish = self.create_publisher(Bool, '/table_rescan_finished', qos_profile)
        self.pub_say = self.create_publisher(String, '/say', qos_profile)
        self.pub_error_status = self.create_publisher(String, '/robot_error_status', qos_profile)
        
        # service_client
        self.scan_table_client= self.create_client(ScanRequest, "/yolo_scan_request")
        self.grip_bbox_client = self.create_client(GripBoundingBox, "/grip_bounding_box")
        self.approach_client = self.create_client(Trigger, "/arrived_goal")

        # service_client(DB)
        self.db_fixed_pose_client = self.create_client(GetFixedPose, "/get_fixed_pose")
        self.db_scan_case_client = self.create_client(GetScanCase, "/get_scan_case")

        # service_server
        self.get_keyword_service = self.create_service(VoiceKeyword, "/get_keyword", self.command_callback)
        self.ungrip_service = self.create_service(Trigger, "/ungrip", self.ungrip_callback)

        # action
        self.find_target_order_client = ActionClient(self, FindOrder, "/find_target_order")
        self.find_hand_order_client = ActionClient(self, FindOrder, "/find_hand_order")        
        
        self.target_scanner = ConeScanner(
            node=self,
            action_client=self.find_target_order_client,
            scan_tilt_angle=SCAN_TILT_ANGLE,
            scan_point_count=SCAN_POINT_COUNT,
            scan_velocity=SCAN_VELOCITY,
            scan_acceleration=SCAN_ACCELERATION,
        )
        self.hand_scanner = ConeScanner(
            node=self,
            action_client=self.find_hand_order_client,
            scan_tilt_angle=SCAN_TILT_ANGLE,
            scan_point_count=SCAN_POINT_COUNT,
            scan_velocity=SCAN_VELOCITY,
            scan_acceleration=SCAN_ACCELERATION,
        )

        self.get_logger().info("target_scan_node 시작")


    # VLA로 부터 명령 수신 시 실행되는 콜백 함수
    def command_callback(self, request, response):
        target_names = request.target.strip().split()
        goal_name = request.goal.strip()

        self.get_logger().info(f"명령 수신: target={target_names}, goal={goal_name}")

        home_success = self.back_to_home() # 좌표 초기화

        if not home_success:
            response.accepted = False
            return response
        
        if goal_name == "table_scan":
            threading.Thread(
                target=self.run_table_scan,
                args=(goal_name,),
                daemon=True,
            ).start()
            response.accepted = True
            return response

        threading.Thread(
            target=self.execute_robot_task,
            args=(target_names, goal_name),
            daemon=True,
        ).start()
        response.accepted = True
        return response

    # 메인 로직: target 좌표를 DB에서 가져와서 이동 후, goal 좌표로 이동
    def execute_robot_task(self, target_names, goal_name):

        # goal 좌표 설정
        if not goal_name :
            goal_name = "hand"

        response = self.get_object_from_db(goal_name)

        if list(response.pose) == [0,0,0,0,0,0]:
            self.report_error(f"{goal_name} DB 좌표 수신 실패")
            return
        
        goal_coordinate = list(response.pose)
        self.get_logger().info(f"DB 목표좌표 설정 완료:{goal_coordinate}")
                
        if goal_name == "hand":
            hand_scan = Bool()
            hand_scan.data = True
            self.pub_hand_start.publish(hand_scan) 
            center_pose = list(goal_coordinate)

            i = 0
            for i in range(3):

                scan_success = self.run_cone_scan(center_pose, goal_name)
                if scan_success:
                    break
                self.get_logger().info(f"{goal_name} 좌표 탐색 실패 try {i+1}/3")
            
            if not scan_success:
                self.report_error(f"{goal_name} 좌표 탐색 실패")
                self.back_to_home()
                return
            goal_coordinate[:3] = [round(value, 2) for value in self.detected_coordinate[:3]]
            self.pub_hand_finish.publish(hand_scan)
            self.get_logger().info(f"목표좌표 수정 완료:{goal_coordinate}")
        
        # target 좌표를 DB에서 가져와서 이동
        for target_name in target_names:

            response = self.get_object_from_db(target_name)
            if list(response.pose) == [0,0,0,0,0,0]:
                self.report_error(f"{target_name} DB 좌표 수신 실패")
                continue
            target_coordinate = list(response.pose)

            self.get_logger().info(f"DB 타깃좌표 설정 완료:{target_coordinate}")

            find_target = False
            find_retry = 0
            max_find_retry = 3

            move_name = f"{target_name}_above"
            target_above = list(target_coordinate)
            target_above[2] = round(target_above[2] + 100.0, 2)
                                        
            move_success = self.move_to_position(move_name, target_above)

            if not move_success:
                self.report_error(f"{move_name} 이동 실패")
                self.back_to_home()
                continue

            self.gripper.move_gripper(width_val=1000, force_val=200)

            while not find_target:     
                
                grip_request = GripBoundingBox.Request()
                grip_request.target = target_name

                grip_response = self.call_service(self.grip_bbox_client, grip_request, timeout=10.0)

                if grip_response is None:
                    self.report_error("YOLO 파지 정보 수신 실패")
                    self.back_to_home()
                    return

                self.get_logger().info(
                    f"YOLO 파지 정보 수신: "
                    f"coordinate={list(grip_response.coordinate)}, "
                    f"bbox_width={grip_response.bbox_width}, "
                    f"bbox_height={grip_response.bbox_height}, "
                    f"depth={grip_response.camera_depth_z},"
                    f"grip_angle_deg={grip_response.grip_angle_deg},"
                    f"is_find={grip_response.is_find},"
                )
                # =========================
                # YOLO 좌표로 이동
                # =========================
                if grip_response.is_find:
                    target_coordinate = list(target_coordinate)
                    target_coordinate[:3] = [round(value, 2) for value in grip_response.coordinate[:3]]
                    if target_name == "cable":
                        target_coordinate[2] = round(target_coordinate[2] - 24.0, 2)
                    else:                                
                        target_coordinate[2] = round(target_coordinate[2] - 45.0, 2)
                    ######################################
                    # +ADD
                    grip_angle = float(grip_response.grip_angle_deg)
        
                    # 그리퍼는 180° 뒤집어도 같은 파지 방향
                    grip_angle = (grip_angle + 90.0) % 180.0 - 90.0
        
                    current_joint = get_current_posj()
                    current_j6 = float(current_joint[5])
                    expected_j6 = current_j6 + grip_angle
        
                    J6_MIN = -180.0   # 실제 사용할 안전 범위로 설정
                    J6_MAX = 180.0
        
                    if not J6_MIN <= expected_j6 <= J6_MAX:
                        self.report_error(
                            f"6번 관절 예상각이 허용 범위를 초과합니다: "
                            f"현재={current_j6:.2f}°, "
                            f"보정={grip_angle:.2f}°, "
                            f"예상={expected_j6:.2f}°"
                        )
                        self.back_to_home()
                        return
        
                    target_coordinate[5] += grip_angle
        
                    # 자세 표현만 -180°~180°로 정리
                    target_coordinate[5] = (target_coordinate[5] + 180.0) % 360.0 -180
                    # +ADD
                    ######################################
                            
                    self.get_logger().info(f"타깃좌표 수정 완료:{target_coordinate}")

                    if target_name == "drink":    
                        move_name = f"{target_name}_above"
                        target_above = list(target_coordinate)
                        target_above[2] = round(target_above[2] + 100.0, 2)
                    else:
                        move_name = f"{target_name}_above"
                        target_above = list(target_coordinate)
                        target_above[2] = round(target_above[2] + 100.0, 2)
                                                
                    move_success = self.move_to_position(move_name, target_above)
        
                    if not move_success:
                        self.report_error(f"{move_name} 이동 실패")
                        self.back_to_home()
                        continue

                    current_pos_result = get_current_posx()
                    current_pose = current_pos_result[0]

                    current_x = float(current_pose[0])
                    current_y = float(current_pose[1])
                    target_above_x = float(target_above[0])
                    target_above_y = float(target_above[1])

                    x_error = target_above_x - current_x
                    y_error = target_above_y - current_y

                    xy_aligned = (abs(x_error) <= 10.0 and abs(y_error) <= 10.0)
                    self.get_logger().info(
                        f"XY 정렬 확인: "
                        f"현재=({current_x:.2f}, {current_y:.2f}), "
                        f"YOLO=({target_above_x:.2f}, {target_above_y:.2f}), "
                        f"오차=({x_error:.2f}, {y_error:.2f}), "
                        f"정렬={xy_aligned}"
                    )
                    if xy_aligned:
                        self.get_logger().info(
                            "현재 위치와 YOLO 좌표의 XY 오차가 ±10 mm 이내입니다."
                        )

                        # # 최종 파지 좌표 갱신
                        # target_coordinate[:3] = [
                        #     round(value, 2)
                        #     for value in grip_response.coordinate[:3]
                        # ]

                        # 여기서 Z 보정과 grip_angle 보정을 1회만 실행
                        find_target = True
                    continue

                find_retry += 1

                self.get_logger().warning(f"{target_name} 탐색 실패 ({find_retry}/{max_find_retry})")
                if find_retry >= max_find_retry:
                    self.report_error(f"{target_name} 최종 탐색 실패")
                    self.back_to_home()
                    break
                
                center_pose = target_above
                target = target_name

                target_scan = Bool()
                target_scan.data = True
                self.pub_table_rescan_start.publish(target_scan) 
                scan_success = self.run_cone_scan(center_pose, target)

                if not scan_success:
                    target_scan.data = False
                    self.pub_table_rescan_finish.publish(target_scan) 
                    continue
                    
                self.pub_table_rescan_finish.publish(target_scan) 
                
                target_coordinate = list(center_pose)
                target_coordinate[:3] = [round(value, 2) for value in self.detected_coordinate[:3]]

                if target_name == "drink":    
                    move_name = f"{target_name}_above"
                    target_above = list(target_coordinate)
                    target_above[2] = round(target_above[2] + 100.0, 2)
                else:
                    move_name = f"{target_name}_above"
                    target_above = list(target_coordinate)
                    target_above[2] = round(target_above[2] + 100.0, 2)
                                            
                move_success = self.move_to_position(move_name, target_above)
    
                if not move_success:
                    self.report_error(f"{move_name} 이동 실패")
                    self.back_to_home()
                    continue
            if not find_target:
                continue

            move_name = f"{target_name}_grip"
            move_success = self.move_to_position(move_name, target_coordinate)

            if not move_success:
                continue      

            # =========================
            # 물체 파지
            # =========================

            grip_success = self.run_adaptive_grip(
                bbox_w=grip_response.bbox_width,
                bbox_h=grip_response.bbox_height,
                dist=grip_response.camera_depth_z,
            )
            if not grip_success:
                self.report_error("Adaptive Grip 실패")
                self.back_to_home()
                continue
            # 물체를 잡은 후 위로 이동
            grip_above = list(target_coordinate)
            grip_above[2] = round(grip_above[2] + 100.0, 2)

            move_name = f"{target_name}_above_after_grip"
            move_success = self.move_to_position(move_name, grip_above)

            if not move_success:
                self.place_to_home()
                return

            # =========================
            # goal로 이동
            # =========================
            # 손에 전달할 때
            if goal_name == "hand":
                move_name = f"{goal_name}"
                move_success = self.move_to_position(move_name, goal_coordinate)
        
                if not move_success:
                    self.place_to_home()
                    return
                else:
                    self.get_logger().info(f"{goal_name} 이동 완료")
                    request = Trigger.Request()
                    approach_response = self.call_service(self.approach_client, request, timeout=5.0)

                    if approach_response is None:
                        self.report_error("arrived_goal 서비스 호출 실패")
                        self.place_to_home()
                        return

            # 손이 아닌 경우 "pos1, pos2, pos3"
            else:
                move_name = f"{goal_name}_approach"
                goal_above = list(goal_coordinate)
                goal_above[2] = round(grip_above[2] + 35.0, 2)

                move_success = self.move_to_position(move_name, goal_above)
                if not move_success:
                    self.place_to_home()
                    return
                place_success = self.place_with_compliance()

                if not place_success:
                    self.place_to_home()
                    return
                msg = Bool()
                msg.data = True
                self.pub_task_completed.publish(msg) 
                self.back_to_home()
                self.get_logger().info("로봇 작업 완료")

    # 내려놓기 위한 순응 제어
    def place_with_compliance(self, press_force=20.0, force_threshold=8.0, timeout=10.0):
        self.get_logger().info("컴플라이언스 하강 시작")

        contacted = False

        try:
            # ================================
            # 1. Base 기준
            # ================================
            set_ref_coord(DR_BASE)

            force = get_tool_force(DR_BASE)

            self.get_logger().info(f"Force 시작 전: Fx={force[0]:.2f}, Fy={force[1]:.2f}, Fz={force[2]:.2f}")

            # ================================
            # 2. Compliance ON
            # ================================
            ret_compliance = task_compliance_ctrl(stx=[3000, 3000, 500, 200, 200, 200])

            self.get_logger().info(f"Compliance ON, ret={ret_compliance}")
            # 트러블 슈팅 컴플라이언스 이후 0.5초 후 Force 켜기 두산패키지 이슈에 써있었음
            time.sleep(0.5)

            # ================================
            # 3. Force ON
            # ================================
            ret_force = set_desired_force(
                fd=[0, 0, -press_force, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                time=0.5,
                mod=DR_FC_MOD_REL,
            )

            self.get_logger().info(f"Force ON: -Z {press_force}N, ret={ret_force}")

            start_time = time.time()

            # ================================
            # 4. 실제 Fz를 직접 확인
            # ================================
            while rclpy.ok():

                force = get_tool_force(DR_BASE)
                fz = force[2]

                # 여기서 직접 판단
                if abs(fz) >= force_threshold:
                    contacted = True

                    self.get_logger().info(f"바닥 접촉 감지: |Fz|={abs(fz):.2f} N")
                    break

                if time.time() - start_time > timeout:
                    self.get_logger().warning("접촉 감지 시간 초과")
                    break

                time.sleep(0.05)

        finally:
            release_force(time=0.2)
            time.sleep(0.2)

            release_compliance_ctrl()

            self.get_logger().info(
                "Force / Compliance OFF"
            )

        # 접촉 못 했으면 절대 열지 않기
        if not contacted:
            self.report_error("바닥 접촉이 없어서 그리퍼를 열지 않습니다.")
            return False

        self.gripper.move_gripper(width_val=1000,force_val=200)

        time.sleep(1.0)

        self.get_logger().info("물체 내려놓기 완료")

        return True

    # 원뿔 스캔 실행
    def run_cone_scan(self, center_pose, target_name):
        if target_name == "hand":
            coordinate = self.hand_scanner.scan(center_pose, target_name)
        else:
            coordinate = self.target_scanner.scan(center_pose, target_name)

        if coordinate is not None:
            self.detected_coordinate = coordinate
            self.get_logger().info(f"{target_name} 최종 좌표: {coordinate}")
            return True
        
        self.get_logger().warning(f"{target_name} 좌표 탐색 실패")
        return False

    # ##############################################################################
    # table_scan 
    def run_table_scan(self, goal_name):
        """
        로봇을 두 개의 Waypoint로 순차 이동시킨다.

        각 Waypoint에 도착한 후 YOLO 노드에
        ScanRequest 서비스를 요청하고 응답을 기다린다.
        """
        self.get_logger().info("DB_SCAN_CASE 서비스 호출 시작")
        response = self.get_object_from_db(goal_name)

        if response is None:
            self.report_error(f"{goal_name} DB 좌표 수신 실패")
            return

        waypoints = []

        for i in range(1, 4):
            field_name = f"pose_{i}"
            pose = getattr(response, field_name, None)

            if pose is None:
                self.get_logger().warning(
                    f"{field_name} 값이 None이므로 반복을 종료합니다."
                )
                break

            waypoints.append(list(pose))

        time.sleep(1.0)

        self.get_logger().info("DB_SCAN_CASE 서비스 호출 완료")

        self.get_logger().info("테이블 스캔 작업을 시작합니다.")

        try:
            for i, waypoint in enumerate(waypoints, start=1):

                move_waypoint = list(waypoint)
                field_name = f"pose_name_{i}"
                waypoint_name = getattr(response, field_name, f"Waypoint_{i}")

                # 1. 로봇 이동
                move_success = self.move_to_position(waypoint_name, move_waypoint)

                if not move_success:
                    return

                # 2. YOLO 스캔 요청
                scan_success = self.request_yolo_scan(waypoint_name)

                if not scan_success:
                    self.report_error(f"{waypoint_name} YOLO 스캔 실패로 다음 Waypoint 이동을 중단합니다.")
                    return
                self.get_logger().info(f"{waypoint_name} 작업 완료")

            table_save_done = Bool()
            table_save_done.data = True
            self.pub_table_scan.publish(table_save_done)
            self.get_logger().info("모든 테이블 Waypoint 스캔이 완료되었습니다.")
            self.back_to_home()

        except Exception as error:
            self.report_error(f"테이블 스캔 중 예외 발생: {error}")

    # table_scan 시 waypoint마다 yolo에게 스캔 요청 및 응답을 기다리는 함수
    def request_yolo_scan(self, waypoint_name):
        """
        YOLO 노드의 /yolo_scan_request 서비스를 호출한다.

        반환값:
            True:   서비스 통신 성공 및 response.success == True
            False:  서비스 없음, 응답 시간 초과, 통신 예외 또는 response.success == False
        """

        self.get_logger().info(f"YOLO 서비스 확인 중: {waypoint_name}")
        # 서비스 서버가 살아있는지 확인 => True/ False
        service_ready = self.scan_table_client.wait_for_service(timeout_sec=YOLO_SERVICE_WAIT_TIMEOUT)
        if not service_ready:
            self.get_logger().error("/yolo_scan_request 서비스를 찾을 수 없습니다.")
            return False

        request = ScanRequest.Request()
        request.waypoint_id = waypoint_name
        self.get_logger().info(f"YOLO 스캔 요청 전송: waypoint_id={waypoint_name}")
        response = self.call_service( self.scan_table_client, request, timeout=YOLO_SCAN_RESPONSE_TIMEOUT)

        if response is None:
            self.get_logger().error(f"YOLO 서비스 응답 수신 실패: {waypoint_name}")
            return False
        
        # YOLO가 전달한 정보는 현재 제어 판단에는 사용하지 않고
        # 로그만 출력한다.
        self.get_logger().info(
            f"YOLO 응답 수신: "
            f"waypoint_id={waypoint_name}, "
            f"success={response.success}, "
            f"detected_count={response.detected_count}, "
            f"message='{response.message}'"
            f""
        )

        if not response.success:
            self.get_logger().error(f"YOLO 스캔 자체가 실패했습니다: {response.message}")
            return False

        return True

    # 그립퍼 Adaptive Grip logic start
    def run_adaptive_grip(self, bbox_w, bbox_h, dist):
        pixel_area = bbox_w * bbox_h
        real_area_estimate = pixel_area * (dist ** 2)
        K = 0.00001

        force = min(int(150 + (real_area_estimate * K)), 400)

        self.get_logger().info(
            f"그리퍼 계산 [면적:{pixel_area} | 거리:{dist}m | 힘:{force}]"
        )

        self.gripper.move_gripper(width_val=0, force_val=force)

        time.sleep(2.0)

        status_result = self.gripper.get_status()

        if status_result is None:
            self.get_logger().error("그리퍼 상태 레지스터 읽기 실패")
            return False

        raw_value, binary_value, grip_detected = status_result

        self.get_logger().info(
            f"그리퍼 상태: "
            f"raw={raw_value}, "
            f"hex=0x{raw_value:04X}, "
            f"binary={binary_value}, "
            f"bit1(grip_detected)={grip_detected}"
        )

        if grip_detected == 1:
            self.get_logger().info(">> 파지 성공!")
            return True

        self.get_logger().warning(">> 파지 실패")
        return False

    # 그립퍼 열기 서비스 콜백
    def ungrip_callback(self, request, response):            
        self.gripper.move_gripper(width_val=1000, force_val=200)
        self.back_to_home()
        msg = Bool()
        msg.data = True
        self.pub_task_completed.publish(msg)  # 최종 작업 완료
        response.success = True
        return response
    
    # Move to position using movel
    def move_to_position(self, move_name, target_position):

        self.get_logger().info(f"{move_name} ({target_position}) 이동 시작")
        try:
            ret = movel(target_position, vel=VELOCITY, acc=ACCELERATION, ref= DR_BASE)
            if ret != 0:
                self.report_error(f"{move_name} 이동 실패")
                return False       
            self.get_logger().info(f"{move_name} 이동 완료")
        except Exception as exc:
            self.report_error(f"{move_name} ({target_position}) 이동 중 오류 발생: {exc}, {move_name} 이동 실패")
            return False
        return True

    # DB에서 좌표 가져오는 함수
    def get_object_from_db(self, target_data):
        """
        req_type에 따라 호출할 서비스를 결정합니다.
        - "fixed_pose": pos1, pos2, HAND_SCAN 등 6자유도 포즈 (GetFixedPose)
        - "scan_case": CASE_1 등 웨이포인트 2개 (GetScanCase)
        """

        if target_data == "table_scan":
            request = GetScanCase.Request()
            request.case_name = "CASE_1"
            return self.call_service(self.db_scan_case_client, request)


        elif target_data in ("hand", "pos1", "pos2", "pos3"):
            request = GetFixedPose.Request()
            if target_data == "hand":
                request.pose_name = "HAND_SCAN"
            else:
                request.pose_name = target_data
            return self.call_service(self.db_fixed_pose_client, request)
           
        else: # 기본값은 object
            request = GetFixedPose.Request()
            request.pose_name = target_data
            return self.call_service(self.db_fixed_pose_client, request)
           
    def call_service(self, client, request, timeout=YOLO_SERVICE_WAIT_TIMEOUT):
        future = client.call_async(request)

        response_event = threading.Event()
        future.add_done_callback(lambda _: response_event.set())

        received = response_event.wait(timeout=timeout)

        if not received:
            self.get_logger().error("서비스 응답 시간 초과")

            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"서비스 호출 실패: {exc}")
            return None

    def report_error(self, log_message, status_message=None):
        self.get_logger().error(log_message)
        msg = String()
        msg.data = (status_message if status_message is not None else log_message)
        self.pub_error_status.publish(msg)

    def back_to_home(self):
        ret = movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACCELERATION)
        if ret != 0:
            self.report_error("대기모드 복귀 실패")
            return False     
        self.get_logger().info("대기모드 복귀 완료")
        return True

    def place_to_home(self):

        # 1. HOME 위치로 복귀
        home_success = self.back_to_home()
        movel([0,0,-100,0,0,0], vel=VELOCITY, acc=ACCELERATION, ref=DR_FC_MOD_REL)

        if not home_success:
            return False

        # 2. 컴플라이언스로 내려놓기
        place_success = self.place_with_compliance()

        if not place_success:
            return False

        self.get_logger().info("HOME 위치 비상 내려놓기 완료")
        return True

def main(args=None):
    node = TargetScanNode()
    # 중요:
    # rclpy.spin(node)의 기본 global executor를 사용하지 않는다.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info("노드를 종료합니다.")

    finally:
        executor.shutdown()
        node.gripper.close_connection()
        node.destroy_node()
        dsr_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()