import rclpy
import DR_init
import yaml
import time
import json
from enum import Enum
from typing import Optional, Dict, Any
from dsr_msgs2.srv import SetRobotControl

# ==========================================
# 1. 설정 및 상수 (Configuration)
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 속도 설정
VEL_MOVE = 1000
VEL_WORK = 150   # 작업 시 천천히
ACC = 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 트레이 설정 (2x3)
TRAY_PITCH_X = 57.0  # 구멍 간격 (mm)
TRAY_PITCH_Y = 38.0

# ==========================================
# 2. 에러 핸들링 시스템
# ==========================================

class TaskStep(Enum):
    """작업 단계 정의"""
    # Liquid
    LIQUID_GRAB_UP = "LIQUID_GRAB_UP"
    LIQUID_GRAB = "LIQUID_GRAB"
    LIQUID_SUCTION = "LIQUID_SUCTION"
    LIQUID_DISPENSE = "LIQUID_DISPENSE"
    LIQUID_RETURN = "LIQUID_RETURN"
    
    # Powder
    POWDER_GRAB = "POWDER_GRAB"
    POWDER_SCOOP_MOVE = "POWDER_SCOOP_MOVE"
    POWDER_SCOOP = "POWDER_SCOOP"
    POWDER_FLATTEN = "POWDER_FLATTEN"
    POWDER_POUR_MOVE = "POWDER_POUR_MOVE"
    POWDER_POUR = "POWDER_POUR"
    POWDER_RETURN = "POWDER_RETURN"
    
    # Stick
    STICK_GRAB = "STICK_GRAB"
    STICK_STIR = "STICK_STIR"
    STICK_DROP = "STICK_DROP"
    
    # Tray
    TRAY_GRAB = "TRAY_GRAB"
    TRAY_DROP = "TRAY_DROP"
    
    DONE = "DONE"

class TaskStateManager:
    """작업 상태 저장/복원 관리자"""
    
    def __init__(self, state_file: str = "./robot_task_state.json"):
        self.state_file = state_file
        self.state = self._default_state()
    
    def _default_state(self) -> Dict[str, Any]:
        return {
            "task_type": None,      # "powder", "liquid", "stick", "tray"
            "tray_idx": None,       # 현재 트레이 인덱스
            "count_idx": None,      # 현재 반복 횟수
            "step": None,           # 현재 세부 단계
            "timestamp": None,      # 마지막 저장 시간
            "error_count": 0,       # 연속 에러 횟수
            "material_key": None    # powder_A, powder_B, liquid_A 등
        }
    
    def save(self):
        """상태 저장"""
        self.state["timestamp"] = time.time()
        try:
            with open(self.state_file, "w") as f:
                json.dump(self.state, f, indent=2)
            # 간결한 로그
            if self.state["task_type"]:
                print(f"💾 저장: [{self.state['task_type']}] Tray={self.state['tray_idx']}, Count={self.state['count_idx']}, Step={self.state['step']}")
        except Exception as e:
            print(f"❌ 상태 저장 실패: {e}")
    
    def load(self):
        """상태 복원"""
        try:
            with open(self.state_file, "r") as f:
                self.state = json.load(f)
            print(f"📂 상태 복원: {self.state}")
            return True
        except FileNotFoundError:
            print("ℹ️ 저장된 상태 없음 (새 작업 시작)")
            return False
        except Exception as e:
            print(f"❌ 상태 로드 실패: {e}")
            return False
    
    def clear(self):
        """상태 초기화"""
        self.state = self._default_state()
        self.save()
    
    def should_skip(self, task_type: str, tray_idx: Optional[int], count_idx: Optional[int], step: TaskStep) -> bool:
        """이미 완료한 작업인지 확인"""
        # 저장된 작업이 없으면 스킵 안함
        if self.state["task_type"] is None:
            return False
            
        # 다른 작업 타입이면 스킵 안함
        if self.state["task_type"] != task_type:
            return False
        
        # 트레이가 없는 작업 (tray_out, 초기 grab 등)
        if tray_idx is None:
            # 저장된 트레이도 None이 아니면 이미 트레이 작업 시작했으므로 스킵 안함
            if self.state["tray_idx"] is not None:
                return False
            # 저장된 단계가 현재 단계보다 이후면 스킵
            if self.state["step"] and self._step_completed(TaskStep(self.state["step"]), step):
                return True
            return False
        
        # 트레이 인덱스 비교
        if self.state["tray_idx"] is None:
            return False
        
        # 이전 트레이는 전부 스킵
        if tray_idx < self.state["tray_idx"]:
            return True
        
        # 다음 트레이는 스킵 안함
        if tray_idx > self.state["tray_idx"]:
            return False
        
        # 같은 트레이인 경우
        # count가 없는 작업이면 단계만 비교
        if count_idx is None:
            if self.state["step"] and self._step_completed(TaskStep(self.state["step"]), step):
                return True
            return False
        
        # count 비교
        if self.state["count_idx"] is None:
            return False
            
        # 이전 반복은 스킵
        if count_idx < self.state["count_idx"]:
            return True
        
        # 다음 반복은 스킵 안함
        if count_idx > self.state["count_idx"]:
            return False
        
        # 같은 반복에서 완료된 단계는 스킵
        if self.state["step"] and self._step_completed(TaskStep(self.state["step"]), step):
            return True
        
        return False
    
    def _step_completed(self, completed_step: TaskStep, current_step: TaskStep) -> bool:
        """완료된 단계인지 확인"""
        # 같은 단계이거나 이전 단계면 스킵
        order_map = {
            # Liquid
            TaskStep.LIQUID_GRAB_UP: 1,
            TaskStep.LIQUID_GRAB: 2,
            TaskStep.LIQUID_SUCTION: 3,
            TaskStep.LIQUID_DISPENSE: 4,
            TaskStep.LIQUID_RETURN: 5,
            
            # Powder
            TaskStep.POWDER_GRAB: 1,
            TaskStep.POWDER_SCOOP_MOVE: 2,
            TaskStep.POWDER_SCOOP: 3,
            TaskStep.POWDER_FLATTEN: 4,
            TaskStep.POWDER_POUR_MOVE: 5,
            TaskStep.POWDER_POUR: 6,
            TaskStep.POWDER_RETURN: 7,
            
            # Stick
            TaskStep.STICK_GRAB: 1,
            TaskStep.STICK_STIR: 2,
            TaskStep.STICK_DROP: 3,
            
            # Tray
            TaskStep.TRAY_GRAB: 1,
            TaskStep.TRAY_DROP: 2,
            
            TaskStep.DONE: 999
        }
        
        return order_map.get(current_step, 0) <= order_map.get(completed_step, 0)
    
    def update(self, task_type: str, tray_idx: Optional[int], count_idx: Optional[int], 
               step: TaskStep, material_key: Optional[str] = None):
        """현재 작업 상태 업데이트"""
        self.state["task_type"] = task_type
        self.state["tray_idx"] = tray_idx
        self.state["count_idx"] = count_idx
        self.state["step"] = step.value
        if material_key:
            self.state["material_key"] = material_key
        self.save()
    
    def increment_error(self):
        """에러 카운트 증가"""
        self.state["error_count"] += 1
        self.save()
    
    def reset_error(self):
        """에러 카운트 초기화"""
        self.state["error_count"] = 0


class RobotErrorHandler:
    """로봇 에러 감지 및 복구 처리"""
    
    ERROR_STATES = {3, 5, 6, 9, 10}
    MAX_ERROR_COUNT = 5
    
    def __init__(self, state_manager: TaskStateManager):
        self.state_manager = state_manager
        self.node = DR_init._dsr_node 

    def _call_set_robot_control(self, control_cmd: int) -> bool:
        SERVICE_NAME = f'/{ROBOT_ID}/system/set_robot_control'
        
        client = self.node.create_client(SetRobotControl, SERVICE_NAME)
        if not client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error(f'서비스를 찾을 수 없음: {SERVICE_NAME}')
            return False

        request = SetRobotControl.Request()

        try:
            request.robot_control = control_cmd 
        except AttributeError:
             self.node.get_logger().error("SetRobotControl 요청 필드 이름(robot_control)이 틀렸을 수 있습니다.")
             return False
        
        future = client.call_async(request)
        
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1) # 짧은 시간동안 스핀
            if future.done():
                try:
                    response = future.result()
                    # 4. 응답 결과 확인
                    if response.success:
                        self.node.get_logger().info(f'SetRobotControl({control_cmd}) 성공')
                        return True
                    else:
                        self.node.get_logger().warn(f'SetRobotControl({control_cmd}) 실패: {response.message}')
                        return False
                except Exception as e:
                    self.node.get_logger().error(f'서비스 호출 중 예외 발생: {e}')
                    return False
        
        return False
    
    def check_and_recover(self) -> bool:
        """
        로봇 상태 확인 및 복구 시도
        
        Returns:
            True: 정상 또는 복구 성공
            False: 작업 중단 필요
        """
        from DSR_ROBOT2 import get_robot_state, get_current_posj
        
        state = get_robot_state()
        
        # 정상 상태
        if state not in self.ERROR_STATES:
            self.state_manager.reset_error()
            return True
        
        # 에러 발생
        print(f"\n{'='*60}")
        print(f"🚨 로봇 에러 감지")
        print(f"   상태 코드: {state}")
        
        # 에러 상태 설명
        error_descriptions = {
            3: "Safe Off (충돌 또는 외력 감지)",
            5: "Safe Stop (서보 꺼짐)",
            6: "Emergency Stop (비상정지)",
        }
        print(f"   상태 설명: {error_descriptions.get(state, '알 수 없음')}")
        
        try:
            current_pos = get_current_posj()
            print(f"   현재 위치: {[round(j, 2) for j in current_pos]}")
        except:
            pass
        
        print(f"{'='*60}\n")
        
        self.state_manager.increment_error()
        
        # 최대 에러 횟수 초과
        if self.state_manager.state["error_count"] >= self.MAX_ERROR_COUNT:
            print(f"❌ 연속 에러 {self.MAX_ERROR_COUNT}회 초과 - 작업 중단")
            return False
        
        # 비상정지는 수동 해제만 가능
        if state == 6:  # EMERGENCY_STOP
            print("⚠️ 비상정지 상태")
            print("   1. 티치펜던트 또는 컨트롤러의 비상정지 버튼을 해제하세요")
            print("   2. 해제 후 아래 선택지를 입력하세요\n")
            decision = self._wait_user_decision()
            if decision == "s":
                return False
            
            # 비상정지 해제 확인
            time.sleep(0.5)
            state = get_robot_state()
            if state == 6:
                print("❌ 비상정지가 아직 해제되지 않았습니다")
                return False
        
        # Safe Stop (3) 또는 Safe Off (5) 처리
        if state == 5 or state == 9:
            print("⚠️ Safe Off 상태")
            print("   - 티치펜던트에서 복구가 필요합니다")
            print("   - 티치펜던트 화면에서:")
            print("     1. 노란색 팝업이 표시되면 'Reset' 버튼 클릭")
            print("     2. 수동 모드로 전환됩니다")
            print("     3. 'Servo On' 버튼 클릭")
            print("     4. 'Auto' 모드로 전환")
            print("   - 완료 후 'c'를 입력하세요\n")
        elif state == 3 or state == 10:
            print("🔧 Safe Stop 상태 복구 시도")
            print("   - 로봇이 외력이나 충돌로 인해 정지했습니다")
            print("   - 장애물을 제거하고 'c'를 입력하면 자동 복구를 시도합니다\n")
        
        decision = self._wait_user_decision()
        
        if decision == "s":
            print("🛑 사용자가 작업 중단 선택")
            return False
        
        # 복구 시도
        print("\n🔄 로봇 복구 중...")

        try:
            # DSR 라이브러리 임포트 중 SetRobotControl은 제거합니다.
            # from DSR_ROBOT2 import set_safe_stop_reset_type, SetRobotControl # <== 기존
            from DSR_ROBOT2 import set_safe_stop_reset_type

            print("   → Safe Stop 리셋...")
            
            # 1. Safe Stop Reset Type 설정 (DSR 라이브러리 함수 사용)
            set_safe_stop_reset_type(2)
            
            # 2. 로봇 컨트롤을 SAFE_STOP_RESET (값 2)으로 설정하는 ROS 2 서비스 호출
            # (값 3은 SAFE_OFF_RESET일 가능성이 높으므로, 여기서는 SAFE_STOP_RESET인 2를 사용합니다.)
            # SetRobotControl(2)를 ROS 2 서비스 클라이언트 래퍼 함수로 호출
            print("   → SetRobotControl(2) 호출 (SAFE_STOP_RESET)...")
            
            self._call_set_robot_control(3)
            time.sleep(1.0)
            self._call_set_robot_control(2)
            time.sleep(1.0)
            self._call_set_robot_control(1)
            time.sleep(3.0)

            new_state = get_robot_state()
            print(f"   ⭐ 복구 후 현재 상태: {new_state}")
            
            if new_state not in self.ERROR_STATES:
                # 복구가 성공하여 정상 상태로 돌아왔다면, 사용자에게 재개 확인
                print("\n✅ 로봇 상태 정상 복구 완료. 작업을 재개합니다.")
                self.state_manager.reset_error()
                return True
            else:
                # 복구 시도 후에도 여전히 에러 상태인 경우
                print("\n⚠️ 자동 복구 시도 후에도 에러 상태가 지속됩니다.")
                print("   - 티치펜던트 확인 후 'c'를 입력하여 수동 복구를 시도하거나 's'를 입력하여 중단하세요.")
                
                decision_after_fail = self._wait_user_decision()
                if decision_after_fail == "s":
                    return False
                
                # 사용자가 'c'를 선택했다면, 복구 루프를 재시작 (재귀 호출)
                return self.check_and_recover() # <== 핵심! 함수 재호출
                
        except Exception as e:
            print(f"   ⚠️ 안전 복구 실패: {e}")
            return False # 예외 발생 시 복구 실패
    
    def _wait_user_decision(self) -> str:
        """사용자 결정 대기 (나중에 Web UI로 대체 가능)"""
        print("┌─────────────────────────────────┐")
        print("│  계속 (c) / 중단 (s)?             │")
        print("└─────────────────────────────────┘")
        while True:
            decision = input("선택: ").strip().lower()
            if decision in ("c", "s"):
                return decision
            print("⚠️ 'c' 또는 's'를 입력하세요.")


# ==========================================
# 3. 공통 유틸리티 함수 (Utility)
# ==========================================
def load_yaml(path):
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"[ERROR] YAML 로드 실패: {path}\n{e}")
        return None

def get_tray_pose(base_pose, tray_idx):
    """
    YAML의 기준점(1번 구멍)을 바탕으로 tray_idx(1~6)의 좌표를 계산
    """
    from DSR_ROBOT2 import posx
    
    x, y, z, rx, ry, rz = base_pose[:6]
    idx = tray_idx - 1
    row = idx // 2
    col = idx % 2

    xt = x + (col * TRAY_PITCH_X)
    yt = y - (row * TRAY_PITCH_Y)
    
    return posx([xt, yt, z, rx, ry, rz])

def make_stir_pose(base_stir, p_tray_down, TRAY_BASE):
    from DSR_ROBOT2 import posx

    dx = p_tray_down[0] - TRAY_BASE[0]
    dy = p_tray_down[1] - TRAY_BASE[1]

    return posx([
        base_stir[0] + dx,
        base_stir[1] + dy,
        p_tray_down[2],
        base_stir[3],
        base_stir[4],
        base_stir[5],
    ])

def flatten_and_shake(center_pose):
    """파우더 평탄화 (좌우 흔들기)"""
    from DSR_ROBOT2 import movel, posx
    x, y, z, rx, ry, rz = center_pose
    shake_width = 5.0
    
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    for _ in range(5):
        movel(posx([x+shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
        movel(posx([x-shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)

def gripper_control(mode):
    """그리퍼 통합 제어"""
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    
    if mode == "init":
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
    elif mode == "squeeze":
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(1.0)
    elif mode == "hold":
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.5)

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    print(">>> Robot Initialized")

# ==========================================
# 4. 액체 작업 (에러 핸들링 추가)
# ==========================================
def execute_liquid(library, recipe, state_mgr, error_handler):
    from DSR_ROBOT2 import posx, posj, movel, movej

    print("\n[Start] Liquid Process")
    
    liquid_key = recipe["selection"]["liquid"]
    liq_data = library["liquids"][liquid_key]["poses"]
    
    p_grab_up = posx(liq_data["grab_up"]["value"])
    p_grab    = posx(liq_data["grab"]["value"])
    p_cup_up  = posx(liq_data["cup_up"]["value"])
    p_cup_down = posx(liq_data["cup_down"]["value"])
    
    try:
        tray_base_raw = liq_data["trays"]["values"][0] 
    except:
        tray_base_raw = liq_data["trays"]["value"]

    # === 스텝: 홈 이동 ===
    step = TaskStep.LIQUID_GRAB_UP
    if not state_mgr.should_skip("liquid", None, None, step):
        print(f"🔄 {step.value}")
        state_mgr.update("liquid", None, None, step, liquid_key)
        if not error_handler.check_and_recover(): return False
        
        movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)
        gripper_control("init")

    # === 스텝: 스포이드 집기 ===
    step = TaskStep.LIQUID_GRAB
    if not state_mgr.should_skip("liquid", None, None, step):
        print(f"🔄 {step.value} - Picking {liquid_key}")
        state_mgr.update("liquid", None, None, step, liquid_key)
        if not error_handler.check_and_recover(): return False
        
        movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
        movel(p_grab, vel=VEL_WORK, acc=ACC)
        gripper_control("hold")
        movel(p_grab_up, vel=VEL_MOVE, acc=ACC)

    # === 작업 루프 ===
    trays = recipe["trays"]
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"]["liquid"]
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        print(f">> Processing Tray #{tray_idx} (Count: {count})")
        
        p_tray = get_tray_pose(tray_base_raw, tray_idx)
        p_tray_up = posx(list(p_tray))
        p_tray_up[2] += 50.0
        
        for c in range(count):
            # === 스텝: 흡입 ===
            step = TaskStep.LIQUID_SUCTION
            if state_mgr.should_skip("liquid", tray_idx, c, step):
                print(f"  ⏭️ 스킵: 트레이 {tray_idx}, 반복 {c+1}/{count}")
                continue
            
            print(f"  🔄 Loop {c+1}: {step.value}")
            state_mgr.update("liquid", tray_idx, c, step, liquid_key)
            if not error_handler.check_and_recover(): return False
            
            movel(p_cup_up, vel=VEL_MOVE, acc=ACC)
            gripper_control("squeeze")
            movel(p_cup_down, vel=VEL_WORK, acc=ACC)
            gripper_control("hold")
            movel(p_cup_up, vel=VEL_MOVE, acc=ACC)

            # === 스텝: 배출 ===
            step = TaskStep.LIQUID_DISPENSE
            print(f"     ↳ {step.value}")
            state_mgr.update("liquid", tray_idx, c, step, liquid_key)
            if not error_handler.check_and_recover(): return False
            
            movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
            movel(p_tray, vel=VEL_WORK, acc=ACC)
            gripper_control("squeeze")
            gripper_control("hold")
            gripper_control("squeeze")
            gripper_control("hold")
            movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
            
            print(f"     ✅ 반복 {c+1} 완료")

    # === 스텝: 스포이드 정리 ===
    step = TaskStep.LIQUID_RETURN
    if not state_mgr.should_skip("liquid", None, None, step):
        print(f"🔄 {step.value}")
        state_mgr.update("liquid", None, None, step, liquid_key)
        if not error_handler.check_and_recover(): return False
        
        movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
        movel(p_grab, vel=VEL_WORK, acc=ACC)
        gripper_control("init")
        movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    
    print("✅ Liquid Process Complete")
    return True

# ==========================================
# 5. 분말 작업 (에러 핸들링 추가)
# ==========================================
def execute_powder(library, recipe, state_mgr, error_handler):
    from DSR_ROBOT2 import posx, posj, movel, movej, get_current_posx, get_current_posj

    print("\n[Start] Powder Process")

    powder_key = recipe["selection"]["powder"]
    pow_data = library["powders"][powder_key]["poses"]
    
    p_grab = posx(pow_data["grab"]["posx"])
    xg, yg, zg, rxg, ryg, rzg = p_grab

    p_bowl = posx(pow_data["bowl"]["posx"])
    
    # scoop_1은 posx로 변경됨
    p_scoop_1 = posx(pow_data["scoop"]["posx"])
    
    # flat은 3개의 좌표 리스트
    p_flat_list = [posx(p) for p in pow_data["flat"]["posx"]]
    
    p_tray_base = pow_data["tray_base"]["posx"]

    spoon_shift = -40 if powder_key == "powder_A" else 40
    POUR_ANGLE = -130 if powder_key == "powder_A" else 130
    
    # 스쿠핑 깊이 (y방향으로 밀기)
    SCOOP_Y_PUSH = 68.92  # mm

    try:
        # === 스텝: 스푼 집기 ===
        step = TaskStep.POWDER_GRAB
        if not state_mgr.should_skip("powder", None, None, step):
            print(f"🔄 {step.value} - Picking {powder_key}")
            state_mgr.update("powder", None, None, step, powder_key)
            
            gripper_control("init")
            movel(posx([xg, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
            movel(p_grab, vel=VEL_WORK, acc=ACC)
            gripper_control("squeeze")
            movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
            movel(posx([xg + spoon_shift, yg, zg+110, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)

        trays = recipe["trays"]
        for t_idx, t_cfg in trays.items():
            count = t_cfg["count"]["powder"]
            if count <= 0: continue
            
            tray_idx = int(t_idx)
            print(f">> Processing Tray #{tray_idx} (Count: {count})")

            p_tray = get_tray_pose(p_tray_base, tray_idx)
            # p_pour = posj(p_pour_list[tray_idx - 1])

            for c in range(count):
                # === 스텝: 스쿠핑 이동 ===
                step = TaskStep.POWDER_SCOOP_MOVE
                if state_mgr.should_skip("powder", tray_idx, c, step):
                    print(f"  ⏭️ 스킵: 트레이 {tray_idx}, 반복 {c+1}/{count}")
                    continue
                
                print(f"  🔄 Loop {c+1}: {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                
                movel(p_bowl, vel=VEL_MOVE, acc=ACC)
                movel(p_scoop_1, vel=VEL_WORK, acc=ACC)

                # === 스텝: 스쿠핑 (y방향으로 밀기) ===
                step = TaskStep.POWDER_SCOOP
                print(f"     ↳ {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                
                # p_scoop_1에서 y값을 SCOOP_Y_PUSH만큼 증가
                xs1, ys1, zs1, rxs1, rys1, rzs1 = p_scoop_1
                p_scoop_2 = posx([xs1, ys1 + SCOOP_Y_PUSH, zs1, rxs1, rys1, rzs1])
                movel(p_scoop_2, vel=VEL_WORK, acc=ACC)

                # === 스텝: 평탄화 (3개 좌표 순차 이동) ===
                step = TaskStep.POWDER_FLATTEN
                print(f"     ↳ {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                if not error_handler.check_and_recover(): return False
                
                for i, p_flat in enumerate(p_flat_list):
                    print(f"        → 평탄화 {i+1}/3")
                    movel(p_flat, vel=VEL_MOVE, acc=ACC)

                # === 스텝: 붓기 위치 이동 ===
                step = TaskStep.POWDER_POUR_MOVE
                print(f"     ↳ {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                if not error_handler.check_and_recover(): return False
                
                movel(p_tray, vel=VEL_MOVE, acc=ACC)

                # === 스텝: 붓기 ===
                step = TaskStep.POWDER_POUR
                print(f"     ↳ {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                if not error_handler.check_and_recover(): return False
                
                j1, j2, j3, j4, j5, j6 = get_current_posj()
                movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE]), vel=VEL_WORK, acc=ACC)
                
                # === 스텝: 털기 ===
                for _ in range(3):
                    movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE + 10.0]), vel=VEL_WORK, acc=ACC)
                    movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE - 10.0]), vel=VEL_WORK, acc=ACC)
                
                movel(p_tray, vel=VEL_MOVE, acc=ACC)
                print(f"     ✅ 반복 {c+1} 완료")

        # === 스텝: 스푼 정리 ===
        step = TaskStep.POWDER_RETURN
        if not state_mgr.should_skip("powder", None, None, step):
            print(f"🔄 {step.value}")
            state_mgr.update("powder", None, None, step, powder_key)
            if not error_handler.check_and_recover(): return False
            
            (x, y, z, rx, ry, rz), _ = get_current_posx()
            movel(posx([x, y, z + 150, rx, ry, rz]), vel=VEL_MOVE, acc=ACC) # 트레이에서 위로
            movel(posx([xg + spoon_shift, yg, 500, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
            movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
            movel(p_grab, vel=VEL_WORK, acc=ACC)
            gripper_control("init")
            (x, y, z, rx, ry, rz), _ = get_current_posx()
            movel(posx([x, y, z + 200, rx, ry, rz]), vel=VEL_WORK, acc=ACC)
            movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)

        print("✅ Powder Process Complete")
        return True
        
    except RuntimeError as e:
        print(f"\n❌ Powder 작업 중단: {e}")
        return False

# ==========================================
# 6. 스틱 작업 (에러 핸들링 추가)
# ==========================================
def execute_sticks(library, recipe, state_mgr, error_handler):
    from DSR_ROBOT2 import posx, posj, movel, movej

    print("\n[Start] Stick Process")

    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["stick"]

    GRAB = posx(stick_poses["grab"]["posx"])
    GRAB_UP = posx(stick_poses["grab_up"]["posx"])
    TRAY_BASE = posx(stick_poses["tray"]["posx"])
    STIR_POSES_BASE = [posx(p) for p in stick_poses["stir"]["posx"]]
    DROP = posx(stick_poses["drop"]["posx"])

    TRAY_UP_Z, TRAY_DOWN_Z = 550, 427

    trays = recipe["trays"]
    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)

    for t_idx in trays:
        tray_idx = int(t_idx)
        print(f">> Processing Tray #{tray_idx}")

        # === 스텝: 스틱 집기 ===
        step = TaskStep.STICK_GRAB
        if state_mgr.should_skip("stick", tray_idx, None, step):
            print(f"  ⏭️ 스킵: 트레이 {tray_idx}")
            continue
        
        print(f"  🔄 {step.value}")
        state_mgr.update("stick", tray_idx, None, step)
        if not error_handler.check_and_recover(): return False
        
        gripper_control("init")
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)
        movel(GRAB, vel=VEL_WORK, acc=ACC)
        gripper_control("squeeze")
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)

        p_tray = get_tray_pose(TRAY_BASE, tray_idx)
        p_tray_up = posx([
            p_tray[0], p_tray[1], TRAY_UP_Z,
            p_tray[3], p_tray[4], p_tray[5],
        ])
        p_tray_down = posx([
            p_tray[0], p_tray[1], TRAY_DOWN_Z,
            p_tray[3], p_tray[4], p_tray[5]
        ])

        stir_poses_tray = [
            make_stir_pose(p, p_tray_down, TRAY_BASE)
            for p in STIR_POSES_BASE
        ]

        # === 스텝: 젓기 ===
        step = TaskStep.STICK_STIR
        print(f"     ↳ {step.value}")
        state_mgr.update("stick", tray_idx, None, step)
        if not error_handler.check_and_recover(): return False

        movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
        movel(p_tray_down, vel=VEL_WORK, acc=ACC)

        for _ in range(3):
            for i, p in enumerate(stir_poses_tray):
                if i == len(stir_poses_tray) - 1:
                    movel(p, vel=VEL_MOVE, acc=ACC, radius=0)
                else:
                    movel(p, vel=VEL_MOVE, acc=ACC, radius=10)

        movel(p_tray_up, vel=VEL_MOVE, acc=ACC)

        # === 스텝: 스틱 버리기 ===
        step = TaskStep.STICK_DROP
        print(f"     ↳ {step.value}")
        state_mgr.update("stick", tray_idx, None, step)
        if not error_handler.check_and_recover(): return False

        movel(DROP, vel=VEL_MOVE, acc=ACC)

        gripper_control("init")
        movel(DROP, vel=VEL_MOVE, acc=ACC)
        
        print(f"     ✅ 트레이 {tray_idx} 완료")

    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
    print("✅ Stick Process Complete")
    return True

# ==========================================
# 7. 트레이 배출 작업 (에러 핸들링 추가)
# ==========================================
def execute_tray(library, state_mgr, error_handler):
    from DSR_ROBOT2 import posx, posj, movel, movej

    print("\n[Start] Tray Out Process")

    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["tray_out"]

    READY_1 = posx(stick_poses["ready_1"]["posx"])
    READY_2 = posx(stick_poses["ready_2"]["posx"])
    GRAB = posx(stick_poses["grab"]["posx"])
    DROP = posx(stick_poses["drop"]["posx"])
    FINISH = posx(stick_poses["finished"]["posx"])

    # === 스텝: 트레이 잡기 ===
    step = TaskStep.TRAY_GRAB
    if not state_mgr.should_skip("tray", None, None, step):
        print(f"🔄 {step.value}")
        state_mgr.update("tray", None, None, step)
        if not error_handler.check_and_recover(): return False
        
        gripper_control("init")
        movel(READY_1, vel=VEL_MOVE, acc=ACC)
        movel(READY_2, vel=VEL_MOVE, acc=ACC)
        movel(GRAB, vel=VEL_MOVE, acc=ACC)
        gripper_control("hold")
        movel(posx(GRAB[0:2] + [GRAB[2] + 200] + GRAB[3:6]), vel=VEL_MOVE, acc=ACC)

    # === 스텝: 트레이 배출 ===
    step = TaskStep.TRAY_DROP
    if not state_mgr.should_skip("tray", None, None, step):
        print(f"     ↳ {step.value}")
        state_mgr.update("tray", None, None, step)
        if not error_handler.check_and_recover(): return False
        
        movel(DROP, vel=VEL_MOVE, acc=ACC)
        gripper_control("init")
        movel(FINISH, vel=VEL_MOVE, acc=ACC)
        movel(posx(FINISH[0:2] + [FINISH[2] + 80] + FINISH[3:6]), vel=VEL_MOVE, acc=ACC)

    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
    print("✅ Tray Out Process Complete")
    return True


# ==========================================
# 8. 메인 실행 함수
# ==========================================
def main(args=None):
    import os

    rclpy.init(args=args)
    node = rclpy.create_node("recipe_integration", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    DR_init._dsr_node = rclpy.create_node("dsr_client_node")

    # 상태 관리자 및 에러 핸들러 초기화
    state_mgr = TaskStateManager()
    has_saved_state = state_mgr.load()  # 이전 상태 복원 시도
    error_handler = RobotErrorHandler(state_mgr)

    try:
        initialize_robot()

        BASE_DIR = os.path.dirname(os.path.abspath(__file__))

        library = load_yaml(os.path.join(BASE_DIR, "material_library.yaml"))
        recipe = load_yaml(os.path.join(BASE_DIR, "recipe.yaml"))

        if library and recipe:
            # 재시작 상태 확인
            if has_saved_state and state_mgr.state["task_type"]:
                print(f"\n{'='*60}")
                print(f"📍 이전 작업 재개:")
                print(f"   작업: {state_mgr.state['task_type']}")
                print(f"   트레이: {state_mgr.state['tray_idx']}")
                print(f"   반복: {state_mgr.state['count_idx']}")
                print(f"   단계: {state_mgr.state['step']}")
                print(f"{'='*60}\n")
                
                user_confirm = input("이전 작업을 재개하시겠습니까? (y/n): ").strip().lower()
                if user_confirm != 'y':
                    print("🔄 처음부터 새로 시작합니다")
                    state_mgr.clear()
            else:
                # 새 작업 시작
                state_mgr.clear()
            
            # 각 작업 실행 (에러 시 중단)
            # Liquid 작업이 완료되지 않았거나, Liquid로 재시작하는 경우
            if (state_mgr.state["task_type"] is None or 
                state_mgr.state["task_type"] == "liquid"):
                if not execute_liquid(library, recipe, state_mgr, error_handler):
                    print("\n[ERROR] Liquid task failed or stopped")
                    return
                # Liquid 완료 후 상태 초기화
                state_mgr.clear()
            
            # Powder 작업
            if (state_mgr.state["task_type"] is None or 
                state_mgr.state["task_type"] == "powder"):
                if not execute_powder(library, recipe, state_mgr, error_handler):
                    print("\n[ERROR] Powder task failed or stopped")
                    return
                # Powder 완료 후 상태 초기화
                state_mgr.clear()
            
            # Stick 작업
            if (state_mgr.state["task_type"] is None or 
                state_mgr.state["task_type"] == "stick"):
                if not execute_sticks(library, recipe, state_mgr, error_handler):
                    print("\n[ERROR] Stick task failed or stopped")
                    return
                # Stick 완료 후 상태 초기화
                state_mgr.clear()
            
            # Tray 작업
            if (state_mgr.state["task_type"] is None or 
                state_mgr.state["task_type"] == "tray"):
                if not execute_tray(library, state_mgr, error_handler):
                    print("\n[ERROR] Tray task failed or stopped")
                    return
                # 모든 작업 완료
                state_mgr.clear()
            
            print("\n" + "="*60)
            print("✅ ALL TASKS COMPLETED SUCCESSFULLY")
            print("="*60 + "\n")
            
        else:
            print("[ERROR] Failed to load YAML files.")

    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user")
        print(f"💾 현재 상태가 저장되었습니다: {state_mgr.state_file}")
        print(f"   다음 실행 시 중단된 지점부터 재개할 수 있습니다")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        print(f"💾 현재 상태가 저장되었습니다: {state_mgr.state_file}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()