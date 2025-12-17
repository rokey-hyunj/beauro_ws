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


# ==========================================
# 2-1. 안전한 동작 래퍼 함수들
# ==========================================

# 전역 에러 핸들러 (나중에 설정됨)
_global_error_handler = None

def set_error_handler(handler):
    """전역 에러 핸들러 설정"""
    global _global_error_handler
    _global_error_handler = handler

def safe_movel(*args, **kwargs):
    """에러 체크가 포함된 movel"""
    from DSR_ROBOT2 import movel as _movel
    
    # 동작 전 에러 체크
    if _global_error_handler and not _global_error_handler.check_and_recover():
        raise RuntimeError("로봇 복구 실패 - 작업 중단")
    
    # 실제 동작 수행
    result = _movel(*args, **kwargs)
    
    # 동작 완료 대기
    time.sleep(0.1)
    
    # 동작 후 에러 체크
    if _global_error_handler and not _global_error_handler.check_and_recover():
        raise RuntimeError("로봇 복구 실패 - 작업 중단")
    
    return result

def safe_movej(*args, **kwargs):
    """에러 체크가 포함된 movej"""
    from DSR_ROBOT2 import movej as _movej
    
    # 동작 전 에러 체크
    if _global_error_handler and not _global_error_handler.check_and_recover():
        raise RuntimeError("로봇 복구 실패 - 작업 중단")
    
    # 실제 동작 수행
    result = _movej(*args, **kwargs)
    
    # 동작 완료 대기
    time.sleep(0.1)
    
    # 동작 후 에러 체크
    if _global_error_handler and not _global_error_handler.check_and_recover():
        raise RuntimeError("로봇 복구 실패 - 작업 중단")
    
    return result

def safe_gripper_control(mode):
    """에러 체크가 포함된 그리퍼 제어"""
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    
    # 동작 전 에러 체크
    if _global_error_handler and not _global_error_handler.check_and_recover():
        raise RuntimeError("로봇 복구 실패 - 작업 중단")
    
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
    
    # 동작 후 에러 체크
    if _global_error_handler and not _global_error_handler.check_and_recover():
        raise RuntimeError("로봇 복구 실패 - 작업 중단")
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
    
    # ROBOT_STATE 상수 (공식 문서 기준)
    STATE_INITIALIZING = 0
    STATE_STANDBY = 1
    STATE_MOVING = 2
    STATE_SAFE_OFF = 3
    STATE_TEACHING = 4
    STATE_SAFE_STOP = 5
    STATE_EMERGENCY_STOP = 6
    STATE_HOMING = 7
    STATE_RECOVERY = 8
    STATE_SAFE_STOP2 = 9
    STATE_SAFE_OFF2 = 10
    STATE_NOT_READY = 15
    
    # ROBOT_CONTROL 상수 (공식 문서 기준)
    CONTROL_INIT_CONFIG = 0
    CONTROL_ENABLE_OPERATION = 1
    CONTROL_RESET_SAFE_STOP = 2
    CONTROL_RESET_SAFE_OFF = 3
    CONTROL_RECOVERY_SAFE_STOP = 4
    CONTROL_RECOVERY_SAFE_OFF = 5
    CONTROL_RECOVERY_BACKDRIVE = 6
    CONTROL_RESET_RECOVERY = 7
    
    ERROR_STATES = {3, 5, 6, 9, 10}  # SAFE_OFF, SAFE_STOP, E_STOP, SAFE_STOP2, SAFE_OFF2
    MAX_ERROR_COUNT = 5  # 최대 연속 에러 허용 횟수
    
    def __init__(self, state_manager: TaskStateManager, node=None):
        self.state_manager = state_manager
        self.node = node  # ROS2 노드
        self._setup_service_clients()
    
    def _setup_service_clients(self):
        """ROS2 서비스 클라이언트 초기화"""
        if self.node is None:
            # DR_init에서 노드 가져오기
            import DR_init
            self.node = DR_init.__dsr__node
        
        if self.node:
            from dsr_msgs2.srv import SetRobotControl
            
            # SetRobotControl 서비스 클라이언트 생성
            service_name = f'/{ROBOT_ID}/system/set_robot_control'
            self.set_robot_control_client = self.node.create_client(
                SetRobotControl,
                service_name
            )
            
            print(f"🔧 서비스 클라이언트 생성: {service_name}")
        else:
            print("⚠️ ROS2 노드를 찾을 수 없습니다. 서비스 기반 복구를 사용할 수 없습니다.")
            self.set_robot_control_client = None
    
    def _call_set_robot_control(self, robot_control: int) -> bool:
        """
        SetRobotControl 서비스 호출
        
        Args:
            robot_control: ROBOT_CONTROL 값 (0-7)
        
        Returns:
            성공 여부
        """
        if self.set_robot_control_client is None:
            print("❌ SetRobotControl 서비스 클라이언트가 초기화되지 않았습니다")
            return False
        
        try:
            from dsr_msgs2.srv import SetRobotControl
            
            # 서비스 요청 생성
            request = SetRobotControl.Request()
            request.robot_control = robot_control
            
            # 서비스 대기 (최대 3초)
            if not self.set_robot_control_client.wait_for_service(timeout_sec=3.0):
                print("⚠️ SetRobotControl 서비스를 사용할 수 없습니다")
                return False
            
            # 서비스 호출
            future = self.set_robot_control_client.call_async(request)
            
            # 응답 대기
            import rclpy
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response and response.success:
                    print(f"   ✓ SetRobotControl({robot_control}) 성공")
                    return True
                else:
                    print(f"   ✗ SetRobotControl({robot_control}) 실패")
                    return False
            else:
                print("   ⚠️ 서비스 호출 시간 초과")
                return False
                
        except Exception as e:
            print(f"   ❌ 서비스 호출 예외: {e}")
            return False
    
    def check_and_recover(self) -> bool:
        """
        로봇 상태 확인 및 복구 시도 (SetRobotControl 서비스 사용)
        
        Returns:
            True: 정상 또는 복구 성공
            False: 작업 중단 필요
        """
        from DSR_ROBOT2 import get_robot_state, get_current_posj
        
        state = get_robot_state()
        
        # 정상 상태 (STANDBY, MOVING)
        if state in [self.STATE_STANDBY, self.STATE_MOVING]:
            self.state_manager.reset_error()
            return True
        
        # INITIALIZING 상태는 대기
        if state == self.STATE_INITIALIZING:
            print("⏳ 로봇 초기화 중...")
            time.sleep(1.0)
            return True
        
        # 에러 상태가 아니면 통과
        if state not in self.ERROR_STATES:
            return True
        
        # 에러 발생
        print(f"\n{'='*60}")
        print(f"🚨 로봇 에러 감지")
        print(f"   상태 코드: {state}")
        
        # 에러 상태 설명
        state_descriptions = {
            self.STATE_SAFE_OFF: "STATE_SAFE_OFF (서보 꺼짐 - 모터/브레이크 전원 차단)",
            self.STATE_SAFE_STOP: "STATE_SAFE_STOP (안전 정지 - 제어 일시정지)",
            self.STATE_EMERGENCY_STOP: "STATE_EMERGENCY_STOP (비상정지)",
            self.STATE_SAFE_STOP2: "STATE_SAFE_STOP2 (동작 범위 이탈 - 복구 필요)",
            self.STATE_SAFE_OFF2: "STATE_SAFE_OFF2 (동작 범위 이탈 - 복구 필요)",
        }
        print(f"   상태 설명: {state_descriptions.get(state, '알 수 없음')}")
        
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
        
        # 상태별 복구 프로세스
        return self._recover_by_state(state)
    
    def _recover_by_state(self, state: int) -> bool:
        """
        상태에 따른 복구 처리
        """
        from DSR_ROBOT2 import set_robot_control, get_robot_state, set_robot_mode
        
        # === 1. EMERGENCY_STOP (6) ===
        if state == self.STATE_EMERGENCY_STOP:
            print("⚠️ 비상정지 상태")
            print("   복구 절차:")
            print("   1. 티치펜던트 또는 컨트롤러의 비상정지 버튼을 해제하세요")
            print("   2. 티치펜던트에서 안전 경고창이 나타나면 확인을 누르세요")
            print("   3. 'Servo On' 버튼을 눌러 서보를 활성화하세요")
            print("   4. 준비 완료 후 아래 선택지를 입력하세요\n")
            
            decision = self._wait_user_decision()
            if decision == "s":
                return False
            
            # 비상정지 해제 확인
            time.sleep(0.5)
            new_state = get_robot_state()
            if new_state == self.STATE_EMERGENCY_STOP:
                print("❌ 비상정지가 아직 해제되지 않았습니다")
                return False
            
            # 비상정지 해제 후 SAFE_OFF나 SAFE_STOP 상태로 전환될 수 있음
            if new_state in [self.STATE_SAFE_OFF, self.STATE_SAFE_STOP]:
                return self._recover_by_state(new_state)
            
            return True
        
        # === 2. SAFE_STOP (5) ===
        elif state == self.STATE_SAFE_STOP:
            print("🔧 SAFE_STOP 상태 복구")
            print("   - 외력 감지, 충돌 등으로 인한 안전 정지")
            print("   - CONTROL_RESET_SAFE_STOP (2)를 사용하여 자동 복구 시도")
            print("   - 장애물을 제거한 후 'c'를 입력하세요\n")
            
            decision = self._wait_user_decision()
            if decision == "s":
                return False
            
            return self._execute_control_reset(
                self.CONTROL_RESET_SAFE_STOP,
                "SAFE_STOP",
                state
            )
        
        # === 3. SAFE_OFF (3) ===
        elif state == self.STATE_SAFE_OFF:
            print("⚠️ SAFE_OFF 상태 복구")
            print("   - 서보 모터 전원이 차단된 상태")
            print("   - 티치펜던트 복구 절차:")
            print("     1. 안전 경고창에서 'Reset' 버튼 클릭")
            print("     2. 'Servo On' 버튼 클릭하여 서보 활성화")
            print("   - 복구 완료 후 'c'를 입력하세요\n")
            
            decision = self._wait_user_decision()
            if decision == "s":
                return False
            
            return self._execute_control_reset(
                self.CONTROL_RESET_SAFE_OFF,
                "SAFE_OFF",
                state
            )
        
        # === 4. SAFE_STOP2 (9) - 동작 범위 이탈 ===
        elif state == self.STATE_SAFE_STOP2:
            print("⚠️ SAFE_STOP2 상태 - 동작 범위 이탈")
            print("   - 로봇이 동작 범위를 벗어났습니다")
            print("   - 복구 모드로 전환 필요")
            print("   복구 방법 선택:")
            print("   1. S/W 복구 (CONTROL_RECOVERY_SAFE_STOP)")
            print("   2. 수동 복구")
            
            decision = self._wait_user_decision()
            if decision == "s":
                return False
            
            # S/W 복구 시도
            return self._execute_recovery_mode(
                self.CONTROL_RECOVERY_SAFE_STOP,
                "SAFE_STOP2"
            )
        
        # === 5. SAFE_OFF2 (10) - 동작 범위 이탈 ===
        elif state == self.STATE_SAFE_OFF2:
            print("⚠️ SAFE_OFF2 상태 - 동작 범위 이탈")
            print("   - 로봇이 동작 범위를 벗어났습니다")
            print("   - 복구 모드로 전환 필요")
            print("   복구 방법 선택:")
            print("   1. S/W 복구 (CONTROL_RECOVERY_SAFE_OFF)")
            print("   2. H/W 백드라이브 복구 (CONTROL_RECOVERY_BACKDRIVE)")
            print("      ※ 백드라이브 복구 후에는 컨트롤러 재부팅 필요")
            
            decision = self._wait_user_decision()
            if decision == "s":
                return False
            
            # S/W 복구 시도
            return self._execute_recovery_mode(
                self.CONTROL_RECOVERY_SAFE_OFF,
                "SAFE_OFF2"
            )
        
        return False
    
    def _execute_control_reset(self, control_cmd: int, state_name: str, current_state: int) -> bool:
        """
        SetRobotControl을 사용한 리셋 실행
        """
        from DSR_ROBOT2 import set_robot_control, get_robot_state, set_robot_mode
        
        print(f"\n🔄 {state_name} 복구 시도...")
        
        for attempt in range(3):
            try:
                print(f"   시도 {attempt + 1}/3: SetRobotControl({control_cmd})")
                
                # 1. SetRobotControl로 리셋
                result = set_robot_control(control_cmd)
                print(f"   → SetRobotControl 결과: {result}")
                time.sleep(1.5)
                
                # 2. 상태 확인
                new_state = get_robot_state()
                print(f"   → 현재 상태: {new_state}")
                
                # STANDBY(1) 상태로 전환되었으면 성공
                if new_state == self.STATE_STANDBY:
                    print("   ✅ STANDBY 상태 전환 성공")
                    
                    # AUTO 모드로 전환
                    print("   → AUTO 모드 전환 중...")
                    set_robot_mode(1)
                    time.sleep(1.0)
                    
                    final_state = get_robot_state()
                    if final_state in [self.STATE_STANDBY, self.STATE_MOVING]:
                        print("✅ 복구 성공 - 작업 재개\n")
                        self.state_manager.reset_error()
                        return True
                
                # 에러 상태가 아니면 성공으로 간주
                if new_state not in self.ERROR_STATES:
                    print("✅ 복구 성공\n")
                    self.state_manager.reset_error()
                    return True
                
                if attempt < 2:
                    print(f"   ⚠️ 아직 에러 상태 (상태: {new_state}) - 재시도\n")
                    time.sleep(1.0)
                
            except Exception as e:
                print(f"   ❌ 예외 발생: {e}")
                if attempt < 2:
                    time.sleep(1.0)
        
        print(f"\n❌ {state_name} 자동 복구 실패")
        print("티치펜던트에서 수동 복구를 진행해주세요\n")
        
        # 수동 복구 후 재시도
        print("수동 복구 완료 후 다시 시도하시겠습니까?")
        retry = self._wait_user_decision()
        if retry == "c":
            time.sleep(0.5)
            return self.check_and_recover()  # 재귀 호출
        
        return False
    
    def _execute_recovery_mode(self, control_cmd: int, state_name: str) -> bool:
        """
        복구 모드 전환 (SAFE_STOP2, SAFE_OFF2)
        """
        from DSR_ROBOT2 import set_robot_control, get_robot_state
        
        print(f"\n🔄 {state_name} 복구 모드 전환 시도...")
        
        try:
            # 1. 복구 모드로 전환 (CONTROL_RECOVERY_SAFE_STOP or CONTROL_RECOVERY_SAFE_OFF)
            print(f"   1단계: SetRobotControl({control_cmd}) - RECOVERY 모드로 전환")
            result = set_robot_control(control_cmd)
            print(f"   → 결과: {result}")
            time.sleep(2.0)
            
            new_state = get_robot_state()
            print(f"   → 현재 상태: {new_state}")
            
            if new_state == self.STATE_RECOVERY:
                print("   ✅ RECOVERY 모드 전환 성공")
                print("\n   티치펜던트에서 로봇을 수동으로 동작 범위 내로 이동시켜주세요")
                print("   이동 완료 후 'c'를 입력하세요\n")
                
                decision = self._wait_user_decision()
                if decision == "s":
                    return False
                
                # 2. RECOVERY 모드 리셋 (CONTROL_RESET_RECOVERY)
                print(f"   2단계: SetRobotControl({self.CONTROL_RESET_RECOVERY}) - STANDBY로 전환")
                result = set_robot_control(self.CONTROL_RESET_RECOVERY)
                print(f"   → 결과: {result}")
                time.sleep(2.0)
                
                final_state = get_robot_state()
                print(f"   → 최종 상태: {final_state}")
                
                if final_state == self.STATE_STANDBY:
                    print("✅ 복구 완료 - 작업 재개\n")
                    self.state_manager.reset_error()
                    return True
            
            print(f"⚠️ 복구 모드 전환 실패 (현재 상태: {new_state})")
            
        except Exception as e:
            print(f"❌ 복구 중 예외 발생: {e}")
        
        return False
    
    def _wait_user_decision(self) -> str:
        """사용자 결정 대기 (나중에 Web UI로 대체 가능)"""
        print("┌─────────────────────────────────┐")
        print("│  계속 (c) / 중단 (s)?           │")
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
# 4. 액체 작업 (개별 동작 단위 에러 핸들링)
# ==========================================
def execute_liquid(library, recipe, state_mgr, error_handler):
    from DSR_ROBOT2 import posx, posj
    
    # 전역 에러 핸들러 설정
    set_error_handler(error_handler)

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

    try:
        # === 스텝: 홈 이동 ===
        step = TaskStep.LIQUID_GRAB_UP
        if not state_mgr.should_skip("liquid", None, None, step):
            print(f"🔄 {step.value}")
            state_mgr.update("liquid", None, None, step, liquid_key)
            
            safe_movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)
            safe_gripper_control("init")

        # === 스텝: 스포이드 집기 ===
        step = TaskStep.LIQUID_GRAB
        if not state_mgr.should_skip("liquid", None, None, step):
            print(f"🔄 {step.value} - Picking {liquid_key}")
            state_mgr.update("liquid", None, None, step, liquid_key)
            
            safe_movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
            safe_movel(p_grab, vel=VEL_WORK, acc=ACC)
            safe_gripper_control("hold")
            safe_movel(p_grab_up, vel=VEL_MOVE, acc=ACC)

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
                
                safe_movel(p_cup_up, vel=VEL_MOVE, acc=ACC)
                safe_gripper_control("squeeze")
                safe_movel(p_cup_down, vel=VEL_WORK, acc=ACC)
                safe_gripper_control("hold")
                safe_movel(p_cup_up, vel=VEL_MOVE, acc=ACC)

                # === 스텝: 배출 ===
                step = TaskStep.LIQUID_DISPENSE
                print(f"     ↳ {step.value}")
                state_mgr.update("liquid", tray_idx, c, step, liquid_key)
                
                safe_movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
                safe_movel(p_tray, vel=VEL_WORK, acc=ACC)
                safe_gripper_control("squeeze")
                safe_gripper_control("hold")
                safe_gripper_control("squeeze")
                safe_gripper_control("hold")
                safe_movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
                
                print(f"     ✅ 반복 {c+1} 완료")

        # === 스텝: 스포이드 정리 ===
        step = TaskStep.LIQUID_RETURN
        if not state_mgr.should_skip("liquid", None, None, step):
            print(f"🔄 {step.value}")
            state_mgr.update("liquid", None, None, step, liquid_key)
            
            safe_movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
            safe_movel(p_grab, vel=VEL_WORK, acc=ACC)
            safe_gripper_control("init")
            safe_movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
        
        print("✅ Liquid Process Complete")
        return True
        
    except RuntimeError as e:
        print(f"\n❌ Liquid 작업 중단: {e}")
        return False

# ==========================================
# 5. 분말 작업 (개별 동작 단위 에러 핸들링)
# ==========================================
def execute_powder(library, recipe, state_mgr, error_handler):
    from DSR_ROBOT2 import posx, posj
    
    # 전역 에러 핸들러 설정
    set_error_handler(error_handler)

    print("\n[Start] Powder Process")

    powder_key = recipe["selection"]["powder"]
    pow_data = library["powders"][powder_key]["poses"]
    
    p_grab = posx(pow_data["grab"]["posx"])
    xg, yg, zg, rxg, ryg, rzg = p_grab

    p_bowl = posx(pow_data["bowl"]["posx"])
    
    # scoop_1은 posx로 변경됨
    p_scoop_1 = posx(pow_data["scoop_1"]["posx"])
    
    # flat은 3개의 좌표 리스트
    p_flat_list = [posx(p) for p in pow_data["flat"]["posx"]]
    
    p_tray_base = pow_data["tray_base"]["posx"]
    p_pour_list = pow_data["pour"]["posj"]

    spoon_shift = -40 if powder_key == "powder_A" else 40
    POUR_ANGLE = -90 if powder_key == "powder_A" else 90
    
    # 스쿠핑 깊이 (y방향으로 밀기)
    SCOOP_Y_PUSH = 68.92  # mm

    try:
        # === 스텝: 스푼 집기 ===
        step = TaskStep.POWDER_GRAB
        if not state_mgr.should_skip("powder", None, None, step):
            print(f"🔄 {step.value} - Picking {powder_key}")
            state_mgr.update("powder", None, None, step, powder_key)
            
            safe_gripper_control("init")
            safe_movel(posx([xg, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
            safe_movel(p_grab, vel=VEL_WORK, acc=ACC)
            safe_gripper_control("squeeze")
            safe_movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
            safe_movel(posx([xg + spoon_shift, yg, zg+110, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)

        trays = recipe["trays"]
        for t_idx, t_cfg in trays.items():
            count = t_cfg["count"]["powder"]
            if count <= 0: continue
            
            tray_idx = int(t_idx)
            print(f">> Processing Tray #{tray_idx} (Count: {count})")

            p_tray = get_tray_pose(p_tray_base, tray_idx)
            p_pour = posj(p_pour_list[tray_idx - 1])

            for c in range(count):
                # === 스텝: 스쿠핑 이동 ===
                step = TaskStep.POWDER_SCOOP_MOVE
                if state_mgr.should_skip("powder", tray_idx, c, step):
                    print(f"  ⏭️ 스킵: 트레이 {tray_idx}, 반복 {c+1}/{count}")
                    continue
                
                print(f"  🔄 Loop {c+1}: {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                
                safe_movel(p_bowl, vel=VEL_MOVE, acc=ACC)
                safe_movel(p_scoop_1, vel=VEL_WORK, acc=ACC)

                # === 스텝: 스쿠핑 (y방향으로 밀기) ===
                step = TaskStep.POWDER_SCOOP
                print(f"     ↳ {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                
                # p_scoop_1에서 y값을 SCOOP_Y_PUSH만큼 증가
                xs1, ys1, zs1, rxs1, rys1, rzs1 = p_scoop_1
                p_scoop_2 = posx([xs1, ys1 + SCOOP_Y_PUSH, zs1, rxs1, rys1, rzs1])
                safe_movel(p_scoop_2, vel=VEL_WORK, acc=ACC)

                # === 스텝: 평탄화 (3개 좌표 순차 이동) ===
                step = TaskStep.POWDER_FLATTEN
                print(f"     ↳ {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                
                for i, p_flat in enumerate(p_flat_list):
                    print(f"        → 평탄화 {i+1}/3")
                    safe_movel(p_flat, vel=VEL_MOVE, acc=ACC)

                # === 스텝: 붓기 위치 이동 ===
                step = TaskStep.POWDER_POUR_MOVE
                print(f"     ↳ {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                
                safe_movel(p_tray, vel=VEL_MOVE, acc=ACC)
                safe_movej(p_pour, vel=VEL_MOVE, acc=ACC)

                # === 스텝: 붓기 ===
                step = TaskStep.POWDER_POUR
                print(f"     ↳ {step.value}")
                state_mgr.update("powder", tray_idx, c, step, powder_key)
                
                j1, j2, j3, j4, j5, j6 = p_pour
                p_pour_j = posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE])
                safe_movej(p_pour_j, vel=VEL_WORK, acc=ACC)
                
                for _ in range(3):
                    safe_movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE + 5.0]), vel=VEL_WORK, acc=ACC)
                    safe_movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE - 5.0]), vel=VEL_WORK, acc=ACC)
                
                safe_movel(p_tray, vel=VEL_MOVE, acc=ACC)
                print(f"     ✅ 반복 {c+1} 완료")

        # === 스텝: 스푼 정리 ===
        step = TaskStep.POWDER_RETURN
        if not state_mgr.should_skip("powder", None, None, step):
            print(f"🔄 {step.value}")
            state_mgr.update("powder", None, None, step, powder_key)
            
            safe_movel(posx([xg + spoon_shift, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
            safe_movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
            safe_movel(p_grab, vel=VEL_WORK, acc=ACC)
            safe_gripper_control("init")
            safe_movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)

        print("✅ Powder Process Complete")
        return True
        
    except RuntimeError as e:
        print(f"\n❌ Powder 작업 중단: {e}")
        return False


# ==========================================
# 6. 스틱 작업 (개별 동작 단위 에러 핸들링)
# ==========================================
def execute_sticks(library, recipe, state_mgr, error_handler):
    from DSR_ROBOT2 import posx, posj
    
    # 전역 에러 핸들러 설정
    set_error_handler(error_handler)

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
    
    try:
        safe_movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)

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
            
            safe_gripper_control("init")
            safe_movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)
            safe_movel(GRAB, vel=VEL_WORK, acc=ACC)
            safe_gripper_control("squeeze")
            safe_movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)

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

            safe_movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
            safe_movel(p_tray_down, vel=VEL_WORK, acc=ACC)

            for _ in range(3):
                for i, p in enumerate(stir_poses_tray):
                    if i == len(stir_poses_tray) - 1:
                        safe_movel(p, vel=VEL_WORK, acc=ACC, radius=0)
                    else:
                        safe_movel(p, vel=VEL_WORK, acc=ACC, radius=10)

            safe_movel(p_tray_up, vel=VEL_MOVE, acc=ACC)

            # === 스텝: 스틱 버리기 ===
            step = TaskStep.STICK_DROP
            print(f"     ↳ {step.value}")
            state_mgr.update("stick", tray_idx, None, step)

            safe_movel(DROP, vel=VEL_MOVE, acc=ACC)
            safe_movel(posx([
                DROP[0], DROP[1], DROP[2] - 158,
                DROP[3], DROP[4], DROP[5]
            ]), vel=VEL_WORK, acc=ACC)

            safe_gripper_control("init")
            safe_movel(DROP, vel=VEL_MOVE, acc=ACC)
            
            print(f"     ✅ 트레이 {tray_idx} 완료")

        safe_movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
        print("✅ Stick Process Complete")
        return True
        
    except RuntimeError as e:
        print(f"\n❌ Stick 작업 중단: {e}")
        return False

# ==========================================
# 7. 트레이 배출 작업 (개별 동작 단위 에러 핸들링)
# ==========================================
def execute_tray(library, state_mgr, error_handler):
    from DSR_ROBOT2 import posx, posj
    
    # 전역 에러 핸들러 설정
    set_error_handler(error_handler)

    print("\n[Start] Tray Out Process")

    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["tray_out"]

    READY_1 = posx(stick_poses["ready_1"]["posx"])
    READY_2 = posx(stick_poses["ready_2"]["posx"])
    GRAB = posx(stick_poses["grab"]["posx"])
    DROP = posx(stick_poses["drop"]["posx"])
    FINISH = posx(stick_poses["finished"]["posx"])

    try:
        # === 스텝: 트레이 잡기 ===
        step = TaskStep.TRAY_GRAB
        if not state_mgr.should_skip("tray", None, None, step):
            print(f"🔄 {step.value}")
            state_mgr.update("tray", None, None, step)
            
            safe_gripper_control("init")
            safe_movel(READY_1, vel=VEL_MOVE, acc=ACC)
            safe_movel(READY_2, vel=VEL_MOVE, acc=ACC)
            safe_movel(GRAB, vel=VEL_MOVE, acc=ACC)
            safe_gripper_control("hold")
            safe_movel(posx(GRAB[0:2] + [GRAB[2] + 100] + GRAB[3:6]), vel=VEL_MOVE, acc=ACC)

        # === 스텝: 트레이 배출 ===
        step = TaskStep.TRAY_DROP
        if not state_mgr.should_skip("tray", None, None, step):
            print(f"     ↳ {step.value}")
            state_mgr.update("tray", None, None, step)
            
            safe_movel(DROP, vel=VEL_MOVE, acc=ACC)
            safe_gripper_control("init")
            safe_movel(FINISH, vel=VEL_MOVE, acc=ACC)
            safe_movel(posx(FINISH[0:2] + [FINISH[2] + 80] + FINISH[3:6]), vel=VEL_MOVE, acc=ACC)

        safe_movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
        print("✅ Tray Out Process Complete")
        return True
        
    except RuntimeError as e:
        print(f"\n❌ Tray 작업 중단: {e}")
        return False


# ==========================================
# 8. 메인 실행 함수
# ==========================================
def main(args=None):
    import os

    rclpy.init(args=args)
    node = rclpy.create_node("recipe_integration", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # 상태 관리자 및 에러 핸들러 초기화
    state_mgr = TaskStateManager()
    has_saved_state = state_mgr.load()  # 이전 상태 복원 시도
    error_handler = RobotErrorHandler(state_mgr, node)  # 노드 전달

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