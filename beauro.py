import rclpy
import DR_init
import yaml
import time
import os
import json
import firebase_admin
from enum import Enum
from typing import Optional, Dict, Any
from firebase_admin import credentials, db
from dsr_msgs2.srv import SetRobotControl

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 속도 설정
VEL_MOVE = 2000
VEL_WORK = 2000
ACC = 1000

# 트레이 설정
TRAY_PITCH_X = 57.0
TRAY_PITCH_Y = 38.0

# 경로 설정
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FIREBASE_KEY_PATH = os.path.join(BASE_DIR, "serviceAccountKey.json")
LIBRARY_YAML_PATH = os.path.join(BASE_DIR, "material_library.yaml")
FIREBASE_DB_URL = 'https://beauro-ac0ad-default-rtdb.asia-southeast1.firebasedatabase.app/'

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==========================================
# 2. 상태 관리 및 에러 핸들링 (Firebase 적용)
# ==========================================

class TaskStep(Enum):
    """작업 단계 정의"""
    # Liquid
    LIQUID_INIT = "LIQUID_INIT"
    LIQUID_PICK = "LIQUID_PICK"
    LIQUID_SUCTION = "LIQUID_SUCTION"
    LIQUID_DISPENSE = "LIQUID_DISPENSE"
    LIQUID_RETURN = "LIQUID_RETURN"
    
    # Powder
    POWDER_INIT = "POWDER_INIT"
    POWDER_PICK = "POWDER_PICK"
    POWDER_SCOOP_MOVE = "POWDER_SCOOP_MOVE"
    POWDER_SCOOP = "POWDER_SCOOP"
    POWDER_FLATTEN = "POWDER_FLATTEN"
    POWDER_POUR_MOVE = "POWDER_POUR_MOVE"
    POWDER_POUR = "POWDER_POUR"
    POWDER_RETURN = "POWDER_RETURN"
    
    # Stick
    STICK_PICK = "STICK_PICK"
    STICK_STIR = "STICK_STIR"
    STICK_DROP = "STICK_DROP"
    
    # Tray
    TRAY_PICK = "TRAY_PICK"
    TRAY_DROP = "TRAY_DROP"
    
    DONE = "DONE"

class TaskStateManager:
    """작업 상태 저장/복원 관리자 (Firebase 버전 - 안전성 강화)"""
    def __init__(self):
        # Firebase 내 저장 경로
        self.db_path = 'task_checkpoint' 
        self.state = self._default_state()
    
    def _default_state(self) -> Dict[str, Any]:
        return {
            "order_id": None,       # [중요] 이 키가 없어서 에러가 났었음
            "task_type": None,      
            "tray_idx": None,       
            "count_idx": None,      
            "step": None,           
            "material_key": None,   
            "error_count": 0,
            "timestamp": 0
        }
    
    def save(self):
        """Firebase에 상태 저장"""
        self.state["timestamp"] = int(time.time())
        try:
            # 로컬 파일 대신 DB에 set
            db.reference(self.db_path).set(self.state)
        except Exception as e:
            print(f"❌ Firebase 상태 저장 실패: {e}")
    
    def load(self):
        """Firebase에서 상태 불러오기 (안전 모드)"""
        try:
            data = db.reference(self.db_path).get()
            if data:
                # [수정] DB 데이터를 바로 대입하지 않고, 기본값 위에 업데이트합니다.
                # 이렇게 하면 DB에 'order_id'가 없어도 기본값 None이 유지되어 에러가 안 납니다.
                loaded_state = self._default_state()
                loaded_state.update(data) 
                self.state = loaded_state
                return True
            return False
        except Exception as e:
            print(f"❌ Firebase 상태 로드 실패: {e}")
            return False
    
    def clear(self):
        """상태 초기화 (DB도 초기화)"""
        self.state = self._default_state()
        self.save()

    def should_skip(self, current_order_id, task_type, tray_idx, count_idx, step) -> bool:
        """현재 단계가 이미 완료된 단계인지 확인"""
        # 저장된 주문 ID가 없거나(None) 현재와 다르면 스킵하지 않음
        if self.state.get("order_id") != current_order_id:
            return False
            
        saved_task = self.state.get("task_type")
        if saved_task is None: return False
        
        # 1. Task Type 비교 (순서: liquid -> powder -> stick -> tray)
        task_order = ["liquid", "powder", "stick", "tray"]
        try:
            curr_type_idx = task_order.index(task_type)
            saved_type_idx = task_order.index(saved_task)
        except ValueError: return False

        if curr_type_idx < saved_type_idx: return True 
        if curr_type_idx > saved_type_idx: return False 

        # 2. 같은 Task Type일 때 세부 비교
        saved_tray = self.state.get("tray_idx")
        if tray_idx is not None and saved_tray is not None:
            if tray_idx < saved_tray: return True
            if tray_idx > saved_tray: return False
        
        saved_count = self.state.get("count_idx")
        if count_idx is not None and saved_count is not None:
            if count_idx < saved_count: return True
            if count_idx > saved_count: return False
            
        if self.state.get("step"):
            if self._step_completed(TaskStep(self.state["step"]), step):
                return True
                
        return False

    def _step_completed(self, saved_step: TaskStep, current_step: TaskStep) -> bool:
        return saved_step == current_step

    def update(self, order_id, task_type, tray_idx, count_idx, step, material_key=None):
        self.state["order_id"] = order_id
        self.state["task_type"] = task_type
        self.state["tray_idx"] = tray_idx
        self.state["count_idx"] = count_idx
        self.state["step"] = step.value
        if material_key: self.state["material_key"] = material_key
        self.save()
        
class RobotErrorHandler:
    ERROR_STATES = {3, 5, 6, 9, 10} 
    
    def __init__(self, node, state_mgr):
        self.node = node
        self.state_mgr = state_mgr
        
    def _call_service(self, control_cmd):
        cli = self.node.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')
        if not cli.wait_for_service(timeout_sec=1.0): return False
        req = SetRobotControl.Request()
        req.robot_control = control_cmd
        future = cli.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done(): return future.result().success
        return False

    def _wait_for_web_decision(self, error_msg="Error Detected"):
        """
        [수정됨] 콘솔 입력 대신 Firebase 명령을 대기함
        """
        print(f"\n⏳ [WEB WAIT] {error_msg}")
        print("   Waiting for 'resume' or 'stop' from Web...")

        # 1. Firebase에 에러 상태 알림
        try:
            db.reference('robot_state').update({
                'state': 3,  # Error/Paused
                'error_message': error_msg,
                'is_error': True
            })
            # [수정] set(None) 대신 delete() 사용
            db.reference('command/recovery').delete()
        except: pass

        # 2. 명령 대기 루프
        while rclpy.ok():
            try:
                cmd = db.reference('command/recovery').get()
                
                if cmd == 'resume':
                    print("📩 Received: RESUME")
                    
                    # [수정] 명령 삭제는 delete() 사용
                    db.reference('command/recovery').delete()
                    
                    # [수정] None 대신 빈 문자열("") 사용
                    db.reference('robot_state').update({
                        'is_error': False, 
                        'error_message': "", 
                        'state': 2
                    })
                    return 'c' # Continue
                
                elif cmd == 'stop':
                    print("📩 Received: STOP")
                    # [수정] delete() 사용
                    db.reference('command/recovery').delete()
                    return 's' # Stop
                
            except Exception as e:
                # Polling 에러가 나도 죽지 않고 잠시 대기
                print(f"Firebase Polling Warning: {e}")
            
            time.sleep(1.0) # 1초 간격 확인
        
        return 's' # 강제 종료 시

    def check_and_recover(self):
        from DSR_ROBOT2 import get_robot_state, set_safe_stop_reset_type
        
        state = get_robot_state()
        
        # 정상 상태면 패스
        if state not in self.ERROR_STATES:
            if self.state_mgr.state["error_count"] > 0:
                self.state_mgr.state["error_count"] = 0
                self.state_mgr.save()
            return True
            
        print(f"\n🚨 ERROR DETECTED (State: {state})")
        self.state_mgr.state["error_count"] += 1
        self.state_mgr.save()
        
        # 에러 너무 많으면 중단 -> 웹에 결정 요청
        if self.state_mgr.state["error_count"] > 5:
            print("❌ Too many errors. Asking Web for decision.")
            decision = self._wait_for_web_decision("Too many errors. Force Stop?")
            if decision == 's': return False
            self.state_mgr.state["error_count"] = 0 

        # 비상정지(6)는 자동 복구 불가 -> 웹에 알림
        if state == 6:
            decision = self._wait_for_web_decision("Emergency Stop! Release button and click Resume.")
            if decision == 's': return False
            # Resume을 눌렀다면, 버튼이 해제되었는지 확인 후 진행
            if get_robot_state() == 6:
                print("⚠️ Still in Emergency Stop. Please release button.")
                return self.check_and_recover() # 재귀 호출로 다시 확인

        # 자동 복구 시도
        print("🔄 Attempting Auto-Recovery...")
        try:
            set_safe_stop_reset_type(2)
            self._call_service(3) # Reset
            time.sleep(1)
            self._call_service(2) # Safe Stop Reset
            time.sleep(1)
            self._call_service(1) # Servo On
            time.sleep(3)
            
            # 복구 확인
            if get_robot_state() not in self.ERROR_STATES:
                print("✅ Recovery Successful.")
                # [수정] 성공 시 에러 메시지 초기화 (None -> "")
                db.reference('robot_state').update({'is_error': False, 'error_message': ""})
                return True
            else:
                # 자동 복구 실패 시 웹에 도움 요청
                print("⚠️ Auto-Recovery Failed.")
                decision = self._wait_for_web_decision("Auto-recovery failed. Manual check required.")
                
                if decision == 's': return False
                if decision == 'c': return self.check_and_recover() # 다시 시도

        except Exception as e:
            print(f"Recovery Error: {e}")
            
        return False

# ==========================================
# 3. 유틸리티 함수
# ==========================================
def load_yaml(path):
    try:
        with open(path, "r") as f: return yaml.safe_load(f)
    except Exception as e:
        print(f"[ERROR] YAML Load Failed: {path}\n{e}")
        return None

def get_pose_value(data_dict, key):
    target = data_dict.get(key)
    if not target: return None
    if "posx" in target: return target["posx"]
    if "posj" in target: return target["posj"]
    if "value" in target: return target["value"]
    if isinstance(target, list): return target
    return None

def update_status(action, progress, well_id=0, state_code=2):
    print(f" -> [Action] {action} (Well: {well_id}, {int(progress)}%)")
    try:
        db.reference('robot_state').update({
            'state': state_code,
            'current_action': action,
            'progress_percent': int(progress),
            'current_well': well_id,
            'timestamp': int(time.time() * 1000)
        })
    except: pass

def get_tray_pose(base_pose, tray_idx):
    from DSR_ROBOT2 import posx
    x, y, z, rx, ry, rz = base_pose[:6]
    idx = tray_idx - 1
    row = idx // 2; col = idx % 2
    xt = x + (col * 60.0) 
    yt = y - (row * 40.0)
    return posx([xt, yt, z, rx, ry, rz])

def make_stir_pose(base_stir, p_tray_down, TRAY_BASE):
    from DSR_ROBOT2 import posx
    dx = p_tray_down[0] - TRAY_BASE[0]
    dy = p_tray_down[1] - TRAY_BASE[1]
    return posx([base_stir[0] + dx, base_stir[1] + dy, p_tray_down[2], base_stir[3], base_stir[4], base_stir[5]])

def flatten_and_shake(center_pose):
    from DSR_ROBOT2 import movel, posx
    x, y, z, rx, ry, rz = center_pose
    shake_width = 5.0
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    for _ in range(3):
        movel(posx([x+shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
        movel(posx([x-shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)

def gripper_control(mode):
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    if mode == "init": set_digital_output(1, OFF); set_digital_output(2, OFF)
    elif mode == "squeeze": set_digital_output(1, ON); set_digital_output(2, ON); wait(1.0)
    elif mode == "hold": set_digital_output(1, OFF); set_digital_output(2, ON); wait(1.5)

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP)
    print(">>> Robot Initialized")

# ==========================================
# 4. 공정 함수
# ==========================================

def execute_liquid(library, recipe, current_step, total_steps, order_id, state_mgr, err_handler):
    from DSR_ROBOT2 import posx, posj, movel, movej

    if not recipe["selection"].get("liquid"): return current_step

    liquid_key = recipe["selection"]["liquid"]
    liq_data = library["liquids"][liquid_key]["poses"]
    
    p_grab_up = posx(get_pose_value(liq_data, "grab_up"))
    p_grab    = posx(get_pose_value(liq_data, "grab"))
    
    if "cup_up" in liq_data:
        p_cup_up = posx(get_pose_value(liq_data, "cup_up"))
        p_cup_down = posx(get_pose_value(liq_data, "cup_down"))
    else:
        p_cup_up = posx(get_pose_value(liq_data, "cup"))
        p_cup_down = posx(list(p_cup_up)); p_cup_down[2] -= 150

    tray_data = liq_data["trays"]
    if "values" in tray_data: tray_base_raw = tray_data["values"][0]
    else: tray_base_raw = get_pose_value(liq_data, "trays")

    progress = (current_step / total_steps) * 100
    
    # === STEP: INIT ===
    step = TaskStep.LIQUID_INIT
    if not state_mgr.should_skip(order_id, "liquid", None, None, step):
        update_status(f"Setting up {liquid_key}", progress)
        state_mgr.update(order_id, "liquid", None, None, step, liquid_key)
        if not err_handler.check_and_recover(): return current_step
        
        movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)
        gripper_control("init")

    # === STEP: PICK ===
    step = TaskStep.LIQUID_PICK
    if not state_mgr.should_skip(order_id, "liquid", None, None, step):
        update_status(f"Picking Pipette ({liquid_key})", progress)
        state_mgr.update(order_id, "liquid", None, None, step, liquid_key)
        if not err_handler.check_and_recover(): return current_step

        movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
        movel(p_grab, vel=VEL_WORK, acc=ACC)
        gripper_control("hold")
        movel(p_grab_up, vel=VEL_MOVE, acc=ACC)

    trays = recipe["trays"]
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"].get("liquid", 0)
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        current_step += 1
        progress = (current_step / total_steps) * 100
        
        p_tray = get_tray_pose(tray_base_raw, tray_idx)
        p_tray_up = posx(list(p_tray))
        p_tray_up[2] += 50.0

        for c in range(count):
            # === STEP: SUCTION ===
            step = TaskStep.LIQUID_SUCTION
            if state_mgr.should_skip(order_id, "liquid", tray_idx, c, step): continue
            
            update_status(f"Dispensing {liquid_key} to Well {tray_idx}", progress, tray_idx)
            state_mgr.update(order_id, "liquid", tray_idx, c, step, liquid_key)
            if not err_handler.check_and_recover(): return current_step

            movel(p_cup_up, vel=VEL_MOVE, acc=ACC)
            gripper_control("squeeze")
            movel(p_cup_down, vel=VEL_WORK, acc=ACC)
            gripper_control("hold")
            movel(p_cup_up, vel=VEL_MOVE, acc=ACC)

            # === STEP: DISPENSE ===
            step = TaskStep.LIQUID_DISPENSE
            state_mgr.update(order_id, "liquid", tray_idx, c, step, liquid_key)
            if not err_handler.check_and_recover(): return current_step

            movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
            movel(p_tray, vel=VEL_WORK, acc=ACC)
            gripper_control("squeeze"); gripper_control("hold")
            gripper_control("squeeze"); gripper_control("hold")
            movel(p_tray_up, vel=VEL_MOVE, acc=ACC)

    # === STEP: RETURN ===
    step = TaskStep.LIQUID_RETURN
    if not state_mgr.should_skip(order_id, "liquid", None, None, step):
        update_status("Returning Pipette", progress)
        state_mgr.update(order_id, "liquid", None, None, step, liquid_key)
        if not err_handler.check_and_recover(): return current_step

        movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
        movel(p_grab, vel=VEL_WORK, acc=ACC)
        gripper_control("init")
        movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    
    return current_step

def execute_powder(library, recipe, current_step, total_steps, order_id, state_mgr, err_handler):
    from DSR_ROBOT2 import posx, posj, movel, movej

    if not recipe["selection"].get("powder"): return current_step

    powder_key = recipe["selection"]["powder"]
    pow_data = library["powders"][powder_key]["poses"]

    p_grab = posx(get_pose_value(pow_data, "grab"))
    xg, yg, zg, rxg, ryg, rzg = p_grab

    p_bowl = posx(get_pose_value(pow_data, "bowl"))
    p_scoop_1 = posj(get_pose_value(pow_data, "scoop_1"))
    p_scoop_2 = posj(get_pose_value(pow_data, "scoop_2"))
    p_scoop_3 = posj(get_pose_value(pow_data, "scoop_3"))
    p_flat = posx(get_pose_value(pow_data, "flat"))
    p_tray_base = get_pose_value(pow_data, "tray_base")
    p_pour_list = get_pose_value(pow_data, "pour")

    progress = (current_step / total_steps) * 100
    
    spoon_shift = -40 if powder_key == "powder_A" else 40

    # === STEP: PICK ===
    step = TaskStep.POWDER_PICK
    if not state_mgr.should_skip(order_id, "powder", None, None, step):
        update_status(f"Picking Spoon ({powder_key})", progress)
        state_mgr.update(order_id, "powder", None, None, step, powder_key)
        if not err_handler.check_and_recover(): return current_step

        gripper_control("init")
        movel(posx([xg, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
        movel(p_grab, vel=VEL_WORK, acc=ACC)
        gripper_control("squeeze")
        movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
        movel(posx([xg + spoon_shift, yg, zg+110, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)

    trays = recipe["trays"]
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"].get("powder", 0)
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        current_step += 1
        progress = (current_step / total_steps) * 100
        
        p_tray = get_tray_pose(p_tray_base, tray_idx)
        if p_pour_list and len(p_pour_list) >= tray_idx:
            p_pour = posj(p_pour_list[tray_idx - 1])
        else: continue

        for c in range(count):
            # === STEP: SCOOP & POUR ===
            step = TaskStep.POWDER_POUR
            if state_mgr.should_skip(order_id, "powder", tray_idx, c, step): continue

            update_status(f"Dispensing {powder_key} to Well {tray_idx}", progress, tray_idx)
            state_mgr.update(order_id, "powder", tray_idx, c, step, powder_key)
            if not err_handler.check_and_recover(): return current_step

            movel(p_bowl, vel=VEL_MOVE, acc=ACC) 
            movej(p_scoop_1, vel=VEL_WORK, acc=ACC)
            movej(p_scoop_2, vel=VEL_WORK, acc=ACC)
            movej(p_scoop_3, vel=VEL_WORK, acc=ACC)
            
            movel(p_flat, vel=VEL_MOVE, acc=ACC)
            flatten_and_shake(p_flat)

            movel(p_tray, vel=VEL_MOVE, acc=ACC)
            movej(p_pour, vel=VEL_MOVE, acc=ACC)
            
            POUR_ANGLE = -90 if powder_key == "powder_A" else 90
            j1, j2, j3, j4, j5, j6 = p_pour
            p_pour_j = posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE])
            movej(p_pour_j, vel=VEL_WORK, acc=ACC)
            
            for _ in range(3):
                movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE + 5.0]), vel=VEL_WORK, acc=ACC)
                movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE - 5.0]), vel=VEL_WORK, acc=ACC)
            
            movel(p_tray, vel=VEL_MOVE, acc=ACC)

    # === STEP: RETURN ===
    step = TaskStep.POWDER_RETURN
    if not state_mgr.should_skip(order_id, "powder", None, None, step):
        update_status("Returning Spoon", progress)
        state_mgr.update(order_id, "powder", None, None, step, powder_key)
        if not err_handler.check_and_recover(): return current_step

        movel(posx([xg + spoon_shift, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
        movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
        movel(p_grab, vel=VEL_WORK, acc=ACC)
        gripper_control("init")
        movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)
    
    return current_step

def execute_sticks(library, recipe, current_step, total_steps, order_id, state_mgr, err_handler):
    from DSR_ROBOT2 import posx, posj, movel, movej
    
    progress = (current_step / total_steps) * 100
    
    step = TaskStep.STICK_PICK
    if state_mgr.should_skip(order_id, "stick", None, None, step): return current_step

    update_status("Starting Mixing", progress)
    state_mgr.update(order_id, "stick", None, None, step)
    
    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["stick"]
    
    GRAB = posx(get_pose_value(stick_poses, "grab"))
    GRAB_UP = posx(get_pose_value(stick_poses, "grab_up"))
    TRAY_BASE = posx(get_pose_value(stick_poses, "tray"))
    STIR_POSES_BASE = [posx(p) for p in get_pose_value(stick_poses, "stir")]
    DROP = posx(get_pose_value(stick_poses, "drop"))
    
    TRAY_UP_Z, TRAY_DOWN_Z = 550, 427
    trays = recipe["trays"]

    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)

    for t_idx in trays:
        tray_idx = int(t_idx)
        well_counts = trays[t_idx]["count"]
        if (well_counts.get("liquid", 0) + well_counts.get("powder", 0)) == 0: continue

        current_step += 1
        progress = (current_step / total_steps) * 100
        
        step = TaskStep.STICK_STIR
        if state_mgr.should_skip(order_id, "stick", tray_idx, None, step): continue

        update_status(f"Mixing Well {tray_idx}", progress, tray_idx)
        state_mgr.update(order_id, "stick", tray_idx, None, step)
        if not err_handler.check_and_recover(): return current_step

        gripper_control("init")
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)
        movel(GRAB, vel=VEL_WORK, acc=ACC)
        gripper_control("squeeze")
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)

        p_tray = get_tray_pose(TRAY_BASE, tray_idx)
        p_tray_up = posx([p_tray[0], p_tray[1], TRAY_UP_Z, p_tray[3], p_tray[4], p_tray[5]])
        p_tray_down = posx([p_tray[0], p_tray[1], TRAY_DOWN_Z, p_tray[3], p_tray[4], p_tray[5]])

        stir_poses_tray = [make_stir_pose(p, p_tray_down, TRAY_BASE) for p in STIR_POSES_BASE]

        movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
        movel(p_tray_down, vel=VEL_WORK, acc=ACC)

        for _ in range(3):
            for i, p in enumerate(stir_poses_tray):
                rad = 0 if i == len(stir_poses_tray) - 1 else 10
                movel(p, vel=VEL_WORK, acc=ACC, radius=rad)

        movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
        movel(DROP, vel=VEL_MOVE, acc=ACC)
        movel(posx([DROP[0], DROP[1], DROP[2] - 158, DROP[3], DROP[4], DROP[5]]), vel=VEL_WORK, acc=ACC)

        gripper_control("init")
        movel(DROP, vel=VEL_MOVE, acc=ACC)

    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
    return current_step

def execute_tray(library, order_id, state_mgr, err_handler):
    from DSR_ROBOT2 import posx, posj, movel, movej
    
    step = TaskStep.TRAY_DROP
    if state_mgr.should_skip(order_id, "tray", None, None, step): return

    update_status("Transferring Tray to Output", 95)
    state_mgr.update(order_id, "tray", None, None, step)
    if not err_handler.check_and_recover(): return

    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["tray_out"]

    READY_1 = posx(get_pose_value(stick_poses, "ready_1"))
    READY_2 = posx(get_pose_value(stick_poses, "ready_2"))
    GRAB = posx(get_pose_value(stick_poses, "grab"))
    DROP = posx(get_pose_value(stick_poses, "drop"))
    FINISH = posx(get_pose_value(stick_poses, "finished"))

    gripper_control("init")
    movel(READY_1, vel=VEL_MOVE, acc=ACC)
    movel(READY_2, vel=VEL_MOVE, acc=ACC)
    movel(GRAB, vel=VEL_MOVE, acc=ACC)
    gripper_control("hold")

    movel(posx(GRAB[0:2] + [GRAB[2] + 100] + GRAB[3:6]), vel=VEL_MOVE, acc=ACC)
    movel(DROP, vel=VEL_MOVE, acc=ACC)
    gripper_control("init")

    movel(FINISH, vel=VEL_MOVE, acc=ACC)
    movel(posx(FINISH[0:2] + [FINISH[2] + 80] + FINISH[3:6]), vel=VEL_MOVE, acc=ACC)
    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)

# ==========================================
# 5. 재료별 레시피 생성 헬퍼
# ==========================================
def create_dynamic_recipe_for_ingredient(doe_matrix, target_code, yaml_key):
    trays_config = {}
    has_work = False
    is_liquid = target_code.startswith('l')
    
    for well in doe_matrix:
        w_id = well['well_id']
        counts = well.get('counts', {})
        count = counts.get(target_code, 0)
        
        if count > 0:
            has_work = True
            trays_config[w_id] = {
                "count": { 
                    "liquid": count if is_liquid else 0,
                    "powder": count if not is_liquid else 0
                }
            }
            
    if not has_work: return None

    return {
        "selection": { 
            "liquid": yaml_key if is_liquid else None, 
            "powder": yaml_key if not is_liquid else None 
        },
        "trays": trays_config
    }

# ==========================================
# 6. 메인 함수
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    # 노드 2개 생성 (Action용, ErrorHandler Service Call용)
    node = rclpy.create_node("beauro_executor", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    
    # 1. Firebase 접속
    print("🔑 Initializing Firebase...")
    try:
        cred = credentials.Certificate(FIREBASE_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': FIREBASE_DB_URL})
        print("✅ Connected.")
    except Exception as e:
        print(f"❌ Init Error: {e}"); return

    # 2. 상태 관리자 로드 (Firebase 연결 후)
    state_mgr = TaskStateManager()
    has_saved_state = state_mgr.load()
    
    err_handler = RobotErrorHandler(node, state_mgr)

    # 3. 로봇 초기화
    initialize_robot()

    library = load_yaml(LIBRARY_YAML_PATH)
    if not library: return

    ref_order = db.reference('current_order')

    print("\n✅ Executor Ready. Waiting for Web Order...")

    try:
        while rclpy.ok():
            order = ref_order.get()
            
            if order and order.get('status') == 'start':
                order_id = order.get('order_id')
                print(f"\n🔔 Processing Order: {order_id}")
                
                # 주문 상태 변경
                ref_order.update({'status': 'processing'})
                
                # [재개 로직] 이전 작업과 동일한 주문이면 이어하기
                if state_mgr.state["order_id"] == order_id and has_saved_state:
                    print("🔄 Resuming previous task...")
                else:
                    state_mgr.clear()
                    state_mgr.update(order_id, None, None, None, TaskStep.LIQUID_INIT)
                    update_status("Initializing...", 0)

                doe_matrix = order.get('doe_matrix', [])
                
                # 전체 스텝 수 계산
                total_ops = 0
                for well in doe_matrix:
                    counts = well.get('counts', {})
                    total_ops += sum(counts.values()) + 1
                total_steps = total_ops + 10
                current_step = 0

                # 1. Phase 1: Dispensing
                
                # L1
                recipe_l1 = create_dynamic_recipe_for_ingredient(doe_matrix, 'l1', 'liquid_A')
                if recipe_l1:
                    current_step = execute_liquid(library, recipe_l1, current_step, total_steps, order_id, state_mgr, err_handler)
                    
                # L2
                recipe_l2 = create_dynamic_recipe_for_ingredient(doe_matrix, 'l2', 'liquid_B')
                if recipe_l2:
                    current_step = execute_liquid(library, recipe_l2, current_step, total_steps, order_id, state_mgr, err_handler)
                    
                # P1
                recipe_p1 = create_dynamic_recipe_for_ingredient(doe_matrix, 'p1', 'powder_A')
                if recipe_p1:
                    current_step = execute_powder(library, recipe_p1, current_step, total_steps, order_id, state_mgr, err_handler)
                    
                # P2
                recipe_p2 = create_dynamic_recipe_for_ingredient(doe_matrix, 'p2', 'powder_B')
                if recipe_p2:
                    current_step = execute_powder(library, recipe_p2, current_step, total_steps, order_id, state_mgr, err_handler)

                # Phase 1 Done (메시지 주의: Completed 제거)
                print(">> Dispensing Phase Done (100%)")
                update_status("Dispensing Done. Switch to Mixer...", 100, 0, 2)
                time.sleep(3.0) 

                # 2. Phase 2: Mixing
                mix_trays = {}
                for well in doe_matrix:
                    c = well.get('counts', {})
                    if sum(c.values()) > 0:
                        mix_trays[well['well_id']] = {"count": {"liquid": 1, "powder": 1}}

                mix_recipe = {"trays": mix_trays}
                total_mix_steps = len(mix_trays) + 2
                current_mix_step = 0
                
                update_status("Starting Mixing Process...", 0, 0, 2)

                # Stick Mixing
                current_mix_step = execute_sticks(library, mix_recipe, current_mix_step, total_mix_steps, order_id, state_mgr, err_handler)
                
                # Tray Output
                execute_tray(library, order_id, state_mgr, err_handler)

                # All Done (메시지 주의: Output 단어 필수)
                print(">> All Actions Done. Sending 100%...")
                update_status("Output Process Completed.", 100, 0, 2)
                
                time.sleep(2.0)
                ref_order.update({'status': 'finished'})
                print("✨ Order Status -> finished")
                
                state_mgr.clear() # 작업 완료 후 상태 초기화
                
                time.sleep(1.0)
                update_status("Ready", 0, 0, 1)
                print("🤖 Robot State -> Idle")

            else:
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n[STOP]")
        state_mgr.save() # 강제 종료 시 저장
    except Exception as e:
        print(f"\n[CRITICAL ERROR] {e}")
        state_mgr.save()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
