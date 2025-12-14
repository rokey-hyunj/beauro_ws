import rclpy
import DR_init
import yaml
import time
import threading
import firebase_admin
from firebase_admin import credentials, db

# ==========================================
# 1. 설정 및 전역 변수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# [업데이트] 속도 설정 (팀원 코드 반영)
VEL_MOVE = 250
VEL_WORK = 150
ACC = 80

# 트레이 설정
TRAY_PITCH_X = 57.0
TRAY_PITCH_Y = 38.0
TRAY_Z_OFFSET = 0.0

# Firebase 설정
FIREBASE_KEY_PATH = "/home/flashbulb/beauro-backend/serviceAccountKey.json" # 경로 확인!
FIREBASE_DB_URL = 'https://beauro-ac0ad-default-rtdb.asia-southeast1.firebasedatabase.app/'

# 상태 공유 클래스
class RobotState:
    is_working = False
    current_action = "Ready"
    progress = 0
    current_well = 0
    total_steps = 1
    current_step = 0

state = RobotState()

# ==========================================
# 2. Firebase 모니터링 스레드
# ==========================================
def firebase_monitor_thread():
    ref_status = db.reference('robot_state')
    from DSR_ROBOT2 import get_current_posx

    while True:
        try:
            status_data = {
                'is_online': True,
                'state': 2 if state.is_working else 1,
                'current_action': state.current_action,
                'progress_percent': int((state.current_step / state.total_steps) * 100) if state.total_steps > 0 else 0,
                'current_well': state.current_well,
                'timestamp': int(time.time() * 1000)
            }
            if rclpy.ok():
                try:
                    curr_x = get_current_posx()
                    status_data['tcp'] = {'x': round(curr_x[0], 2), 'y': round(curr_x[1], 2), 'z': round(curr_x[2], 2)}
                except: pass
            
            ref_status.update(status_data)
        except: pass
        time.sleep(0.5)

# ==========================================
# 3. 유틸리티 및 그리퍼
# ==========================================
def load_yaml(path):
    with open(path, "r") as f: return yaml.safe_load(f)

def update_progress(action, step_increment=0, well_id=None):
    """상태 업데이트 및 진행률 증가"""
    state.current_action = action
    if step_increment > 0: state.current_step += step_increment
    if well_id is not None: state.current_well = well_id
    print(f" -> [Action] {action}")

def gripper_control(mode):
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    if mode == "init":
        set_digital_output(1, OFF); set_digital_output(2, OFF)
    elif mode == "squeeze":
        set_digital_output(1, ON); set_digital_output(2, ON); wait(1.0)
    elif mode == "hold":
        set_digital_output(1, OFF); set_digital_output(2, ON); wait(1.5)

def get_tray_pose(base_pose, tray_idx):
    from DSR_ROBOT2 import posx
    x, y, z, rx, ry, rz = base_pose[:6]
    idx = tray_idx - 1
    row = idx // 2
    col = idx % 2
    xt = x + (col * TRAY_PITCH_X)
    yt = y - (row * TRAY_PITCH_Y)
    return posx([xt, yt, z + TRAY_Z_OFFSET, rx, ry, rz])

def flatten_and_shake(center_pose):
    """파우더 평탄화 (팀원 코드 추가)"""
    from DSR_ROBOT2 import movel, posx
    x, y, z, rx, ry, rz = center_pose
    shake_width = 5.0
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    for _ in range(5):
        movel(posx([x+shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
        movel(posx([x-shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)

# ==========================================
# 4. 핵심 공정 로직
# ==========================================

# --- [1] 액체 공정 (기존 유지) ---
def execute_liquid(library, recipe, matrix_override=None):
    from DSR_ROBOT2 import posx, posj, movel, movej
    
    update_progress("Liquid Process Started")
    liquid_key = recipe["selection"]["liquid"]
    liq_data = library["liquids"][liquid_key]["poses"]
    
    p_grab_up = posx(liq_data["grab_up"]["value"])
    p_grab    = posx(liq_data["grab"]["value"])
    p_cup_up  = posx(liq_data["cup_up"]["value"])
    p_cup_down = posx(liq_data["cup_down"]["value"])
    
    try: tray_base = liq_data["trays"]["values"][0]
    except: tray_base = liq_data["trays"]["value"]

    movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)
    gripper_control("init")

    update_progress("Picking Pipette", 5)
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("hold")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)

    # 웹 데이터 우선 사용
    target_wells = []
    if matrix_override:
        for item in matrix_override: target_wells.append(item.get('well_id'))
    else:
        for t_idx, t_cfg in recipe["trays"].items():
            if t_cfg["count"]["liquid"] > 0: target_wells.extend([int(t_idx)] * t_cfg["count"]["liquid"])

    for well_idx in target_wells:
        update_progress(f"Filling Liquid - Well {well_idx}", 10, well_idx)
        p_tray = get_tray_pose(tray_base, well_idx)
        p_tray_down = posx(list(p_tray))
        p_tray_down[2] -= 50.0

        movel(p_cup_up, vel=VEL_MOVE, acc=ACC)
        gripper_control("squeeze")
        movel(p_cup_down, vel=VEL_WORK, acc=ACC)
        gripper_control("hold")
        movel(p_cup_up, vel=VEL_MOVE, acc=ACC)

        movel(p_tray, vel=VEL_MOVE, acc=ACC)
        movel(p_tray_down, vel=VEL_WORK, acc=ACC)
        gripper_control("squeeze")
        gripper_control("hold")
        gripper_control("squeeze")
        gripper_control("hold")
        movel(p_tray, vel=VEL_MOVE, acc=ACC)

    update_progress("Returning Pipette")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("init")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)


# --- [2] 파우더 공정 (팀원 코드 통합 + Firebase 연결) ---
def execute_powder(library, recipe):
    from DSR_ROBOT2 import posx, posj, movel, movej
    
    update_progress("Powder Process Started")
    
    powder_key = recipe["selection"]["powder"]
    pow_data = library["powders"][powder_key]["poses"]
    
    p_grab = posx(pow_data["grab"]["value"])
    xg, yg, zg, rxg, ryg, rzg = p_grab

    p_bowl = posx(pow_data["bowl"]["value"])
    p_scoop = posj(pow_data["scoop"]["value"])
    p_flat = posx(pow_data["flat"]["value"])
    
    p_tray_x_list = pow_data["trays"]["posx"]
    p_tray_j_list = pow_data["trays"]["posj"]

    gripper_control("init")

    # 1. 스푼 집기
    update_progress(f"Picking Spoon: {powder_key}", 5)
    movel(posx([xg, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("squeeze") 
    movel(posx([xg-80, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(posx([xg-80, yg, zg+90, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)

    trays = recipe["trays"]
    
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"]["powder"]
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        update_progress(f"Powder to Tray #{tray_idx}", 5, tray_idx)
        
        p_tray_raw = p_tray_x_list[tray_idx - 1]
        p_tray_j_raw = p_tray_j_list[tray_idx - 1]
        p_tray_j_target = posj(p_tray_j_raw)

        for c in range(count):
            # 2. 스쿠핑 (J-Shape)
            update_progress(f"Scooping ({c+1}/{count})", 5)
            movel(p_bowl, vel=VEL_MOVE, acc=ACC) 
            movej(p_scoop, vel=VEL_WORK, acc=ACC)
            
            s1, s2, s3, s4, s5, s6 = pow_data["scoop"]["value"]
            movej(posj([s1, s2, s3, s4, s5, s6+90]), vel=VEL_MOVE, acc=ACC)
            
            movel(p_flat, vel=VEL_MOVE, acc=ACC)
            flatten_and_shake(p_flat)

            # 3. 붓기 (Pouring)
            update_progress(f"Pouring ({c+1}/{count})", 5)
            
            xf, yf, zf, rxf, ryf, rzf = list(p_flat) 
            xt, yt, zt, rxt, ryt, rzt = p_tray_raw
            
            # 안전 지점 이동
            p_tray_approach_safe = posx([xt, yt, zt + 80, rxf, ryf, rzf]) 
            movel(p_tray_approach_safe, vel=VEL_MOVE, acc=ACC)

            # 붓기 준비 포즈 (Joint)
            movej(p_tray_j_target, vel=VEL_MOVE, acc=ACC)
            
            # 붓기 (J6 회전)
            j1, j2, j3, j4, j5, j6 = p_tray_j_raw
            p_pour_j = posj([j1, j2, j3, j4, j5, j6-90])
            movej(p_pour_j, vel=VEL_WORK, acc=ACC)
            
            # 털기
            for _ in range(3):
                movej(posj([j1, j2, j3, j4, j5, j6-90 + 5.0]), vel=VEL_WORK, acc=ACC)
                movej(posj([j1, j2, j3, j4, j5, j6-90 - 5.0]), vel=VEL_WORK, acc=ACC)
            
            movel(p_tray_j_target, vel=VEL_MOVE, acc=ACC) # 복귀

    # 4. 스푼 정리
    update_progress("Returning Spoon")
    movel(posx([xg-60, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(posx([xg-60, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("init")
    movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)

# ==========================================
# 5. 메인 함수
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node("beauro_web_manager", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # [1] Firebase 초기화
    print("🔑 Initializing Firebase...")
    try:
        cred = credentials.Certificate(FIREBASE_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': FIREBASE_DB_URL})
        print("✅ Firebase Connected.")
    except Exception as e:
        print(f"❌ Firebase Init Error: {e}")
        return

    # [2] 로봇 및 설정 초기화
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    base_path = "/home/flashbulb/beauro_ws/src/beauro_project/beauro_project/"
    library = load_yaml(base_path + "material_library.yaml")
    recipe = load_yaml(base_path + "recipe.yaml")

    monitor_thread = threading.Thread(target=firebase_monitor_thread, daemon=True)
    monitor_thread.start()

    ref_order = db.reference('current_order')
    print("\n✅ System Ready. Waiting for Web Command...")

    try:
        while rclpy.ok():
            order = ref_order.get()
            if order and order.get('status') == 'start':
                print("\n🔔 New Order!")
                state.is_working = True
                state.current_action = "Initializing"
                state.progress = 0
                
                # 총 단계 수 대략 계산 (프로그레스바 용)
                state.total_steps = 100 
                
                ref_order.update({'status': 'processing'})
                matrix_data = order.get('doe_matrix', [])

                # 공정 수행
                execute_liquid(library, recipe, matrix_override=matrix_data)
                
                # 필요시 파우더 공정도 실행 (웹에서 제어하거나 항상 실행하려면 주석 해제)
                execute_powder(library, recipe)

                state.is_working = False
                state.current_action = "Finished"
                state.progress = 100
                ref_order.update({'status': 'finished'})
                print("✨ Order Completed.")
            else:
                state.current_action = "Ready"
                time.sleep(1)
    except KeyboardInterrupt:
        print("\n[STOP]")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()