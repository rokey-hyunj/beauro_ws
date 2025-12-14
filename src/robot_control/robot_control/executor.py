import rclpy
import DR_init
import yaml
import time

# ==========================================
# 1. 설정 및 상수 (Configuration)
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 속도 설정
VEL_MOVE = 250
VEL_WORK = 150   # 작업 시 천천히
ACC = 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 트레이 설정 (2x3)
TRAY_PITCH_X = 57.0  # 구멍 간격 (mm) - [수정됨: 이전 대화 반영]
TRAY_PITCH_Y = 38.0  
TRAY_Z_OFFSET = 0.0  # 트레이 기준 높이 보정

# ==========================================
# 2. 공통 유틸리티 함수 (Utility)
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
    
    # base_pose가 list 형태인지 확인
    x, y, z, rx, ry, rz = base_pose[:6]

    # 인덱스 0부터 시작하도록 조정 (1번 -> 0,0)
    idx = tray_idx - 1
    row = idx // 2
    col = idx % 2

    # 좌표 계산 (팀장님 코드 로직 + Pitch 보정)
    # X축은 증가(+), Y축은 감소(-) 방향 가정 (현장 상황에 맞춰 부호 확인!)
    xt = x + (col * TRAY_PITCH_X)
    yt = y - (row * TRAY_PITCH_Y)
    
    return posx([xt, yt, z + TRAY_Z_OFFSET, rx, ry, rz])

def gripper_control(mode):
    """그리퍼 통합 제어"""
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    
    if mode == "init":      # 초기화
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
    elif mode == "squeeze": # 짜기 (배출/공기빼기)
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(1.0)
    elif mode == "hold":    # 잡기/흡입
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.5)

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    print(">>> Robot Initialized")

# ==========================================
# 3. 액체 작업 (Liquid Task - v2.0 Logic 적용)
# ==========================================
def execute_liquid(library, recipe):
    from DSR_ROBOT2 import posx, posj, movel, movej

    print("\n[Start] Liquid Process")
    
    # 1. YAML 데이터 파싱
    liquid_key = recipe["selection"]["liquid"]
    liq_data = library["liquids"][liquid_key]["poses"]
    
    # 좌표 가져오기 (List 형태라고 가정)
    p_grab_up = posx(liq_data["grab_up"]["value"])
    p_grab    = posx(liq_data["grab"]["value"])
    p_cup_up  = posx(liq_data["cup_up"]["value"])
    p_cup_down = posx(liq_data["cup_down"]["value"])
    
    # 트레이 기준점 (YAML에 'tray_base'가 있다고 가정하거나 첫번째 좌표 사용)
    # 만약 YAML 구조가 list라면 첫번째 값을 기준점으로 사용
    try:
        tray_base_raw = liq_data["trays"]["values"][0] 
    except:
        tray_base_raw = liq_data["trays"]["value"] # 단일 값일 경우

    # 초기화
    movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC) # HOME
    gripper_control("init")

    # [스포이드 집기]
    print(f"Picking Spoid: {liquid_key}")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("hold")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)

    # [작업 루프]
    trays = recipe["trays"] # Dictionary 형태 가정
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"]["liquid"]
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        print(f">> Processing Tray #{tray_idx} (Count: {count})")
        
        # 타겟 좌표 계산
        p_tray = get_tray_pose(tray_base_raw, tray_idx)
        p_tray_down = posx(list(p_tray))
        p_tray_down[2] -= 50.0 # 깊이 조정

        for c in range(count):
            # 1) 흡입 (Suction) - v2.0 로직
            print(f"  Loop {c+1}: Suction")
            movel(p_cup_up, vel=VEL_MOVE, acc=ACC)
            gripper_control("squeeze") # 들어가기 전에 짬 (공기 배출)
            movel(p_cup_down, vel=VEL_WORK, acc=ACC)
            gripper_control("hold")    # 흡입
            movel(p_cup_up, vel=VEL_MOVE, acc=ACC)

            # 2) 배출 (Dispense)
            print(f"  Loop {c+1}: Dispense")
            movel(p_tray, vel=VEL_MOVE, acc=ACC)
            movel(p_tray_down, vel=VEL_WORK, acc=ACC)
            gripper_control("squeeze") # 1차 배출
            gripper_control("hold")    # 잔여물 털기
            gripper_control("squeeze") # 2차 배출
            gripper_control("hold")    # 원복
            movel(p_tray, vel=VEL_MOVE, acc=ACC)

    # [스포이드 정리]
    print("Returning Spoid")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("init")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)

# ==========================================
# 4. 분말 작업 (Powder Task - 스쿠핑 개선)
# ==========================================
def execute_powder(library, recipe):
    from DSR_ROBOT2 import posx, posj, movel, movej

    print("\n[Start] Powder Process")

    powder_key = recipe["selection"]["powder"]
    pow_data = library["powders"][powder_key]["poses"]
    
    p_grab = posx(pow_data["grab"]["value"])
    xg, yg, zg, rxg, ryg, rzg = p_grab

    p_bowl = posx(pow_data["bowl"]["value"])
    p_scoop = posj(pow_data["scoop"]["value"])
    p_flat = posx(pow_data["flat"]["value"])
    
    # [수정] YAML에서 새로 받은 트레이 좌표를 로드
    p_tray_x_list = pow_data["trays"]["posx"]
    p_tray_j_list = pow_data["trays"]["posj"]

    gripper_control("init")

    # [스푼 집기]
    print(f"Picking Spoon: {powder_key}")
    movel(posx([xg, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("squeeze") # 스푼을 꽉 잡음 (Squeeze 모드 사용 가정)
    # 왼쪽으로 살짝 빼기
    movel(posx([xg-80, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(posx([xg-80, yg, zg+90, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)

    trays = recipe["trays"]
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"]["powder"]
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        print(f">> Processing Tray #{tray_idx} (Count: {count})")
        
        # [수정] 트레이 좌표를 리스트에서 직접 가져와 posx 객체로 변환
        # tray_idx는 1부터 시작하므로 인덱스는 tray_idx - 1
        p_tray_raw = p_tray_x_list[tray_idx - 1]
        p_tray = posx(p_tray_raw)
        
        p_tray_j_raw = p_tray_j_list[tray_idx - 1]
        p_tray_j_target = posj(p_tray_j_raw)

        for c in range(count):
            # 1) 스쿠핑 (Scooping) - 개선된 동작
            # Bowl 위로 이동
            print("[INFO] 보울로 이동")
            movel(p_bowl, vel=VEL_MOVE, acc=ACC) 
            
            # [동작] 내려가서 -> 앞으로 긁으며 -> 올리기 (J-Shape)
            print("[INFO] 스쿠핑 준비")
            movej(p_scoop, vel=VEL_WORK, acc=ACC)
            
            # 긁기/회전 동작
            print("[INFO] 스쿠핑")
            s1, s2, s3, s4, s5, s6 = pow_data["scoop"]["value"]
            movej(posj([s1, s2, s3, s4, s5, s6+90]), vel=VEL_MOVE, acc=ACC)
            
            # 들어올리기 (Up)
            print("[INFO] 스푼 들어올리기")
            movel(p_flat, vel=VEL_MOVE, acc=ACC)

            # 평탄화 및 털기
            print("[INFO] 평탄하게 털기")
            flatten_and_shake(p_flat)

            # 2) 붓기 (Pouring)
            print(f"{c+1}번째 붓기")
            
            # 붓기 시작 위치 (Joint Target)로 이동
            xf, yf, zf, rxf, ryf, rzf = list(p_flat) 
            # 트레이 위치 (p_tray_raw)
            xt, yt, zt, rxt, ryt, rzt = p_tray_raw
            
            # 1. 트레이 위 안전 지점 계산 (p_flat의 방향 유지)
            p_tray_approach_safe = posx([xt, yt, zt + 80, rxf, ryf, rzf]) 
            
            print("[INFO] 안전 이송 포즈로 이동 (J6 회전 없이)")
            movel(p_tray_approach_safe, vel=VEL_MOVE, acc=ACC)

            print("[INFO] 붓기 준비 포즈로 이동")
            movej(p_tray_j_target, vel=VEL_MOVE, acc=ACC)
            
            # 6번 조인트 회전으로 붓기 시작
            j1, j2, j3, j4, j5, j6 = p_tray_j_raw
            p_pour_j = posj([j1, j2, j3, j4, j5, j6-90])
            print("[INFO] 붓기")
            movej(p_pour_j, vel=VEL_WORK, acc=ACC)
            
            # 털기 (Shake)
            print("[INFO] 잔여물 털기")
            for _ in range(3):
                movej(posj([j1, j2, j3, j4, j5, j6-90 + 5.0]), vel=VEL_WORK, acc=ACC)
                movej(posj([j1, j2, j3, j4, j5, j6-90 - 5.0]), vel=VEL_WORK, acc=ACC)
            
            movel(p_tray_j_target, vel=VEL_MOVE, acc=ACC) # 복귀

    # [스푼 정리]
    print("[INFO] 스푼 정리")
    movel(posx([xg-60, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(posx([xg-60, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("init")
    movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC) # HOME

def flatten_and_shake(center_pose):
    """파우더 평탄화 (좌우 흔들기)"""
    from DSR_ROBOT2 import movel, posx
    x, y, z, rx, ry, rz = center_pose
    shake_width = 5.0 # mm
    
    # 쉐이킹 동작 (빠르게)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    for _ in range(5):
        movel(posx([x+shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
        movel(posx([x-shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)

def rel_move(delta_list, ref_pos):
    """상대 이동 헬퍼 함수"""
    from DSR_ROBOT2 import movel, posx
    # 현재 위치에서 delta만큼 이동 (간단 구현)
    target = [ref_pos[i] + delta_list[i] for i in range(6)]
    movel(posx(target), vel=100, acc=100)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("recipe_integration", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()

        # 경로 수정 필요 (실제 로봇 PC 경로로 변경하세요)
        yaml_path_lib = "/home/hyunjong/beauro_ws/src/robot_control/robot_control/material_library.yaml"
        yaml_path_recipe = "/home/hyunjong/beauro_ws/src/robot_control/robot_control/recipe.yaml"
        
        library = load_yaml(yaml_path_lib)
        recipe = load_yaml(yaml_path_recipe)

        if library and recipe:
            # execute_liquid(library, recipe)
            execute_powder(library, recipe)
            print("\n[SUCCESS] All recipes execution finished.")
        else:
            print("[ERROR] Failed to load YAML files.")

    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()