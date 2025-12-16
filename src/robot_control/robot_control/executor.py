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
VEL_MOVE = 1000
VEL_WORK = 150   # 작업 시 천천히
ACC = 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 트레이 설정 (2x3)
TRAY_PITCH_X = 57.0  # 구멍 간격 (mm)
TRAY_PITCH_Y = 38.0

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
    shake_width = 5.0 # mm
    
    # 쉐이킹 동작 (빠르게)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    for _ in range(5):
        movel(posx([x+shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
        movel(posx([x-shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)

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
        # p_tray_down = posx(list(p_tray))
        # p_tray_down[2] -= 50.0 # 깊이 조정
        p_tray_up = posx(list(p_tray))
        p_tray_up[2] += 50.0 # 깊이 조정
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
            movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
            movel(p_tray, vel=VEL_WORK, acc=ACC)
            gripper_control("squeeze") # 1차 배출
            gripper_control("hold")    # 잔여물 털기
            gripper_control("squeeze") # 2차 배출
            gripper_control("hold")    # 원복
            movel(p_tray_up, vel=VEL_MOVE, acc=ACC)

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
    
    p_grab = posx(pow_data["grab"]["posx"])
    xg, yg, zg, rxg, ryg, rzg = p_grab

    p_bowl = posx(pow_data["bowl"]["posx"])
    p_scoop_1 = posj(pow_data["scoop_1"]["posj"])
    p_scoop_2 = posj(pow_data["scoop_2"]["posj"])
    p_scoop_3 = posj(pow_data["scoop_3"]["posj"])
    p_flat = posx(pow_data["flat"]["posx"])
    
    p_tray_base = pow_data["tray_base"]["posx"]
    p_pour_list = pow_data["pour"]["posj"]

    gripper_control("init")

    # [스푼 집기]
    print(f"Picking Spoon: {powder_key}")
    movel(posx([xg, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("squeeze")

    # 스푼 들기 전에 좌/우로 살짝 빼기
    if powder_key == "powder_A":
        spoon_shift = -40
    else:
        spoon_shift = 40
        
    movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(posx([xg + spoon_shift, yg, zg+110, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)

    trays = recipe["trays"]
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"]["powder"]
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        print(f">> Processing Tray #{tray_idx} (Count: {count})")

        p_tray = get_tray_pose(p_tray_base, tray_idx)
        p_pour = posj(p_pour_list[tray_idx - 1])

        for c in range(count):
            print("[INFO] 보울로 이동")
            movel(p_bowl, vel=VEL_MOVE, acc=ACC) 
            
            print("[INFO] 스쿠핑 준비")
            movej(p_scoop_1, vel=VEL_WORK, acc=ACC)
            
            # 긁기/회전 동작
            print("[INFO] 스쿠핑")
            movej(p_scoop_2, vel=VEL_WORK, acc=ACC)
            movej(p_scoop_3, vel=VEL_WORK, acc=ACC)
            
            # 들어올리기 (Up)
            print("[INFO] 스푼 들어올리기")
            movel(p_flat, vel=VEL_MOVE, acc=ACC)

            # 평탄화 및 털기
            print("[INFO] 평탄하게 털기")
            flatten_and_shake(p_flat)

            # 2) 붓기 (Pouring)
            print(f"{c+1}번째 붓기")
            
            print("[INFO] 안전 이송 포즈로 이동 (J6 회전 없이)")
            movel(p_tray, vel=VEL_MOVE, acc=ACC)

            print("[INFO] 붓기 준비 포즈로 이동")
            movej(p_pour, vel=VEL_MOVE, acc=ACC)
            
            # 6번 조인트 회전으로 붓기 시작
            if powder_key == "powder_A":
                POUR_ANGLE = -90
            else:
                POUR_ANGLE = 90

            j1, j2, j3, j4, j5, j6 = p_pour
            p_pour_j = posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE])
            print("[INFO] 붓기")
            movej(p_pour_j, vel=VEL_WORK, acc=ACC)
            
            # 털기 (Shake)
            print("[INFO] 잔여물 털기")
            for _ in range(3):
                movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE + 5.0]), vel=VEL_WORK, acc=ACC)
                movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE - 5.0]), vel=VEL_WORK, acc=ACC)
            
            movel(p_tray, vel=VEL_MOVE, acc=ACC) # 복귀

    # [스푼 정리]
    print("[INFO] 스푼 정리")
    movel(posx([xg + spoon_shift, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("init")
    movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC) # HOME

def execute_sticks(library, recipe):
    from DSR_ROBOT2 import posx, posj, movel, movej

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

        movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
        movel(p_tray_down, vel=VEL_WORK, acc=ACC)

        for _ in range(3):
            for i, p in enumerate(stir_poses_tray):
                if i == len(stir_poses_tray) - 1:
                    movel(p, vel=VEL_WORK, acc=ACC, radius=0)
                else:
                    movel(p, vel=VEL_WORK, acc=ACC, radius=10)

        movel(p_tray_up, vel=VEL_MOVE, acc=ACC)

        movel(DROP, vel=VEL_MOVE, acc=ACC)
        movel(posx([
            DROP[0], DROP[1], DROP[2] - 158,
            DROP[3], DROP[4], DROP[5]
        ]), vel=VEL_WORK, acc=ACC)

        gripper_control("init")
        movel(DROP, vel=VEL_MOVE, acc=ACC)

    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)

def execute_tray(library):
    from DSR_ROBOT2 import posx, posj, movel, movej

    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["tray_out"]

    READY_1 = posx(stick_poses["ready_1"]["posx"])
    READY_2 = posx(stick_poses["ready_2"]["posx"])
    GRAB = posx(stick_poses["grab"]["posx"])
    DROP = posx(stick_poses["drop"]["posx"])
    FINISH = posx(stick_poses["finished"]["posx"])

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


def main(args=None):
    import os

    rclpy.init(args=args)
    node = rclpy.create_node("recipe_integration", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()

        BASE_DIR = os.path.dirname(os.path.abspath(__file__))

        library = load_yaml(os.path.join(BASE_DIR, "material_library.yaml"))
        recipe = load_yaml(os.path.join(BASE_DIR, "recipe.yaml"))

        if library and recipe:
            execute_liquid(library, recipe)
            execute_powder(library, recipe)
            execute_sticks(library, recipe)
            execute_tray(library)
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