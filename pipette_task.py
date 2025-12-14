import rclpy
import DR_init
import sys

# ==========================================
# 1. 설정 및 상수 (Configuration)
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 속도 설정
VEL_MOVE = 80       # 일반 이동
VEL_APPRO = 50      # 접근 (천천히)
VEL_WORK = 30       # 작업 (신중하게)
ACC = 80

# --- [핵심] 트레이 좌표 설정 (이것만 수정하면 됨) ---
# 1번 구멍(좌측 상단)의 좌표를 기준점으로 잡습니다.
TARGET_BASE_POS = [518.207, -176.499, 483.024, 120.842, 179.159, -153.782]
TARGET_Z_DOWN_OFFSET = 50.0  # 타겟 진입 시 내려갈 깊이 (mm)

# 트레이 구멍 간격 (mm) - 자로 잰 실제 간격을 입력하세요.
# (보내주신 좌표를 역산해보니 X는 약 57mm, Y는 약 38mm 차이가 납니다)
PITCH_X = 57.0  
PITCH_Y = 38.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    print(">>> Robot Initialized.")

def perform_task():
    from DSR_ROBOT2 import movej, movel, posj, posx, set_digital_output, ON, OFF, wait

    # --------------------------------------
    # 2. 좌표 정의 (Coordinates)
    # --------------------------------------
    HOME_POSE = [0, 0, 90, 0, 90, 0]

    # [스포이드 집는 위치]
    # (참고: 필요시 좌표값 미세 조정)
    spoid_pick = {
        1: (posj([-53.89, -7.94, 97.70, -0.02, 91.03, 30.54]),   # Approach Joint
            posx([185.04, -246.28, 371.25, 124.25, 179.08, -151.0])), # Grip Pose
        2: (posj([-59.00, -2.77, 93.68, -0.02, 89.37, 31.17]), 
            posx([182.36, -295.15, 375.22, 121.08, 179.61, -148.4])),
    }
    
    # 스포이드 들어올리는 높이 (상대값 이용)
    LIFT_HEIGHT = 250.0 

    # [용액 위치]
    liquid_pose = {
        1: posx([583.78, 212.38, 463.13, 140.82, 179.45, -134.16]),
        2: posx([565.91, -21.93, 492.26, 125.26, 179.22, -149.30]),
    }
    LIQUID_DOWN_OFFSET = 85.0 # 용액 안으로 들어가는 깊이

    # --------------------------------------
    # 3. 계산 및 동작 함수 (Logic)
    # --------------------------------------
    def get_target_pose(num):
        """
        1~6번 번호를 입력받아 기준점(TARGET_BASE)으로부터 
        계산된 좌표를 반환합니다. (2x3 트레이 가정)
        순서: 
        1 2
        3 4
        5 6
        """
        # 행(row)과 열(col) 인덱스 계산 (0부터 시작)
        # num 1 -> row 0, col 0
        # num 2 -> row 0, col 1
        # num 3 -> row 1, col 0 ...
        row = (num - 1) // 2 
        col = (num - 1) % 2

        # 기준 좌표 복사
        target_x = list(TARGET_BASE_POS)
        
        # 좌표 계산: X축은 col만큼, Y축은 row만큼 이동
        # (주의: 로봇 좌표계 방향에 따라 +, - 부호 확인 필요)
        # 보내주신 데이터상 X는 증가(+), Y는 감소(-)하는 방향임
        target_x[0] += (col * PITCH_X)  
        target_x[1] -= (row * PITCH_Y) 

        return posx(target_x)

    def gripper_control(action):
        """그리퍼(스포이드) 제어 통합 함수"""
        if action == "init":     # 초기화
            set_digital_output(1, OFF)
            set_digital_output(2, OFF)
        elif action == "squeeze": # 짜기 (배출/공기빼기)
            set_digital_output(1, ON)
            set_digital_output(2, ON)
            wait(1.0)
        elif action == "hold":    # 잡기/흡입 (펴짐)
            set_digital_output(1, OFF)
            set_digital_output(2, ON)
            wait(1.5) # 액체가 빨려올라오는 시간 대기

    # --------------------------------------
    # 4. 메인 실행 로직
    # --------------------------------------
    
    # 사용자 입력
    try:
        repeat_count = int(input("반복 횟수 (Repeat): "))
        liquid_type = int(input("용액 선택 (1 or 2): "))
        target_num = int(input("목표 트레이 번호 (1~6): "))
    except ValueError:
        print("숫자만 입력해주세요.")
        return

    spoid_num = liquid_type
    
    # 좌표 준비
    liq_pos_up = liquid_pose[liquid_type]
    liq_pos_down = posx(list(liq_pos_up)) # 복사해서 사용
    liq_pos_down[2] -= LIQUID_DOWN_OFFSET # Z축 내리기

    tgt_pos_up = get_target_pose(target_num)
    tgt_pos_down = posx(list(tgt_pos_up))
    tgt_pos_down[2] -= TARGET_Z_DOWN_OFFSET

    spoid_j, spoid_x = spoid_pick[spoid_num]
    spoid_up_x = posx(list(spoid_x))
    spoid_up_x[2] += LIFT_HEIGHT # 집고 나서 올릴 높이

    print(">>> Start Task")
    
    # 1. 초기화 및 스포이드 집기
    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
    gripper_control("init")
    
    print(f"Picking Spoid #{spoid_num}")
    movej(spoid_j, vel=VEL_MOVE, acc=ACC)
    movel(spoid_x, vel=VEL_APPRO, acc=ACC)
    gripper_control("hold")
    movel(spoid_up_x, vel=VEL_MOVE, acc=ACC)

    # 2. 반복 작업 (흡입 -> 배출)
    for i in range(repeat_count):
        print(f"[Cycle {i+1}] Suction -> Dispense")

        # --- [흡입 시퀀스] ---
        # 1) 용액 위로 이동
        movel(liq_pos_up, vel=VEL_MOVE, acc=ACC) 
        # 2) 들어가기 전에 미리 짠다 (공기 빼기)
        gripper_control("squeeze") 
        # 3) 용액 안으로 하강
        movel(liq_pos_down, vel=VEL_APPRO, acc=ACC)
        # 4) 스포이드를 펴서 용액 흡입
        gripper_control("hold") 
        # 5) 상승
        movel(liq_pos_up, vel=VEL_MOVE, acc=ACC)

        # --- [배출 시퀀스] ---
        # 1) 타겟 위로 이동
        movel(tgt_pos_up, vel=VEL_MOVE, acc=ACC)
        # 2) 타겟 안으로 하강
        movel(tgt_pos_down, vel=VEL_APPRO, acc=ACC)
        # 3) 짜서 배출
        gripper_control("squeeze")
        # 4) 잔여물 털기 (살짝 폈다 다시 짜기)
        gripper_control("hold")
        wait(0.2)
        gripper_control("squeeze")
        # 5) 원복 후 상승
        gripper_control("hold")
        movel(tgt_pos_up, vel=VEL_MOVE, acc=ACC)

    # 3. 정리 (스포이드 제자리에 놓기)
    print("Returning Spoid")
    # 홈 근처 경유지 (필요시 사용, 여기서는 바로 이동)
    movel(spoid_up_x, vel=VEL_MOVE, acc=ACC)
    movel(spoid_x, vel=VEL_APPRO, acc=ACC)
    gripper_control("init") # 놓기
    movej(spoid_j, vel=VEL_MOVE, acc=ACC) # 안전하게 빠져나오기
    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
    
    print(">>> Task Finished")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("spoid_task_v2", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        perform_task()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()