import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
# VELOCITY = 100  # simulation
VELOCITY = 50   # real
ACC = 30

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool,set_tcp

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 설정된 설정값 출력
    print("#"*50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#"*50)


def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")
    from DSR_ROBOT2 import posx, posj, movel, movej, set_digital_output

    # Robot coordinates
    HOME_POSE = [0, 0, 90, 0, 90, 0]
    grab = posx([358.368, 58.814, 184.341, 93.777, -92.97, 86.539])
    grab_up = posx([343.528, 51.151, 274.059, 93.866, -93.203, 86.522])
    move_to_bowl = posx([465.306, 184.863, 417.41, 150.537, -125.051, 168.183])
    move_down = posx([462.188, 187.671, 269.296, 140.82, -133.338, 148.545])
    rotate = posj([33.591, 20.854, 103.348, -62.96, 61.1, 16.384])
    move_up = posx([409.227, 171.406, 218.761, 148.566, -98.977, 83.226])   # 수평 맞추기
    cup_up = posj([19.467, 14.651, 128.229, -99.605, 61.245, 43.261])
    pour = posj([19.468, 14.655, 128.342, -100.957, 61.739, -40.428])

    def flatten_and_shake(center_pose, shake_angle=4.0, shake_count=5):
        x, y, z, rx, ry, _ = center_pose

        fixed_rx = rx   # 필요하면 0 으로 강제해도 됨
        fixed_ry = ry   # 스푼을 지면과 수평하게
        fixed_rz = 90   # 바라보는 방향은 그대로 유지

        flat = posx([x, y, z, fixed_rx, fixed_ry, fixed_rz])

        print("[INFO] Align spoon horizontally...")
        movel(flat, vel=30, acc=30)

        print("[INFO] Shaking spoon to drop powder...")
        for i in range(shake_count):
            pose_r = posx([x, y, z, fixed_rx + shake_angle, fixed_ry, fixed_rz])
            movel(pose_r, vel=60, acc=60)

            pose_l = posx([x, y, z, fixed_rx - shake_angle, fixed_ry, fixed_rz])
            movel(pose_l, vel=60, acc=60)

        movel(flat, vel=30, acc=30)
        print("[INFO] Powder shake done.")

    # 초기 위치
    print("[INFO] Returning HOME...")
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)
    print("[INFO] Opening grip...")
    set_digital_output(1, 1)
    set_digital_output(2, 1)

    # 스푼 잡기
    print("[INFO] Move-L above spoon...")
    movel(grab_up, vel=VELOCITY, acc=ACC)
    print("[INFO] Move-L to grab...")
    movel(grab, vel=VELOCITY, acc=ACC)
    print("[INFO] Closing grip...")
    set_digital_output(1, 0)
    set_digital_output(2, 0)
    print("[INFO] Move-L above spoon...")
    movel(grab_up, vel=VELOCITY, acc=ACC)

    # 파우더 스쿠핑
    print("[INFO] Move-J above bowl...")    # 충돌 발생 (조금 위 좌표 하나 더 따기)
    movel(move_to_bowl, vel=VELOCITY, acc=ACC)
    print("[INFO] Move-L before scooping ...")
    movel(move_down, vel=VELOCITY, acc=ACC)
    print("[INFO] Move-J to rotate...")
    movej(rotate, vel=VELOCITY, acc=ACC)

    # 평평하게 만들기
    print("[INFO] Move-L for alignment...")
    movel(move_up, vel=VELOCITY, acc=ACC)
    print("[INFO] Shaking to flatten...")
    flatten_and_shake(move_up)

    # 따라내기
    print("[INFO] Move-J above cup...")
    movej(cup_up, vel=VELOCITY, acc=ACC)
    print("[INFO] Move-J to pour out...")
    movej(pour, vel=VELOCITY, acc=ACC)

    # 스푼 내려놓기
    print("[INFO] Move-L above spoon...")
    movel(grab_up, vel=VELOCITY, acc=ACC)
    print("[INFO] Move-L to grab...")
    movel(grab, vel=VELOCITY, acc=ACC)
    print("[INFO] Opening grip...")
    set_digital_output(1, 1)
    set_digital_output(2, 1)
    print("[INFO] Move-L above spoon...")
    movel(grab_up, vel=VELOCITY, acc=ACC)

    # 초기 위치로 복귀
    print("[INFO] Returning HOME...")
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("spoon_flatten", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()
        # 작업 수행
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()