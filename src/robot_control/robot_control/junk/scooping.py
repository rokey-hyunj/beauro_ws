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
ACC = 60

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
    from DSR_ROBOT2 import posx,posj,movej,movel,movec,set_digital_output

    # Robot coordinates
    HOME_POSE = [0, 0, 90, 0, 90, 0]
    spoon_up1 = posj([-19.571, 10.596, 80.013, -0.188, 89.12, -18.926])         # 스푼 잡기 전 위치
    spoon_down = posx([416.898, -141.854, 386.5, 139.822, -179.328, 140.521])
    spoon_up2 = posx([407.358, -130.353, 600.97, 137.843, -179.706, 138.161])   # 스쿠핑 전에 위로 올릴 위치
    scoop_up = posj([28.458, 18.254, 102.896, -126.347, 44.212, 47.24])
    scoop_1 = posj([18.909, 15.484, 97.19, -21.628, 18.737, -60.757])
    scoop_2 = posx([491.46, 168.075, 275.056, 6.672, 138.633, -90.893])
    scoop_3 = posx([498.045, 176.155, 333.677, 0.318, 154.135, -85.958])

    print("[INFO] Returning HOME...")
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)
    print("[INFO] Opening grip...")
    set_digital_output(1, 1)
    set_digital_output(2, 1)
    print("[INFO] Move-J towards spoon...")
    movej(spoon_up1, vel=VELOCITY, acc=ACC)
    print("[INFO] Move down to grab...")
    movel(spoon_down, vel=VELOCITY, acc=ACC)

    # Close grip
    print("[INFO] Closing grip...")
    set_digital_output(1, 0)
    set_digital_output(2, 0)

    print("[INFO] Lifting before next motion...")
    movel(spoon_up2, vel=VELOCITY, acc=ACC)
    print("[INFO] Move-J towards powder...")
    movej(scoop_up, vel=VELOCITY, acc=ACC)
    print("[INFO] Move-J before Move-C...")
    movej(scoop_1, vel=VELOCITY, acc=ACC)
    movec(scoop_2, scoop_3, time=5)


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("scooping", namespace=ROBOT_ID)

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