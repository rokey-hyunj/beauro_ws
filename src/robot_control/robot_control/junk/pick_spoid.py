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
    from DSR_ROBOT2 import posx,posj,movej,movel,set_digital_output

    # Robot coordinates
    HOME_POSE = [0, 0, 90, 0, 90, 0]
    spoid_up = posj([-18.784, 17.081, 43.587, -0.374, 119.113, -18.864])
    spoid_down = posx([416.898, -141.854, 386.5, 139.822, -179.328, 140.521])

    print("[INFO] Returning HOME...")
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)
    print("[INFO] Opening grip...")
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    print("[INFO] Move-J towards spoid...")
    movej(spoid_up, vel=VELOCITY, acc=ACC)
    print("[INFO] Move down to grab...")
    movel(spoid_down, vel=VELOCITY, acc=ACC)

    # Close grip
    print("[INFO] Closing grip...")
    set_digital_output(1, 1)
    set_digital_output(2, 0)

    print("[INFO] Move-L lifting...")
    movel(spoid_up, vel=VELOCITY, acc=ACC)


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("pick_spoid", namespace=ROBOT_ID)

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