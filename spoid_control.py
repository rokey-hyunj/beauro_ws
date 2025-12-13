import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
# VELOCITY = 100  # simulation
VELOCITY = 80   # real
VELOCITY_DOWN = 50
VELOCITY_UP = 40
ACC = 80

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
    from DSR_ROBOT2 import movej,movel, posj, posx, set_digital_output, ON, OFF, wait
    import time

    # Robot coordinates
    HOME_POSE = [0, 0, 90, 0, 90, 0]
    spoid_up_point1 = posj([-41.548, -21.444, 109.13, -0.123, 91.77, -41.51])
    spoid_up_point2 = posj([-49.776, -16.565, 103.889, -0.111, 91.374, -49.591])
    spoid_grip_point1 = posx([165.433, -140.756, 376.139, 137.212, -179.455, 137.537])
    spoid_grip_point2 = posx([165.383, -189.399, 374.178, 129.204, -178.678, 129.759])
    spoid_up_from_point1 = posx([161.675, -136.09, 519.155, 137.916, -178.871, 138.068])
    spoid_up_from_point2 = posx([161.909, -185.081, 519.458, 129.287, -178.458, 129.815])
    move_to_water = posj([10.49, 12.753, 58.105, -0.055, 108.893, 10.627])
    move_down_water = posx([424.556, 84.908, 375.95, 171.682, -179.222, 171.797])
    move_up_from_water = posx([417.262, 90.463, 479.076, 174.265, -178.963, 174.121])
    move_to_target = posj([26.982, 15.211, 64.751, -0.301, 98.876, 26.359])
    move_down_to_target = posx([421.098, 219.136, 425.147, 5.452, 178.961, 5.055])
    move_up_from_target = posx([417.843, 221.07, 511.094, 2.22, 178.991, 1.68])
    
    print("Get Ready")
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)
    set_digital_output(1, ON)
    set_digital_output(2, ON)
    wait(3.0)

    print("move spoid1")
    movej(spoid_up_point1, vel=VELOCITY, acc=ACC)
    movel(spoid_grip_point1, vel=VELOCITY_DOWN, acc=ACC)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3.0)
    movel(spoid_up_from_point1, vel=VELOCITY_UP, acc=ACC)

    print("get water")
    movej(move_to_water, vel=VELOCITY, acc=ACC)
    movel(move_down_water, vel=VELOCITY, acc=ACC)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(3.0)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3.0)
    movel(move_up_from_water, vel=VELOCITY, acc=ACC)

    print("move to target")
    movej(move_to_target, vel=VELOCITY, acc=ACC)
    movel(move_down_to_target, vel=VELOCITY, acc=ACC)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(3.0)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3.0)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(3.0)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3.0)
    movel(move_up_from_target, vel=VELOCITY, acc=ACC)

    print("get back spoid1")
    movel(spoid_up_from_point1, vel=VELOCITY, acc=ACC)
    movel(spoid_grip_point1, vel=VELOCITY_DOWN, acc=ACC)
    set_digital_output(1, ON)
    set_digital_output(2, ON)
    wait(3.0)
    movel(spoid_up_from_point1, VELOCITY_UP, acc=ACC)

    print("move spoid2")
    movej(spoid_up_point2, vel=VELOCITY, acc=ACC)
    movel(spoid_grip_point2, vel=VELOCITY_DOWN, acc=ACC)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3.0)
    movel(spoid_up_from_point2, vel=VELOCITY_UP, acc=ACC)

    print("get water")
    movej(move_to_water, vel=VELOCITY, acc=ACC)
    movel(move_down_water, vel=VELOCITY, acc=ACC)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(3.0)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3.0)
    movel(move_up_from_water, vel=VELOCITY, acc=ACC)

    print("move to target")
    movej(move_to_target, vel=VELOCITY, acc=ACC)
    movel(move_down_to_target, vel=VELOCITY, acc=ACC)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(3.0)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3.0)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(3.0)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(3.0)
    movel(move_up_from_target, vel=VELOCITY, acc=ACC)

    print("get back spoid2")
    movel(spoid_up_from_point2, vel=VELOCITY, acc=ACC)
    movel(spoid_grip_point2, vel=VELOCITY_DOWN, acc=ACC)
    set_digital_output(1, ON)
    set_digital_output(2, ON)
    wait(3.0)
    movel(spoid_up_from_point2, VELOCITY_UP, acc=ACC)
    wait(3.0)
    print("Finished")
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)
    set_digital_output(1, ON)
    set_digital_output(2, ON)

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("go_home", namespace=ROBOT_ID)

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