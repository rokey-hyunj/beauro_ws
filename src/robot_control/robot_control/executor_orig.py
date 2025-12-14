import rclpy
import DR_init
import yaml

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VEL = 100
ACC = 50

HOME_POSE = [0, 0, 90, 0, 90, 0]

TRAY_DX = 60.0   # 좌우 간격 (mm)
TRAY_DY = 40.0   # 상하 간격 (mm)

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def tray_offset(base_pose, tray_idx):
    x, y, z, rx, ry, rz = base_pose

    idx = tray_idx - 1
    col = idx % 2
    row = idx // 2

    xt = x + col * TRAY_DX
    yt = y - row * TRAY_DY

    return [xt, yt, z, rx, ry, rz]


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    print("#" * 50)
    print("Initializing robot")
    print(f"ROBOT_ID   : {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP  : {ROBOT_TCP}")
    print(f"VEL / ACC  : {VEL} / {ACC}")
    print("#" * 50)


def flatten_and_shake(center_pose, shake_angle=2.0, shake_count=4):
    from DSR_ROBOT2 import posx, movel

    x, y, z, rx, ry, _ = center_pose
    rz = 90

    base = posx([x, y, z, rx, ry, rz])
    movel(base, vel=VEL, acc=ACC)

    for _ in range(shake_count):
        movel(posx([x, y, z + 1, rx + shake_angle, ry, rz]), vel=VEL, acc=ACC)
        movel(posx([x, y, z,     rx - shake_angle, ry, rz]), vel=VEL, acc=ACC)

    movel(base, vel=VEL, acc=ACC)

def execute_liquid(library, recipe):
    from DSR_ROBOT2 import posx, posj, movel, movej, set_digital_output, wait, ON, OFF

    def gripper_init():
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

    def gripper_hold():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.5)

    def gripper_squeeze():
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(1.5)

    liquid_key = recipe["selection"]["liquid"]

    liquid = library["liquids"][liquid_key]["poses"]

    tray_poses = library["liquids"][recipe["selection"]["liquid"]]["poses"]["trays"]["values"]

    # 위치/그리퍼 초기화
    movej(HOME_POSE, vel=VEL, acc=ACC)
    gripper_init()

    print(f"[INFO] Liquid: {liquid_key}")

    print(f"[INFO] 스포이드 파지 준비")
    movel(posj(liquid["grab_up"]["value"]), vel=VEL, acc=ACC)
    wait(1.0)
    print(f"[INFO] 스포이드 파지")
    movel(posx(liquid["grab"]["value"]), vel=VEL, acc=ACC)
    wait(1.0)
    gripper_hold()
    print(f"[INFO] 스포이드 들어올리기")
    movel(posx(liquid["grab_up"]["value"]), vel=VEL, acc=ACC)
    wait(1.0)

    for tray_idx, tray_cfg in recipe["trays"].items():
        count = tray_cfg["count"]["liquid"]
        if count <= 0:
            continue

        print(f"[INFO] Tray {tray_idx} liquid {count}")

        for _ in range(count):
            # 컵으로 이동
            print(f"[INFO] 컵으로 이동")
            movel(posx(liquid["cup_up"]["value"]), vel=VEL, acc=ACC)
            wait(1.0)
            print(f"[INFO] 배출 준비")
            movel(posx(liquid["cup_down"]["value"]), vel=VEL, acc=ACC)
            wait(1.0)

            # ===== 흡입 =====
            gripper_squeeze()
            gripper_hold()

            # 이동 준비
            print(f"[INFO] 트레이로 이동 준비")
            movel(posx(liquid["cup_up"]["value"]), vel=VEL, acc=ACC)
            wait(1.0)

            # tray 이동
            print(f"[INFO] {tray_idx}번 트레이로 이동")
            xt, yt, zt, rxt, ryt, rzt = tray_poses[tray_idx-1]
            movel(posx([xt, yt, zt, rxt, ryt, rzt]), vel=VEL, acc=ACC)
            movel(posx([xt, yt, zt - 50, rxt, ryt, rzt]), vel=VEL, acc=ACC)

            # ===== 배출 =====
            print(f"[INFO] 액체 배출")
            gripper_squeeze()
            gripper_hold()
            gripper_squeeze()
            gripper_hold()

            print(f"[INFO] 배출 완료, 다음 동작 준비")
            movel(posx([xt, yt, zt, rxt, ryt, rzt]), vel=VEL, acc=ACC)

    print(f"[INFO] 스포이드 원위치 준비")
    movel(posx(liquid["grab_up"]["value"]), vel=VEL, acc=ACC)
    wait(1.0)
    print(f"[INFO] 스포이드 원위치")
    movel(posx(liquid["grab"]["value"]), vel=VEL, acc=ACC)
    wait(1.0)

    set_digital_output(1, 0)
    set_digital_output(2, 0)

    print(f"[INFO] 스포이드 동작 완료")
    movel(posx(liquid["grab_up"]["value"]), vel=VEL, acc=ACC)
    wait(1.0)


def execute_powder(library, recipe):
    from DSR_ROBOT2 import posx, movel, movej, set_digital_output, wait, OFF, ON

    def gripper_init():
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

    def gripper_squeeze():
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(1.5)

    powder_key = recipe["selection"]["powder"]
    trays = recipe["trays"]

    powder = library["powders"][powder_key]["poses"]

    grab = powder["grab"]["value"]
    bowl = powder["bowl"]["value"]
    tray_base = powder["tray"]["value"]

    xg, yg, zg, rxg, ryg, rzg = grab
    xb, yb, zb, rxb, ryb, rzb = bowl

    print(f"[INFO] Powder: {powder_key}")

    movej(HOME_POSE, vel=VEL, acc=ACC)
    # 그리퍼 초기화
    gripper_init()

    # 스푼 잡기
    print(f"[INFO] 스푼 파지 준비")
    movel(posx([xg, yg, zg + 80, rxg, ryg, rzg]), vel=VEL, acc=ACC)
    wait(1.0)
    print(f"[INFO] 스푼 파지")
    movel(posx(grab), vel=VEL, acc=ACC)
    gripper_squeeze()
    print(f"[INFO] 보울로 이동 준비")
    movel(posx([xg, yg, zg + 80, rxg, ryg, rzg]), vel=VEL, acc=ACC)
    wait(1.0)

    # 트레이 반복
    print(f"[INFO] 파우더 트레이 동작 시작")
    for tray_idx, tray_cfg in trays.items():
        count = tray_cfg["count"]["powder"]
        if count <= 0:
            continue

        tray_pose = tray_offset(tray_base, int(tray_idx))

        print(f"[INFO] Tray {tray_idx} powder {count}")

        for _ in range(count):
            # 스쿠핑
            print(f"[INFO] 보울로 이동")
            movel(posx(bowl), vel=VEL, acc=ACC)
            wait(1.0)
            print(f"[INFO] 스쿠핑 준비")
            movel(posx([xb, yb, zb - 60, rxb, ryb, rzb]), vel=VEL, acc=ACC)
            wait(1.0)

            print(f"[INFO] 스쿠핑 동작")
            flatten_and_shake(bowl)

            # 파우더 붓기
            xt, yt, zt, rxt, ryt, rzt = tray_pose
            movel(posx([xt, yt, zt + 80, rxt, ryt, rzt]), vel=VEL, acc=ACC)
            wait(1.0)
            movel(posx([xt, yt, zt - 30, rxt, ryt, rzt]), vel=VEL, acc=ACC)
            wait(1.0)
            movel(posx([xt, yt, zt + 80, rxt, ryt, rzt]), vel=VEL, acc=ACC)
            wait(1.0)

    # 스푼 놓기
    movel(posx([xg, yg, zg + 50, rxg, ryg, rzg]), vel=VEL, acc=ACC)
    wait(1.0)
    movel(posx(grab), vel=VEL, acc=ACC)
    gripper_init()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("recipe_executor", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()

        library = load_yaml(
            "/home/hyunjong/beauro_ws/src/robot_control/robot_control/material_library.yaml"
        )
        recipe = load_yaml(
            "/home/hyunjong/beauro_ws/src/robot_control/robot_control/recipe.yaml"
        )

        execute_liquid(library, recipe)

        execute_powder(library, recipe)

        print("[INFO] Recipe execution finished")

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")

    except Exception as e:
        print(f"[ERROR] {e}")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()