from yaml_loader import load_yaml, build_pose_set

HOME_POSE = [0, 0, 90, 0, 90, 0]

def convert_library(library_yaml):
    lib = {
        "liquids": {},
        "powders": {},
        "mixing": {}
    }

    for key, item in library_yaml.get("liquids", {}).items():
        lib["liquids"][key] = {
            "display_name": item.get("display_name", key),
            "poses": build_pose_set(item.get("poses", {}))
        }

    for key, item in library_yaml.get("powders", {}).items():
        lib["powders"][key] = {
            "display_name": item.get("display_name", key),
            "poses": build_pose_set(item.get("poses", {}))
        }

    for key, item in library_yaml.get("mixing", {}).items():
        lib["mixing"][key] = {
            "poses": build_pose_set(item.get("poses", {}))
        }

    return lib


library_yaml = load_yaml("./src/robot_control/robot_control/material_library.yaml")
recipe_yaml = load_yaml("./src/robot_control/robot_control/recipe.yaml")


def execute_powder():
    print("="*25)
    print("[INFO] 파우더 동작 시작")
    print("="*25)

    menu_type = recipe_yaml["menu_type"]["display_name"]
    powder_key = recipe_yaml["selection"]["powder"]
    powder = library_yaml["powders"][powder_key]

    count = recipe_yaml["count"]["powder"]

    x_g, y_g, z_g, rx_g, ry_g, rz_g = powder["poses"]["grab"]["value"]
    x_b, y_b, z_b, rx_b, ry_b, rz_b = powder["poses"]["bowl"]["value"]
    x_t, y_t, z_t, rx_t, ry_t, rz_t = powder["poses"]["tray"]["value"]

    print("[INFO] 그리퍼 열기")

    print("[INFO] 스푼 접근->파지->빼기")

    print("="*25)
    print("[INFO] 사용자 요청 내역:")
    print(f"- 제품 종류: {menu_type}")
    print(f"- 파우더 종류 {powder_key}")
    print(f"- {count}회 반복")
    print("="*25)

    for i in range(count):
        print("[INFO] 보울로 이동")
        print("[INFO] 스쿠핑 준비 -> 스쿠핑 -> 털기")
        print(f"[INFO] {i+1}번 트레이로 이동 -> 파우더 붓기")
    
    print("[INFO] 트레이 동작 완료")
    print("[INFO] 스푼 거치대 접근")
    print("[INFO] 스푼 놓기")

    print("="*25)
    print("[INFO] 파우더 동작 완료")
    print("="*25)


def main():
    execute_powder()


if __name__ == "__main__":
    main()