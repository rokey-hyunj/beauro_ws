import yaml
from DSR_ROBOT2 import posx, posj

def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)

def build_pose(pose_dict):
    if pose_dict["type"] == "posx":
        return posx(pose_dict["value"])
    else:
        return posj(pose_dict["value"])

def build_pose_set(poses_yaml):
    poses = {}
    for name, pose in poses_yaml.items():
        poses[name] = build_pose(pose)
    return poses
