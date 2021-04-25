import os
import numpy as np
import open3d as o3d

def read_image_o3d(filename):
    assert os.path.exists(filename)
    image = o3d.io.read_image(filename)
    assert image is not None, f"Cannot read image from {filename}"
    return image

def read_camera_intrinsics(filename):
    K = np.loadtxt(filename)
    assert K is not None, f"Cannot read camera intrinsics from {filename}"
    return K[0, 0], K[1, 1], K[0, 2], K[1, 2]

def read_camera_pose(filename):
    pose = np.loadtxt(filename)
    assert pose is not None, f"Cannot read camera pose from {filename}"
    return pose

def read_camera_params(filename):
    camera_params = dict()
    with open(filename) as f:
        for line in f:
            (key, val) = line.split()
            camera_params[key] = val
    assert camera_params is not None, f"Cannot read camera params from {filename}"
    return camera_params    

def read_pcd_dict(filename):
    assert os.path.exists(filename), f"File {filename} not found"
    data = np.load(filename, allow_pickle=True)
    pcds_names = data["name"]
    pcds_points = data["xyz"]
    pcds_poses = data["pose"]
    valid_pairs = data["pairs"]
    return pcds_names, pcds_points, pcds_poses, valid_pairs