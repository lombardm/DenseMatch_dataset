import numpy as np

def compute_rotation_distance(source_rot, target_rot):
    eps=1e-16
    rot_dist = np.arccos(
        np.clip((np.trace(source_rot.T @ target_rot) - 1) / 2, -1 + eps, 1 - eps))
    rot_dist = np.rad2deg(rot_dist)
    return rot_dist

def compute_translation_distance(source_transl, target_transl):
    transl_dist = np.linalg.norm(source_transl - target_transl)
    return transl_dist

def compute_transform_distance(source_T, target_T):
    rot_dist = compute_rotation_distance(source_T[:3,:3], target_T[:3,:3])
    transl_dist = compute_translation_distance(source_T[:3,3], target_T[:3,3])
    return rot_dist, transl_dist