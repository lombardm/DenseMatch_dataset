import numpy as np
import open3d as o3d
import copy
import ioutils

def create_rgbd_image(color_file, depth_file, depth_scale=1000.0, depth_trunc=1.0, rgb2grey=False):
    color = ioutils.read_image_o3d(color_file)
    depth = ioutils.read_image_o3d(depth_file)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_scale=depth_scale, depth_trunc=depth_trunc, convert_rgb_to_intensity=rgb2grey)
    assert rgbd_image is not None, f"Cannot create rgbd image from color image: {color_file} "\
        f"and depth image: {depth_file}"
    return rgbd_image

def create_pcd_from_rgbd(rgbd, camera_intrinsic, estimate_normals=True, min_valid_points=100):
    assert rgbd is not None
    pcd = o3d.geometry.PointCloud()
    pcd = pcd.create_from_rgbd_image(rgbd, camera_intrinsic)
    assert pcd is not None, f"Can not create pointcloud from rgbd image {rgbd}"
    if len(pcd.points) > min_valid_points:
        if estimate_normals:
            pcd.estimate_normals()
    else:
        print(f"Warning: point cloud has only {len(pcd.points) } points. Set the point clouds to None")
    return pcd

def create_pcd_from_points(points):
    assert np.shape(points)[1] == 3
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
    return pcd

def merge_pointclouds(list_pcds, list_poses):
    pcd_merged = o3d.geometry.PointCloud()
    for pt, T in zip(list_pcds, list_poses):
        pc_tmp = o3d.geometry.PointCloud()
        pc_tmp.points = o3d.utility.Vector3dVector(pt)
        pc_tmp.transform(np.linalg.inv(T))
        pc_tmp.estimate_normals()
        pc_tmp.orient_normals_towards_camera_location()
        pcd_merged += pc_tmp
    return pcd_merged

def get_matching_indices(source, target, trans, search_voxel_size, K=None):
  source_copy = copy.deepcopy(source)
  target_copy = copy.deepcopy(target)
  source_copy.transform(trans)
  pcd_tree = o3d.geometry.KDTreeFlann(target_copy)

  match_inds = []
  for i, point in enumerate(source_copy.points):
    [_, idx, _] = pcd_tree.search_radius_vector_3d(point, search_voxel_size)
    if K is not None:
      idx = idx[:K]
    for j in idx:
      match_inds.append((i, j))
  return match_inds   

def compute_overlap_ratio(pcd0, pcd1, trans, voxel_size):
  pcd0_down = pcd0.voxel_down_sample(voxel_size)
  pcd1_down = pcd1.voxel_down_sample(voxel_size)
  matching01 = get_matching_indices(pcd0_down, pcd1_down, trans, voxel_size, 1)
  matching10 = get_matching_indices(pcd1_down, pcd0_down, np.linalg.inv(trans),
                                    voxel_size, 1)
  overlap0 = len(matching01) / len(pcd0_down.points)
  overlap1 = len(matching10) / len(pcd1_down.points)
  return max(overlap0, overlap1) 