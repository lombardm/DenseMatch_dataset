import os
import open3d as o3d
from densematch_reader import Scene
import pcdutils

scene = Scene("dummy_01")
scene.read_pcd_scene(os.path.join("dataset", "DenseMatch_Pointcloud"))
pcd_list = scene.get_all_pcds()
pose_list = scene.get_all_pcds()
pcd = pcdutils.merge_pointclouds(pcd_list, pose_list)
o3d.visualization.draw_geometries([pcd])