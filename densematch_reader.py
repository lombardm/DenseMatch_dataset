import os
import argparse
import numpy as np
import open3d as o3d
import glob
import ioutils
import mathutils
import pcdutils

class Frame:
    '''
    Base class for describing a single frame in a Scene.
    Each frame has:
        - name (associated to the file, typically
          each frame as a numerical ID xxxxx)
        - the RGBD data created from color and depth image
          (useful for volumetric integration using Open3d)
        - the extracted point cloud, which can be used with
          registration algorithms such as RANSAC and ICP
        - a pose (to update if any registration occurs), 
          which is useful for instance when we want to 
          integrate the RGBD into a Volume
    '''
    def __init__(self, name, rgbd=None, pcd=None, pose=None):
        self.name = name
        self.rgbd = rgbd
        self.pcd = pcd
        self.pose = pose

    def __str__(self):
        return 'Frame: ' + self.name + '\n' + \
            'RGBD:\n' + str(self.rgbd.color) + str(self.rgbd.depth) + \
            'Pose:\n'  + np.array_str(self.pose)

class Scene:
    def __init__(self, name):
        '''
        Input: configuration settings
        It creates a scene containing a list of frames.
        Each frame has a RGB-D and a Point Cloud data structure implemented in 
        open3D. The user can refer to the library documentation for additional
        experiments with the dataset.
        Additional members:
            - name of the scene
            - intrinsic parameters of the camera used to acquire the data
            - other camera params (W & H of the images, Depth Shift used for
              depth image (please note that usually the value is 1000, whereas
              in this case we adopted 10000))
        '''
        self.name = name
        self.frames_list = []
        self.camera_intrinsic = None
        self.camera_params = None

    def __str__(self):
        return 'Scene: ' + self.name + '\n' + \
            'Number of frames: ' + str(len(self.frames_list))

    def read_rgbd_scene(self, input_dir, extract_pcd=False):
        print(f"Loading Scene {self.name} ", end=" ")        
        # Retrieve data paths:
        scene_dir = os.path.join(input_dir, self.name)
        cam_params_file = os.path.join(scene_dir, "params.txt")
        cam_intrinsics_file = os.path.join(scene_dir, "intrinsicCamera.txt")
        color_files = sorted(glob.glob(scene_dir + "/*.color.*"))
        depth_files = sorted(glob.glob(scene_dir + "/*.depth.*"))
        cpose_files = sorted(glob.glob(scene_dir + "/*.pose.*"))
        assert len(color_files) == len(depth_files), f"Mismatch between color and depth images. "\
            f"Num of color images found: {len(color_files)}. Num of depth images found: {len(depth_files)}"
        if len(cpose_files) == 0:
            print(f"Warning: no camera poses found. Set every pose equal to identity.")        
            cpose_files = [None] * len(color_files)
        else:
            assert len(color_files) == len(cpose_files), f"Mismatch between color and depth images and camera poses. "\
            f"Num of color and depth images found: {len(color_files)}. Num of camera poses found: {len(cpose_files)}"
        
        self.camera_params = ioutils.read_camera_params(cam_params_file)
        assert (all(x in self.camera_params for x in ['Camera_Width', 'Camera_Height', 'Depth_Shift']))
        fx, fy, cx, cy = ioutils.read_camera_intrinsics(cam_intrinsics_file)
        self.camera_intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.camera_intrinsic.set_intrinsics(
                int(self.camera_params['Camera_Width']), 
                int(self.camera_params['Camera_Height']), 
                fx, fy, cx, cy)
        
        for color_f, depth_f, cpose_f in zip(color_files, depth_files, cpose_files):
            color_name = os.path.split(color_f)[-1].split('.')[0]
            depth_name = os.path.split(depth_f)[-1].split('.')[0]
            assert color_name == depth_name, f"Bad sort for files. Check file names"
            if cpose_f is not None:
                cpose_name = os.path.split(cpose_f)[-1].split('.')[0]
                assert color_name == cpose_name, f"Bad sort for files. Check file names"
            name = color_name
            frame = Frame(name)
            frame.rgbd = pcdutils.create_rgbd_image(color_f, depth_f, depth_scale=float(self.camera_params['Depth_Shift']))
            if cpose_f is not None:
                frame.pose = ioutils.read_camera_pose(cpose_f)
            else:
                frame.pose = np.eye(4)
            if extract_pcd:
                frame.pcd = pcdutils.create_pcd_from_rgbd(frame.rgbd, self.camera_intrinsic)
            self.frames_list.append(frame)            
        print(f"Completed! The sequence contains {len(self.frames_list)} frames")

    def read_pcd_scene(self, input_dir):
        print(f"Loading Scene {self.name} ", end=" ")        
        pcd_dict_filename = os.path.join(input_dir, self.name + ".npz")
        pcds_names, pcds_points, pcds_poses, valid_pairs = ioutils.read_pcd_dict(pcd_dict_filename)
        for name, points, pose in zip(pcds_names, pcds_points, pcds_poses):
            pcd = pcdutils.create_pcd_from_points(points)
            frame = Frame(name=name, pcd=pcd, pose=pose)
            self.frames_list.append(frame)
        print(f"Completed! The sequence contains {len(self.frames_list)} frames")

    def get_all_rgbds(self):
        rgbd_list = []
        for frame in self.frames_list:
            rgbd_list.append(frame.rgbd)
        return rgbd_list
    
    def get_all_pcds(self):
        pcd_list = []
        for frame in self.frames_list:
            pcd_list.append(frame.pcd)
        return pcd_list

    def get_all_poses(self):
        pose_list = []
        for frame in self.frames_list:
            pose_list.append(frame.pose)
        return pose_list

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--source_data', default="rgbd", type=str, choices=['rgbd', 'pcd'], help='Source data type (rgbd or pcd)')    
    parser.add_argument(
        '--root_dir', default="dataset", type=str, help='path to Point Cloud data')
    parser.add_argument(
        '--subdir_rgbd', default="DenseMatch_RGBD", type=str, help='subdir containing scenes with RGBD data')        
    parser.add_argument(
        '--subdir_pcd', default="DenseMatch_Pointcloud", type=str, help='subdir containing scenes with point cloud data')                
    parser.add_argument(
        '--scene', default="dummy_01", type=str, help='Name of the scene to load')
    parser.add_argument(
        '--extract_pcd', action='store_true', help='Extract point cloud for each rgbd created')        

    config = parser.parse_args()

    subdir = config.subdir_rgbd if config.source_data == 'rgbd' else config.subdir_pcd
    input_dir = os.path.join(config.root_dir, subdir)

    # Single Scene
    if config.scene != '':
        scene = Scene(config.scene)
        if config.source_data == 'rgbd':
            scene.read_rgbd_scene(input_dir, config.extract_pcd)
        else:
            scene.read_pcd_scene(input_dir)
    # All scenes
    else:
        all_scenes_names = os.listdir(input_dir)
        scenes_list = []
        for scene_name in all_scenes_names:
            scene = Scene(scene_name)
            if config.source_data == 'rgbd':
                scene.read_rgbd_scene(input_dir, config.extract_pcd)
            else:
                scene.read_pcd_scene(input_dir)         
            scenes_list.append(scene)


            