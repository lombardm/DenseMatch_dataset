# 3DenseMatch_dataset
This repository contains the reader for DenseMatch dataset, which refers to _Lombardi M., Savardi M. Signoroni A. - 2021, "DenseMatch: a dataset for real-time 3D reconstruction"_.
If you find this code useful or we you to use the dataset in your work, please consider citing:
```latex
@data{DVN/CU4UXG_2021,
author = {Lombardi, Marco and Savardi. Mattia and Signoroni, Alberto},
publisher = {Harvard Dataverse},
title = {DenseMatch: a dataset for real-time 3D reconstruction},
year = {2021},
version = {V1},
doi = {10.7910/DVN/CU4UXG},
url = {https://doi.org/10.7910/DVN/CU4UXG}
}
```
For any question, please contact me at m.lombardi020@unibs.it

## License
Our code is released under the GNU Genereal Public License v3.0 (see [LICENSE](LICENSE) for more details).

## Download Dataset
In order to use the dataset, please go to https://doi.org/10.7910/DVN/CU4UXG
This is the Dataverse repository containing the direct link to the zip file (20 GB total).
In the zip file there is a folder containing a .npz file for each scene all point clouds data and a .tar.gz file for all the scenes containing the rgb-d data. 
Once the zip file is downloaded, we recommend to extract the files within a "dataset" folder (i.e. the root folder) and then to split the two type of files into separated subfolders named "DenseMatch_RGBD" and "DenseMatch_Pointclouds".
In the end the tree should be like this:
-> dataset (root dir)
    -> DenseMatch_RGBD (subdir_rgbd)
      -> scene 1
          -> colors, depths, camera params...
      -> scene 2
          -> colors, depths, camera params...
    -> DenseMatch_Pointcloud (subdir_pcd)
      -> scene 1.npz
      -> scene 2.npz

## Reader
The code requires only numpy, argparse and Open3d (http://www.open3d.org/). They can be easily installed via pip:
```
pip install numpy open3d argparse
```
In order to check that everything works fine, you can simply run:
```
python example.py
```
This will load a scene from the dataset in the point cloud folder (which for this example must be in "dataset/DenseMatch_Pointcloud").
The main file is _densematch_reader.py_. Here you have a few configuration parameters. The most important are:
- _--source_data_ ("rgbd" or "pcd"): to decide which type of data you want to load
- _--scene_: name of the scene to load (use an empty string "" to load all the scenes)
- _--root_dir_: root directory if different from (<CurrentCodeDir>/dataset)
- subdir names containing rgbd data (_--subdir_rgbd_) and point cloud data (_--subdir_pcd_)

For instance, to load the rgbd data of scene _bike_helmet_01_ you can use:
```
python densematch_reader.py --scene bike_helmet_01
```
