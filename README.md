<p align="center">
<img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/cupoch_logo.png" width="320" />
</p>

# Robotics with GPU computing

[![Build status](https://github.com/neka-nat/cupoch/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/neka-nat/cupoch/actions/workflows/ubuntu.yml/badge.svg)
[![Build status](https://github.com/neka-nat/cupoch/actions/workflows/windows.yml/badge.svg)](https://github.com/neka-nat/cupoch/actions/workflows/windows.yml/badge.svg)[![PyPI version](https://badge.fury.io/py/cupoch.svg)](https://badge.fury.io/py/cupoch)
![PyPI - Python Version](https://img.shields.io/pypi/pyversions/cupoch)
[![Downloads](https://pepy.tech/badge/cupoch)](https://pepy.tech/project/cupoch)
[![xscode](https://img.shields.io/badge/Available%20on-xs%3Acode-blue?style=?style=plastic&logo=appveyor&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAMAAACdt4HsAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAAAZQTFRF////////VXz1bAAAAAJ0Uk5T/wDltzBKAAAAlUlEQVR42uzXSwqAMAwE0Mn9L+3Ggtgkk35QwcnSJo9S+yGwM9DCooCbgn4YrJ4CIPUcQF7/XSBbx2TEz4sAZ2q1RAECBAiYBlCtvwN+KiYAlG7UDGj59MViT9hOwEqAhYCtAsUZvL6I6W8c2wcbd+LIWSCHSTeSAAECngN4xxIDSK9f4B9t377Wd7H5Nt7/Xz8eAgwAvesLRjYYPuUAAAAASUVORK5CYII=)](https://xscode.com/neka-nat/cupoch)

<a href="https://www.buymeacoffee.com/nekanat" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" width="150" height="40" ></a>

Cupoch is a library that implements rapid 3D data processing for robotics using CUDA.

The goal of this library is to implement fast 3D data computation in robot systems.
For example, it has applications in SLAM, collision avoidance, path planning and tracking.
This repository is based on [Open3D](https://github.com/intel-isl/Open3D).

## Core Features

* 3D data processing and robotics computation using CUDA
    * KNN
        * [WIP] [Optimizing LBVH-Construction and Hierarchy-Traversal to accelerate kNN Queries on Point Clouds using the GPU](https://epub.uni-bayreuth.de/5288/1/cgf.14177.pdf)
        * [flann](https://github.com/flann-lib/flann)
    * Point cloud registration
        * ICP
        * [Colored Point Cloud Registration](https://ieeexplore.ieee.org/document/8237287)
        * [Fast Global Registration](http://vladlen.info/papers/fast-global-registration.pdf)
        * [FilterReg](https://arxiv.org/abs/1811.10136)
    * Point cloud features
        * FPFH
        * SHOT
    * Point cloud keypoints
        * ISS
    * Point cloud clustering
        * [G-DBSCAN: A GPU Accelerated Algorithm for Density-based Clustering](https://www.sciencedirect.com/science/article/pii/S1877050913003438)
    * Point cloud/Triangle mesh filtering, down sampling
    * IO
        * Several file types(pcd, ply, stl, obj, urdf)
        * ROS message
    * Create Point Cloud from Laser Scan or RGBD Image
    * Visual Odometry
        * [Real-time visual odometry from dense RGB-D images](https://ieeexplore.ieee.org/document/6130321)
        * [Robust Odometry Estimation for RGB-D Cameras](https://ieeexplore.ieee.org/document/6631104)
    * Kinect Fusion
    * Stereo Matching
    * Collision checking
    * Occupancy grid
    * Distance transform
        * [Parallel Banding Algorithm to Compute Exact Distance Transform with the GPU](https://www.comp.nus.edu.sg/~tants/pba.html)
    * Path finding on graph structure
    * Path planning for collision avoidance
* Support memory pool and managed allocators
* Interactive GUI (OpenGL CUDA interop and [imgui](https://github.com/ocornut/imgui))
* Interoperability between cupoch 3D data and [DLPack](https://github.com/dmlc/dlpack)(Pytorch, Cupy,...) data structure

## Installation

This library is packaged under 64 Bit Ubuntu Linux 20.04 and CUDA 11.7.
You can install cupoch using pip.

```
pip install cupoch
```

Or install cupoch from source.

```
git clone https://github.com/neka-nat/cupoch.git --recurse
cd cupoch
mkdir build
cd build
cmake ..; make install-pip-package -j
```

### Installation for Jetson Nano
You can also install cupoch using pip on Jetson Nano.
Please set up Jetson using [jetpack](https://developer.nvidia.com/embedded/jetpack) and install some packages with apt.

```
sudo apt-get install libxinerama-dev libxcursor-dev libglu1-mesa-dev
pip3 install cupoch
```

Or you can compile it from source.

```
git clone https://github.com/neka-nat/cupoch.git --recurse
cd cupoch/
mkdir build
cd build/
export PATH=/usr/local/cuda/bin:$PATH
cmake -DBUILD_GLEW=ON -DBUILD_GLFW=ON -DBUILD_PNG=ON -DBUILD_JSONCPP=ON ..
sudo make install-pip-package
```

### Use Docker

Setting default container runtime to nvidia-container-runtime.
Edit or create the `/etc/docker/daemon.json`.

```sh
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
         }
    },
    "default-runtime": "nvidia"
}
```

Restart docker daemon.

```sh
sudo systemctl restart docker
```

```sh
docker-compose up -d
# xhost +
docker exec -it cupoch bash
```

## Getting Started

Please see how to use cupoch in [Getting Started](https://github.com/neka-nat/cupoch/blob/master/docs/getting_started.md) first.

## Results
The figure shows Cupoch's point cloud algorithms speedup over Open3D.
The environment tested on has the following specs:
* Intel Core i7-7700HQ CPU
* Nvidia GTX1070 GPU
* OMP_NUM_THREAD=1

You can get the result by running the example script in your environment.

```
cd examples/python/basic
python benchmarks.py
```

![speedup](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/speedup.png)

### Visual odometry with intel realsense D435

![vo](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/vo_gpu.gif)

### Occupancy grid with intel realsense D435

![og](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/og_gpu.gif)

### Kinect fusion with intel realsense L515

![kf](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/kinfu.gif)

### Stereo matching

![sm](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/stereo.png)

### Fast Global Registration

![fgr](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/fgr.png)

### Point cloud from laser scan

![fgr](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/laserscan.gif)

### Collision detection for 2 voxel grids

![col](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/collision_voxels.gif)

### Drone Path planning

![dp](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/drone_pathplanning.gif)

### Visual odometry with ROS + D435

This demo works in the following environment.
* ROS melodic
* Python2.7

```
# Launch roscore and rviz in the other terminals.
cd examples/python/ros
python realsense_rgbd_odometry_node.py
```

![vo](https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/ros_vo.gif)

## Visualization

| Point Cloud | Triangle Mesh | Kinematics |
|-------------|---------------|------------|
| <img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/pointcloud.png" width="640"> |  <img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/trianglemesh.png" width="640"> | <img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/kinematics.png" width="640"> |

| Voxel Grid | Occupancy Grid | Distance Transform |
|------------|----------------|--------------------|
|  <img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/voxelgrid.png" width="640"> | <img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/occupancygrid.png" width="640"> | <img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/distancetransform.png" width="640"> |

| Graph | Image |
|-------|-------|
| <img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/graph.png" width="640"> | <img src="https://raw.githubusercontent.com/neka-nat/cupoch/master/docs/_static/image.png" width="640"> |

## References

* CUDA repository forked from Open3D, https://github.com/theNded/Open3D
* GPU computing in Robotics, https://github.com/JanuszBedkowski/gpu_computing_in_robotics
* Voxel collision comupation for robotics, https://github.com/fzi-forschungszentrum-informatik/gpu-voxels

## Citing

```
@misc{cupoch,
   author = {Kenta Tanaka},
   year = {2020},
   note = {https://github.com/neka-nat/cupoch},
   title = {cupoch -- Robotics with GPU computing}
}
```
