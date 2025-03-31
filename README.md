# CHAD Benchmark

TSDF Slam using CHAD TSDF as its mapping backend

## Description

CHAD Bench was built to provide a testbed for comparison between CHAD TSDF, Voxblox, VDBFusion and Octomap.
It can run within a docker container and therefore requires no dependencies, apart from docker itself.
To run it locally, check the [dockerfile](dockerfile) for depdendencies.
Scripts are provided to ease usage of pointcloud recording and playback, as well as registration and map writes.

## Getting Started

### Acquiring pointclouds
The scripts are mainly tailored towards Ouster scanners, but should work with other setups that provide pointcloud and IMU topics.

### 0. Docker container launch (skip if running locally)

Build and launch the docker container. The parameter allows passing through a GPU for rendering.

Pick one:

```
bash scripts/docker-run.sh none
```

```
bash scripts/docker-run.sh integrated
```

```
bash scripts/docker-run.sh nvidia
```

Once the container is up and running, three separate consoles are needed.

To attach to the running container, simply call in each one:
```
bash scripts/docker-attach
```

### 1. DLIO launch

[DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git) (Direct LiDAR-Inertial Odometry) is used for registration. Topics can be adjusted either in the dockerfile within the project root or in the DLIO launch script itself.
```
bash scripts/dlio-launch.sh
```

### 2. Mapping node launch

The mapping node will use one of the mapping backends of choice using registered points from DLIO. Adjust MAP_BACKEND_IDX in [tsdf_map_node.cpp](src/tsdf_map/src/tsdf_map_node.cpp) to choose a specific backend:
* 0: CHAD TSDF
* 1: Octomap
* 2: Voxblox
* 3: VDBFusion
```
bash scripts/dlio-launch.sh
```
### 3: Pointcloud and IMU playback

Once the DLIO and mapping nodes are running, replay or stream a pointcloud and IMU topic; this example shows three ways using either an Ouster or generic bagfile.

Stream from live Ouster scanner:
```
bash scripts/ouster-stream.sh
```

Replay Ouster bag file:
```
bash scripts/ouster-replay.sh bags/<bagfile>
```


## Research Project Notes:
Currently the use of the git submodules does not work correctly for ros2. For this, currently after a new clone of the repo, run the following commands:
```
cd src/
git clone -b ros2 --recursive https://github.com/ouster-lidar/ouster-ros.git
git clone -b feature/ros2 --recursive https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git
```

For more information about the usage of the research project functionalities, go to the repository https://github.com/annikaul/researchproject.git.


## Authors

Jan Kuhlmann

## Version History

* 0.1
    * Initial Release

## Acknowledgments

* [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)
* [Octomap](https://github.com/OctoMap/octomap)
* [Voxblox](https://github.com/ethz-asl/voxblox)
* [VDBFusion](https://github.com/PRBonn/vdbfusion)
