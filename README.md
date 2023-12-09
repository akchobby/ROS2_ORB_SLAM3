# ORB_SLAM3_ROS2
This repository is ROS2 wrapping to use ORB_SLAM3

---

## Demo Video
[![orbslam3_ros2](https://user-images.githubusercontent.com/31432135/220839530-786b8a28-d5af-4aa5-b4ed-6234c2f4ca33.PNG)](https://www.youtube.com/watch?v=zXeXL8q72lM)

## Prerequisites


Make sure you have the following installed:
- git
- [vcstool](https://github.com/dirk-thomas/vcstool?tab=readme-ov-file#how-to-install-vcstool)
- wget

All test on this repo were done on:
 - Ubuntu 22.04
 - OpenCV 4.5.4
 - ROS Humble

 The repository also contains a dockerfile for developers if using it through docker:

 - nvidia drivers
 - nvidia container toolkit


## How to build (via Docker)
1. From the folder of your choice, run the following command

```bash
wget https://raw.githubusercontent.com/akchobby/ROS2_ORB_SLAM3/main/repos.yaml
```
Note the command takes about 5-6 minutes to complete due to the large repos

2. build the docker image by running the build script

```bash
./ros2_ws/src/ROS2_ORB_SLAM3/scripts/build_docker.sh
```

Its a comprehensive build and can take upto 15mins.

3. Change this [line](https://github.com/akchobby/ROS2_ORB_SLAM3/blob/main/scripts/launch_docker.sh#L3) to your own `datasets` path

Now, you are ready to run!

## How to use
1. Launch a container of via
```
./ros2_ws/src/ROS2_ORB_SLAM3/scripts/launch_docker.sh
```

2. Run orbslam mode, which you want.  
This repository only support `MONO, STEREO, RGBD, STEREO-INERTIAL` mode now.  
You can find vocabulary file and config file in here. (e.g. `orbslam3_ros2/vocabulary/ORBvoc.txt`, `orbslam3_ros2/config/monocular/TUM1.yaml` for monocular SLAM).
  - `MONO` mode  
```
$ ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO` mode  
```
$ ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```
  - `RGBD` mode  
```
$ ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO-INERTIAL` mode  
```
$ ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
``` 

## Acknowledgments
This repository is modified from [this](https://github.com/curryc/ros2_orbslam3) repository.  
To add `stereo-inertial` mode and improve build difficulites.
