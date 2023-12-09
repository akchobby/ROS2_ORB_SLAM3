# ORB_SLAM3_ROS2
This repository is ROS2 wrapping to use ORB_SLAM3

---

## Demo Video
[![orbslam3_ros2](https://user-images.githubusercontent.com/31432135/220839530-786b8a28-d5af-4aa5-b4ed-6234c2f4ca33.PNG)](https://www.youtube.com/watch?v=zXeXL8q72lM)

## Prerequisites

This Version is tested on :
 - Ubuntu 22.04
 - OpenCV 4.5.4
 - ROS Humble

- Build ORB_SLAM3
  - Go to this [repo](https://github.com/akchobby/ORB_SLAM3) and follow build instruction.

- Install related ROS2 package
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

## How to build
1. Clone repository to your ROS workspace
```
$ mkdir -p ros2_ws/src
$ cd ros2_ws/src
$ git clone https://github.com/akchobby/ROS2_ORB_SLAM3 orbslam3_ros2
```

2. Change this [line](https://github.com/akchobby/ROS2_ORB_SLAM3/blob/00c54335ccc010d74c1e24e336aa817604124947/CMakeLists.txt#L5) to your own `python site-packages` path

3. Change this [line](https://github.com/akchobby/ROS2_ORB_SLAM3/blob/00c54335ccc010d74c1e24e336aa817604124947/CMakeModules/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path

Now, you are ready to build!
```
$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select orbslam3
```

## Troubleshootings
1. If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```


## How to use
1. Source the workspace  
```
$ source ros2_ws/install/local_setup.bash
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
