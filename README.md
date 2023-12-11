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

 The repository also contains a dockerfile for developers.
 if using it through docker, ensure the below are installed:

 - nvidia drivers
 - nvidia container toolkit


## How to build (via Docker)
1. From the folder of your choice, run the following commands

```bash
wget https://raw.githubusercontent.com/akchobby/ROS2_ORB_SLAM3/main/repos.yaml

vcs import < repos.yaml 
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

For those not using docker, follow the command in [launch_orbslam.sh]() 
This repository only supports `MONO` for now
```
ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```

## Topics

### Subscribed

name: `dji/image`
type: `sensor msgs/msg/Image` 

### Published

only map points currently visible.  
name: `map_points`  
type: `sensor_msgs/msg/PointCloud2`  

Using the standard tf buffer  
name: `tf`  
type: `geometry_msgs/msg/TransformStamped`  

## Further to improve

- Getting flag related to map merge
- Saving map should be service based
- parameterizing componnents as ROS params.
