# ROS2 
FROM osrf/ros:humble-desktop
ARG ROS_DISTRO=humble 
RUN apt-get -y update && apt-get upgrade -y
RUN apt-get -y install xauth

# NVIDIA vars
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# general update
RUN apt-get update && apt-get upgrade -y
RUN apt install -y python3-pip \
		           python3-tk \
                   sudo \
		           vim \
                   git \
                   gdb
                   

RUN apt install -y ros-${ROS_DISTRO}-geographic-msgs \
                ros-${ROS_DISTRO}-mavros-msgs \
                ros-$ROS_DISTRO-rosbag2-storage-mcap
# general update
RUN apt-get update && apt-get upgrade -y


# install pangolin
RUN mkdir -p /root/ros2_ws/
RUN mkdir -p /root/externals/
COPY ./externals /root/externals
COPY ./ros2_ws /root/ros2_ws
RUN /bin/bash -c '/root/ros2_ws/src/ROS2_ORB_SLAM3/scripts/install_orbslam3.sh'

#Local repo update
RUN mkdir -p /root/externals/
RUN mkdir -p /root/datasets/
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bash_profile
