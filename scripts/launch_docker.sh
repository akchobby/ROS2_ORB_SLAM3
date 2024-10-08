FILE_DIR=$(realpath $(dirname $0))
FOLDER_PATH="../../../../ros2_ws"
DATASETS_PATH="$HOME/datasets/vitro"
CONFIG_PATH=$FILE_DIR/../config
SRC_PATH=$FILE_DIR/$FOLDER_PATH/src


xhost local:root

if [ "$1" = "dev" ] ;then

    docker run -it --rm --name ros2_vitro_localization --privileged --net=host --device=/dev/video0:/dev/video0 \
                                                                                -e DISPLAY=$DISPLAY \
                                                                                -e XAUTHORITY=$XAUTHORITY \
                                                                                -v /tmp/.X11-unix:/tmp/.X11-unix \
                                                                                -v $DATASETS_PATH:/root/datasets \
                                                                                -v $SRC_PATH:/root/ros2_ws/src \
                                                                                -v /dev/shm:/dev/shm \
                                                                                vitro_localization_ros2:latest bash

else
    docker run -it --rm --name ros2_vitro_localization --privileged --net=host --device=/dev/video0:/dev/video0 \
                                                                                -e DISPLAY=$DISPLAY \
                                                                                -e XAUTHORITY=$XAUTHORITY \
                                                                                -v /tmp/.X11-unix:/tmp/.X11-unix \
                                                                                -v $DATASETS_PATH:/root/datasets \
                                                                                -v $CONFIG_PATH:/root/ros2_ws/src/ROS2_ORB_SLAM3/config \
                                                                                -v /dev/shm:/dev/shm \
                                                                                vitro_localization_ros2:latest
                                                                                    
fi
xhost -local:root


