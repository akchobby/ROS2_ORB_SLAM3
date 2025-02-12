FILE_DIR=$(realpath $(dirname $0))
FOLDER_PATH="../../../../ros2_ws"
DATASETS_PATH="$HOME/Datasets/vitro/recordings"
CONFIG_PATH=$FILE_DIR/../config


xhost local:root
docker run -it --rm --name ros2_vitro_localization --gpus all --privileged --net=host --device=/dev/video0:/dev/video0 \
                                                                                    -e NVIDIA_DRIVER_CAPABILITIES=all \
                                                                                    -e DISPLAY=$DISPLAY \
                                                                                    -e XAUTHORITY=$XAUTHORITY \
                                                                                    -v /tmp/.X11-unix:/tmp/.X11-unix \
                                                                                    -v $DATASETS_PATH:/root/datasets \
                                                                                    -v $CONFIG_PATH:/root/ros2_ws/src/ROS2_ORB_SLAM3/config \
                                                                                    -v /dev/shm:/dev/shm \
                                                                                    vitro_localization_ros2:latest
                                                                                    
xhost -local:root


