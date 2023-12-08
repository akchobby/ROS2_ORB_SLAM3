FILE_DIR=$(realpath $(dirname $0))

if [ "$1" = "clean" ] ;then
    cd $FILE_DIR/../../../../ && docker build --no-cache -t vitro_localization_ros2 -f ./ros2_ws/src/ROS2_ORB_SLAM3/Docker/Dockerfile . 
else 
    cd $FILE_DIR/../../../../ && docker build -t vitro_localization_ros2 -f ./ros2_ws/src/ROS2_ORB_SLAM3/Docker/Dockerfile  . 
fi
