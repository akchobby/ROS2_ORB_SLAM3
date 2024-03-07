FILE_DIR=$(realpath $(dirname $0))

source $FILE_DIR/../../../install/setup.bash
ros2 run orbslam3 mono $FILE_DIR/../../../../externals/ORB_SLAM3/Vocabulary/ORBvoc.txt $FILE_DIR/../config/monocular/dji_mini3.yaml --ros-args --params-file $FILE_DIR/../config/monocular/ros_params.yaml