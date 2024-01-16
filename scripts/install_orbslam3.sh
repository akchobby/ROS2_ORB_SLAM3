FILE_DIR=$(realpath $(dirname $0))


$FILE_DIR/install_pangolin.sh

cd $FILE_DIR/../../../../externals/ORB_SLAM3
chmod +x build.sh
./build.sh

source /opt/ros/$ROS_DISTRO/setup.bash
cd $FILE_DIR/../../../
colcon build --symlink-install 
