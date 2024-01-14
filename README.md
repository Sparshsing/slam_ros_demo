# slam_ros_demo

-- run gui
docker run -it --name ros_neotic_gui -e DISPLAY=host.docker.internal:0.0 sparsh/ros_neotic_slam_img:slam_ready bash

start xlaunch, select multiple windows. put number as 0. then nexet next and finish. Then run rqt_graph in terminal , it will display a window.

docker exec -it f38eedf198c8d43f768c9072bf654a6b947d42397842f3d54f9421cd3fbcc49a bash

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash

roslaunch lidar_slam slam_velodyne.launch use_sim_time:=false

--- create container from image from scratch

docker run -it --name ros_noetic_slam osrf/ros:noetic-desktop-full bash
apt-get update
source /opt/ros/noetic/setup.bash



apt-get install -y git && apt-get install -y python3-pip

apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
rosdep update


apt-get install -y ros-$ROS_DISTRO-pcl-ros
apt-get install -y ros-$ROS_DISTRO-geodesy
apt-get install -y ros-$ROS_DISTRO-gps-common
apt-get install -y ros-$ROS_DISTRO-apriltag
apt-get install -y ros-$ROS_DISTRO-libg2o

apt-get install ros-$ROS_DISTRO-velodyne ros-$ROS_DISTRO-velodyne-pcl


apt-get install wget
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'

apt-get update
apt-get install -y python3-catkin-tools

cd ~
cmake -E make_directory catkin_ws && cd catkin_ws
git clone https://gitlab.kitware.com/keu-computervision/slam.git src/slam --recursive

apt install libeigen3-dev
apt install libpcl-dev
apt install libboost-all-dev

apt install software-properties-common
add-apt-repository ppa:borglab/gtsam-release-4.0
apt install libgtsam-dev

cd ~
mkdir libraries && cd libraries
git clone https://ceres-solver.googlesource.com/ceres-solver
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j8
make install

apt-get install libflann-dev ------ not work on ubunutu 20.04
cd ~/libraries
git clone https://github.com/jlblancoc/nanoflann.git
cd nanoflann
mkdir build && cd build
cmake ..
make
make install



cd ~/catkin_ws/
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash


only then we can run lidar_slam nodes

