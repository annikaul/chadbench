#!/bin/bash
sudo apt-get update && apt-get upgrade -y

## Dependencies
# Core:
sudo apt-get install -y build-essential cmake git ccache wget ninja-build gdb
sudo apt-get install -y iputils-ping ros-humble-pcl-ros python3-pip && pip install rosbags
# Vulkan:
sudo wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo tee /etc/apt/trusted.gpg.d/lunarg.asc
sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-jammy.list http://packages.lunarg.com/vulkan/lunarg-vulkan-jammy.list
sudo apt update && apt install -y vulkan-sdk
# Ouster:
sudo apt-get install -y ros-humble-tf2 ros-humble-tf2-eigen
sudo apt-get install -y ros-humble-rviz2 ros-humble-pcl-ros ros-humble-tf2-eigen libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev python3-colcon-common-extensions
# DLIO:
sudo apt-get install -y libomp-dev libpcl-dev libeigen3-dev
# # LVR2:
# sudo apt-get install -y ninja-build build-essential cmake cmake-curses-gui libflann-dev libgsl-dev libeigen3-dev
# sudo apt-get install -y libopenmpi-dev openmpi-bin opencl-c-headers ocl-icd-opencl-dev libcgal-dev doxygen
# sudo apt-get install -y libvtk7-dev libvtk7-qt-dev libboost-all-dev freeglut3-dev libhdf5-dev qtbase5-dev 
# sudo apt-get install -y qt5-default libqt5opengl5-dev liblz4-dev libopencv-dev libyaml-cpp-dev libspdlog-dev
# # Voxblox:
# sudo apt-get install -y python3-wstool python3-catkin-tools ros-noetic-cmake-modules protobuf-compiler autoconf
# sudo git clone https://github.com/catkin/catkin_simple.git
# sudo cd catkin_simple && bash -c ". /opt/ros/noetic/setup.bash && cmake -B build && cmake --build build && cmake --install build"
# # VDBFusion:
# sudo apt-get install -y libgflags-dev

# Lidardatasaver:
sudo apt-get install libyaml-cpp-dev
sudo apt install ros-humble-pcl-conversions ros-humble-sensor-msgs ros-humble-pcl-ros libyaml-cpp-dev

# Image saver
sudo apt-get update
sudo apt-get install -y ros-humble-usb-cam


export ROSCONSOLE_FORMAT='[ROS${severity}]: ${message}'
export LIDAR_ADDR=192.168.168.128
export PCL_TOPIC=/ouster/points
export IMU_TOPIC=/ouster/imu
export RVIZ_OUSTER=true
export RVIZ_DLIO=true