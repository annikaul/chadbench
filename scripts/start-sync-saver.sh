#!/bin/bash

# Save images and lidar data in file structure
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/humble/setup.bash
source ${SCRIPT_DIR}/../install/setup.bash

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video4 & # change video device accordingly
ros2 run synchronized_data synchronized_data_node
