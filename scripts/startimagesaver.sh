#!/bin/bash

# Save images in file structure
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/humble/setup.bash
source ${SCRIPT_DIR}/../install/setup.bash

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video4 & # change video device accordingly
ros2 run image_saver image_saver_node
