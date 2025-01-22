#!/bin/bash

# Save data from scanner in file structure
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/humble/setup.bash
source ${SCRIPT_DIR}/../install/setup.bash
ros2 run lidardatasaver lidardatasaver_node
