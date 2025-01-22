#!/bin/bash
if [ -z "$1" ]; then
    echo "usage: scripts/ouster-replay.sh bags/<name>.bag"
    exit -1
fi
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/humble/setup.bash
source ${SCRIPT_DIR}/../install/setup.bash
ros2 launch ouster_ros replay.launch.xml \
    bag_file:=${SCRIPT_DIR}/../$1 \
    viz:=${RVIZ_OUSTER}