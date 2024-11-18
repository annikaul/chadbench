#!/bin/bash
if [ -z "$1" ]; then
    echo "usage: scripts/ouster-record.sh bags/<name>.bag"
    exit -1
fi
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/noetic/setup.bash
source ${SCRIPT_DIR}/../devel/setup.bash
ros2 launch ouster_ros record.launch.xml \
    sensor_hostname:=${LIDAR_ADDR} \
    bag_file:=${SCRIPT_DIR}/../$1 \
    viz:=${RVIZ_OUSTER} \
    imu_port:=7008 \
    lidar_port:=7009