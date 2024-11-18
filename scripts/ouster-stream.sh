#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/humble/setup.bash
source ${SCRIPT_DIR}/../install/setup.bash
ros2 launch ouster_ros sensor.launch.xml \
    sensor_hostname:=${LIDAR_ADDR} \
    viz:=${RVIZ_OUSTER} \
    imu_port:=7008 \
    lidar_port:=7009