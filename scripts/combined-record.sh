#!/bin/bash

# Save lidar data and images in filestructure
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/humble/setup.bash 
source ${SCRIPT_DIR}/../install/setup.bash

# ensure ros2 daemon is running
ros2 daemon start
ros2 topic list &> /dev/null
ros2_daemon_status=$?
while [ $ros2_daemon_status -ne 0 ]; do
    ros2 topic list &> /dev/null
    ros2_daemon_status=$?
done

scripts/ouster-stream.sh &
# scripts/startimagesaver.sh &
scripts/startlidardatasaver.sh

