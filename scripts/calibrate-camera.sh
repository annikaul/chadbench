#!/bin/bash

# Calibrate camera
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

scripts/ouster-record.sh bags/cal.bag &
sleep 5
scripts/start-sync-saver.sh &
sleep 2

# Check if a new directory with the data was successfully created
declare -i i=0
while ((i < 30)); do
    # Latest directory in sampledata directory
    LAST_DIR=$(ls -td ${SCRIPT_DIR}/../sampledata/raw/* 2>/dev/null | head -n 1)

    LAST_MOD_TIME=$(stat -c %Y "$LAST_DIR")
    CURRENT_TIME=$(date +%s)
    TIME_DIFF=$((CURRENT_TIME - LAST_MOD_TIME))

    if [ $TIME_DIFF -le 3 ]; then
        TARGET_DIR="${LAST_DIR}/lidar_00000000/00000001"
        echo "Looking for directory $TARGET_DIR ..."
        LAST_MOD_TIME=$(stat -c %Y "$TARGET_DIR")
        CURRENT_TIME=$(date +%s)
        TIME_DIFF=$((CURRENT_TIME - LAST_MOD_TIME))
        if [ $TIME_DIFF -le 3 ]; then
            echo "Stopping data registration for calibration"
            scripts/stop-processes.sh &

            echo "Calibrate camera with new data"
            python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/calibration.py newdata
            exit 1
        fi
    fi
    ((i+=1))
    sleep 1
done

# Stop other scripts
echo "Stop ROS2 processes"
scripts/stop-processes.sh &

# Run camera calibration script (from hyperspace)
echo "Calibrate camera with sampledata"
python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/calibration.py sampledata
