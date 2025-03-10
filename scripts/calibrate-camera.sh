#!/bin/bash

set -m

# Calibrate camera
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/humble/setup.bash 
source ${SCRIPT_DIR}/../install/setup.bash





# trap stop_data_registration SIGINT SIGTERM EXIT

# ensure ros2 daemon is running
ros2 daemon start
ros2 topic list &> /dev/null
ros2_daemon_status=$?
while [ $ros2_daemon_status -ne 0 ]; do
    ros2 topic list &> /dev/null
    ros2_daemon_status=$?
done

scripts/ouster-stream.sh &
PID_OUSTER=&!
scripts/startlidardatasaver.sh &
PID_LIDAR=&!
scripts/startimagesaver.sh &
PID_IMAGE=&!

# scripts/combined-record.sh &
# PID_RECORD=$!


stop_data_registration() {
    echo "Stopping data registration for calibration"
    ros2 daemon stop
    # kill -- -$$
    # pkill -SIGTERM -P $$
    # kill -- -$$ 

    kill -SIGTERM $PID_OUSTER $PID_LIDAR $PID_IMAGE
    
    killros2
    pkill -f image_saver_nod
    pkill -f usb_cam_node_ex
    pkill -f lidardatasaver_
    # exit 0
}


sleep 2
declare -i i=0

while ((i < 30)); do
    # Latest directory in sampledata directory
    LAST_DIR=$(ls -td ${SCRIPT_DIR}/../sampledata/raw/* 2>/dev/null | head -n 1)

    LAST_MOD_TIME=$(stat -c %Y "$LAST_DIR")
    CURRENT_TIME=$(date +%s)
    TIME_DIFF=$((CURRENT_TIME - LAST_MOD_TIME))

    if [ $TIME_DIFF -le 3 ]; then
        TARGET_DIR="${LAST_DIR}/lidar_00000000/00000010"
        LAST_MOD_TIME=$(stat -c %Y "$TARGET_DIR")
        CURRENT_TIME=$(date +%s)
        TIME_DIFF=$((CURRENT_TIME - LAST_MOD_TIME))
        if [ $TIME_DIFF -le 3 ]; then
            # Stop other scripts
            # pkill -P $$
            # pkill -P $$
            # pkill -P $(pgrep -P $$)
            # pkill -SIGTERM -P $$
            # echo "Stopping data registration for calibration"
            # kill -TERM $PID_OUSTER $PID_LIDAR $PID_IMAGE
            # sleep 2

            echo "Stopping data registration for calibration"
            stop_data_registration
            # pkill -f lidardatasaver_
            # pkill -f usb_cam_node_ex
            # pkill -f image_saver_nod
            # ros2 daemon stop

            echo "Calibrate camera with new data"
            python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/calibration.py newdata
            exit 1
        fi
    fi
    ((i+=1))
    sleep 1
done

# Stop other scripts
# pkill -P $$

# pkill --signal TERM -P $$
# pkill --signal TERM -P $(pgrep -P $$)
# pkill --signal -P $$
# pkill --signal -P $(pgrep -P $$)
# echo "Stopping data registration for calibration"
# kill -TERM $PID_OUSTER $PID_LIDAR $PID_IMAGE
# sleep 2
stop_data_registration
# pkill -SIGTERM -P $$
# ros2 daemon stop

# Run camera calibration script (from hyperspace)
echo "Calibrate camera with sampledata"
python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/calibration.py sampledata
