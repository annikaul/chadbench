#!/bin/bash

# Calibrate camera
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

scripts/combined-record.sh &
PID_RECORD=$!

sleep 2
declare -i i=0

while ((i < 30)); do
    # Latest directory in sampledata directory
    LAST_DIR=$(ls -td ${SCRIPT_DIR}/../sampledata/raw/* 2>/dev/null | head -n 1)

    LAST_MOD_TIME=$(stat -c %Y "$LAST_DIR")
    CURRENT_TIME=$(date +%s)
    TIME_DIFF=$((CURRENT_TIME - LAST_MOD_TIME))

    if [ $TIME_DIFF -le 3 ]; then
        TARGET_DIR="${LAST_DIR}/lidar_00000000/00000002"
        LAST_MOD_TIME=$(stat -c %Y "$TARGET_DIR")
        CURRENT_TIME=$(date +%s)
        TIME_DIFF=$((CURRENT_TIME - LAST_MOD_TIME))
        if [ $TIME_DIFF -le 3 ]; then
            kill $PID_RECORD
            echo "Calibrate camera with new data"
            python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/calibration.py newdata
            exit 1
        fi
    fi
    ((i+=1))
    sleep 1
done

kill $PID_RECORD

# Run camera calibration script (from hyperspace)
echo "Calibrate camera with sampledata"
python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/calibration.py sampledata
