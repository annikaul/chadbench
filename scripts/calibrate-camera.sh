#!/bin/bash

# Run camera calibration script (from hyperspace)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/calibration.py
