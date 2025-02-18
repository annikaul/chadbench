#!/bin/bash

# Stream colored pointcloud from live-lidardata and images
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# scripts/combined-record.sh &
python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/project_colored_pc.py
