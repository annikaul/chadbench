#!/bin/bash

# Show colored pointcloud from live-lidardata and images
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

python3 ${SCRIPT_DIR}/../../hyperspace/hyperspace/akresearchproject/scripts/project_colored_pc.py
