#!/bin/bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
bash --rcfile <(echo "source /root/repo/install/setup.bash")