#!/bin/bash

# Stop ros2 processes that might not have been stopped when running the sync saver 
ros2 daemon stop

killall -9 \
  ros2 \
  _ros2_daemon \
  rviz2 \
  gzserver \
  robot_state_publisher \
  gzclient controller_server \
  lifecycle_manager \
  component_conta \
  dlio_map_node \
  dlio_odom_node \
  synchronized_da \
  usb_cam_node_ex

