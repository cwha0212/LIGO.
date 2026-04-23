#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/maum/last_navi/install/setup.bash

export ROS_DOMAIN_ID=30
export PYTHONUNBUFFERED=1
export LD_LIBRARY_PATH="/home/maum/local/gtsam-4.1.1/lib:/usr/local/lib:${LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0

python3 -u /home/maum/last_navi/src/LIGO./scripts/mqtt_mode_orchestrator.py
