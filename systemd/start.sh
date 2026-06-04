#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/maum/test_2/install/setup.bash

export ROS_DOMAIN_ID=0
export PYTHONUNBUFFERED=1
export LD_LIBRARY_PATH="/home/maum/local/gtsam-4.1.1/lib:/usr/local/lib:${LD_LIBRARY_PATH}"
export ROS_LOCALHOST_ONLY=0

python3 -u /home/maum/test_2/src/navi/scripts/mqtt_mode_orchestrator.py
