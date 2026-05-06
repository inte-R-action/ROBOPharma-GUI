#!/bin/bash

echo "INITIALISING...."

# Source ROS and set the Domain ID locally
source /opt/ros/jazzy/setup.bash
source /home/roberta/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=172.20.10.3


echo "Starting Bridge and Dashboard..."
# Launch the nodes, save logs to bridge.log, run in background
nohup ros2 launch bridge launchSystem.launch.py > /home/roberta/bridge.log 2>&1 &


# Keep terminal open and stream the background logs
tail -f /home/roberta/bridge.log
