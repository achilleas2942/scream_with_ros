#!/bin/bash

# Source the ROS setup
source /opt/ros/noetic/setup.bash

# Set the parameter
ROS_MASTER_IP=${1:-$(hostname -I | awk '{print $1}')}
ROS_IP=${2:-$ROS_MASTER_IP}

# Pull the latest updates from the repositories
cd /root/catkin_ws/src
if [ -d "cf_cbf" ]; then
    cd cf_cbf && git pull && cd ..
else
    git clone https://github.com/EricssonResearch/scream.git
fi

if [ -d "tb_cbf" ]; then
    cd tb_cbf && git pull && cd ..
else
    git clone https://github.com/achilleas2942/scream_with_ros.git
fi

# Build the workspace
cd /root/catkin_ws/
catkin_make

# Source the workspace
source /root/catkin_ws/devel/setup.bash

# Export ROS_MASTER_URI and ROS_IP
export ROS_MASTER_URI=http://"$ROS_MASTER_IP":11311
export ROS_IP="$ROS_IP"

cp /root/catkin_ws/src/scream_with_ros/src/lidar_receiver.py /root/catkin_ws/src/scream/gstscream/scripts/lidar_receiver.py
cp /root/catkin_ws/src/scream_with_ros/src/lidar_receiver.sh /root/catkin_ws/src/scream/gstscream/scripts/lidar_receiver.sh
cp /root/catkin_ws/src/scream_with_ros/src/lidar_sender.py /root/catkin_ws/src/scream/gstscream/scripts/lidar_sender.py
cp /root/catkin_ws/src/scream_with_ros/src/lidar_sender.sh /root/catkin_ws/src/scream/gstscream/scripts/lidar_sender.sh

# Building gstscream and sample applications
chmod +x /root/catkin_ws/src/scream/gstscream/scripts/build.sh
cd /root/catkin_ws/src/scream/gstscream/
./scripts/build.sh

# Start a bash shell
roscore

