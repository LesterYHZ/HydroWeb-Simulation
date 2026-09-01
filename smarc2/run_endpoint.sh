#!/bin/bash

# 1. Source your ROS 2 workspace setup file
# Replace 'humble' with your ROS 2 distro (e.g., jazzy, iron, foxy)
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

# 2. Get the active local IP address automatically
ROS_IP=$(hostname -I | awk '{print $1}')
ROS_PORT=10000

echo "Starting ROS TCP Endpoint..."
echo "IP Address: $ROS_IP"
echo "TCP Port:   $ROS_PORT"

# 3. Run the executable node
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:="$ROS_IP" -p ROS_TCP_PORT:=$ROS_PORT
