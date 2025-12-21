#!/bin/bash

source /opt/ros/humble/setup.bash

# Determine workspace path based on container
if [ -d "/root/temperature_publisher_ws" ]; then
    WORKSPACE="/root/temperature_publisher_ws"
    PACKAGE="temperature_publisher"
    NODE="temperature_publisher_node"
elif [ -d "/root/temperature_subscriber_ws" ]; then
    WORKSPACE="/root/temperature_subscriber_ws"
    PACKAGE="temperature_subscriber"
    NODE="temperature_subscriber_node"
else
    echo "No workspace found!"
    exit 1
fi

cd $WORKSPACE

# Build workspace
echo "Building workspace..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to $PACKAGE

# Source ROS setup
source $WORKSPACE/install/setup.bash

# Run the node
exec ros2 run $PACKAGE $NODE
