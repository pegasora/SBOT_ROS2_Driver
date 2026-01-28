#!/bin/bash
# Workspace setup script - sources ROS2 and workspace

source /opt/ros/jazzy/setup.bash

if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

echo "Workspace environment activated!"
echo "  ROS2: Jazzy"
echo "  Workspace: $([ -f install/setup.bash ] && echo 'sourced' || echo 'not built yet')"
