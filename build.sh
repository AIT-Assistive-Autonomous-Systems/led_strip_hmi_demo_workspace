#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

# Set the default build type
BUILD_TYPE=RelWithDebInfo

# if ${ROS_DISTRO} is humble, then use this colcon build
if [ "${DEVCONTAINER_ENV}" == "desktop" ]; then
        colcon build \
                --merge-install \
                --symlink-install \
                --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" 
else 
        printf "ROS_DISTRO not supported: ${ROS_DISTRO}\n"
fi

