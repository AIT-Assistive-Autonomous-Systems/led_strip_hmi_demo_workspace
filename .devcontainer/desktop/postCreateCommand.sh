#!/usr/bin/env bash

# TODO: Remove hardcoded workspace path
echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/ubuntu/.bashrc
echo "if [ -f /workspaces/ws_led_strip_hmi/install/setup.bash ]; then source /workspaces/ws_led_strip_hmi/install/setup.bash; fi" >> /home/ubuntu/.bashrc

# Install Python test tools and ensure unbuffered output
pip3 install --upgrade pytest pytest-cov isort autopep8 black ruff pyyaml urdfpy networkx
echo "export PYTHONUNBUFFERED=1" >> /home/ubuntu/.bashrc