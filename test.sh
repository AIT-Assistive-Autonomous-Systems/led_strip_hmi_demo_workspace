#!/usr/bin/env bash
set -e

# 1. Source ROS 2 and your overlay
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# 2. Live pytest output
EVENT_HANDLERS="--event-handlers console_cohesion+"

colcon test \
  --merge-install \
  --packages-skip led_strip_hmi_demos \
  ${EVENT_HANDLERS}


# 3. Show a concise summary
colcon test-result --verbose
