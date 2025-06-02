#!/usr/bin/env bash
set -e

# 1. Source ROS 2 & overlay
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# 2. Build only led_viz_common
colcon build \
  --merge-install \
  --packages-select led_strip_hmi_projector \
  --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"

# 3. Run its tests with live pytest output
colcon test \
  --merge-install \
  --packages-select led_strip_hmi_projector \
  --event-handlers console_cohesion+

# 4. Show summary
colcon test-result --verbose
