#!/usr/bin/env bash
set -e

# 1. Source ROS 2 and your overlay
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# 2. Live pytest output
EVENT_HANDLERS="--event-handlers console_cohesion+"

if [ "${DEVCONTAINER_ENV}" == "jetson" ]; then
  # Jetson: test only your core packages
  colcon test \
    --merge-install \
    --packages-select \
      dummy_detection_publisher \
      led_viz_common \
      led_viz_msgs \
      led_viz \
      led_viz_projector \
    ${EVENT_HANDLERS}

elif [ "${DEVCONTAINER_ENV}" == "desktop" ]; then
  # Desktop: skip the heavy ZED examples
  colcon test \
    --merge-install \
    --packages-skip \
      zed_rgb_convert \
      zed_aruco_localization \
      zed_depth_to_laserscan \
      zed_robot_integration \
      zed_topic_benchmark \
      zed_topic_benchmark_component \
      zed_topic_benchmark_interfaces \
      zed_tutorial_depth \
      zed_tutorial_pos_tracking \
      zed_tutorial_video \
      zed_display_rviz2 \
    --packages-select \
      dummy_detection_publisher \
      led_viz_common \
      led_viz_msgs \
      led_viz \
      led_viz_projector \
    ${EVENT_HANDLERS}

else
  echo "Unsupported DEVCONTAINER_ENV: ${DEVCONTAINER_ENV}"
  exit 1
fi

# 3. Show a concise summary
colcon test-result --verbose
