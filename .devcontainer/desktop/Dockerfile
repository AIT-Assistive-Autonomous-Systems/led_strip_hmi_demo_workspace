FROM ros:jazzy-perception

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   ros-${ROS_DISTRO}-desktop-full \ 
   ros-${ROS_DISTRO}-vision-msgs-rviz-plugins\
   ros-${ROS_DISTRO}-zed-msgs \
   ros-${ROS_DISTRO}-ros-gz \
   ros-${ROS_DISTRO}-ros-gz-bridge \
   ros-${ROS_DISTRO}-xacro \
   ros-${ROS_DISTRO}-clearpath-desktop \
   # Install pip and related
   python3-pip \
   python3-venv \
   python-is-python3 \
   python3-autopep8 python3-pip \
   git-lfs \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog
