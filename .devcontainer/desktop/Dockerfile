FROM ros:jazzy-perception

ENV DEBIAN_FRONTEND=noninteractive

# --- BEGIN KEY FIX: install new ROS GPG key before any apt-get update ---
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
# --- END KEY FIX ---

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
