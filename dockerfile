# default to $ROS_DISTRO; pass --build-arg ROS_DISTRO=jazzy for Jazzy
ARG ROS_DISTRO=humble

# Predefine base stages per Ubuntu; names must match "<ros>_base" below
# Jammy base for ROS $ROS_DISTRO
FROM ubuntu:22.04 AS humble_base     
# Noble base for ROS Jazzy
FROM ubuntu:24.04 AS jazzy_base      

# Select the right base by ROS_DISTRO ($ROS_DISTRO_base or jazzy_base)
FROM ${ROS_DISTRO}_base AS base

ENV DEBIAN_FRONTEND=noninteractive

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
# Do not delete this following line !!
ARG ROS_DISTRO 
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV ROS_PACKAGE=ros_base

# Common base setup
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release locales \
  && locale-gen en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*

# Add ROS repo once, verify Ubuntu↔ROS mapping, then install
RUN set -e; \
    apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg lsb-release locales && \
    locale-gen en_US.UTF-8 && rm -rf /var/lib/apt/lists/*; \
    . /etc/os-release; CODENAME="$VERSION_CODENAME"; \
    case "${ROS_DISTRO}:${CODENAME}" in \
      humble:jammy|jazzy:noble) echo "ROS ${ROS_DISTRO} on ${CODENAME}";; \
      *) echo "Unsupported combo: ROS ${ROS_DISTRO} on ${CODENAME}"; exit 1;; \
    esac; \
    mkdir -p /etc/apt/keyrings; \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
      > /etc/apt/sources.list.d/ros2.list; \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop-full \
      python3-rosdep python3-colcon-common-extensions && \
    rosdep init || true; rosdep update || true; \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

RUN apt-get update 
# RUN apt-get full-upgrade -y

# Utils
RUN apt update && apt install -y git wget nano sudo gawk vim iputils-ping ssh byobu software-properties-common micro curl apt-transport-https

# Download and build the required franka libraries
RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \ 
    libboost-thread-dev \ 
    libboost-date-time-dev \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Dependencies
RUN apt-get install -y python3-opencv ca-certificates python3-dev ninja-build \
	dirmngr gnupg2 build-essential python3-pip python3-yaml python3-tk python3-venv gnupg

# Make a ROS workspace
RUN mkdir -p /colcon_ws/src

# Clone the vicon bridge
WORKDIR /colcon_ws/src
RUN git clone https://github.com/dasc-lab/ros2-vicon-bridge.git

COPY vicon_launch.py /colcon_ws/src/ros2-vicon-bridge/launch/all_segments.launch.py

# Build
WORKDIR /colcon_ws
RUN apt update && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install"

# Add lines to the bashrc file that source ROS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /colcon_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
