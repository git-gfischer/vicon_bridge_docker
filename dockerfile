FROM osrf/ros:humble-desktop-full

ENV ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive

# Clean up any old broken ROS sources or keys
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Install tools and add the fresh GPG key
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Re-add the ROS 2 repository with correct signed-by config
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros2-latest.list

# Now update should work without errors
RUN apt-get update


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
