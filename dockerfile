FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install tools
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release

# 2. Add new ROS key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# 3. Add ROS Noetic repo using new key
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list

# 4. Install ROS
RUN apt-get update && apt-get install -y ros-noetic-desktop-full

# 5. Optional: rosdep
RUN apt-get install -y python3-rosdep && rosdep init && rosdep update

# Utils
RUN apt-get install -y git wget nano sudo gawk vim iputils-ping ssh byobu software-properties-common micro curl apt-transport-https

# Dependencies
RUN apt-get install -y python3-opencv ca-certificates python3-dev ninja-build \
	dirmngr gnupg2 build-essential python3-pip python3-yaml python3-tk python3-venv gnupg

#install catkin
RUN apt update && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y python3-catkin-tools && \
    pip3 install -U catkin_tools

# Make a ROS workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
#SHELL ["/bin/bash", "-c"]  # so we can use `source`
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Clone the vicon bridge
WORKDIR /catkin_ws/src
RUN git clone https://github.com/ethz-asl/vicon_bridge.git
# Build
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Add lines to the bashrc file that source ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
