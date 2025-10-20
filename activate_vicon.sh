source /opt/ros/humble/setup.bash && \
source /colcon_ws/install/setup.bash && \
echo ${VICON_IP} ${PORT}&& \
ros2 run vicon_bridge vicon_bridge --ros-args -p host_name:="${VICON_IP}:${PORT}"
#ros2 launch vicon_bridge all_segments.launch.py  vicon_ip:=${VICON_IP} port_number:=${PORT}