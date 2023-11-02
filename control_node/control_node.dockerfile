FROM ros:foxy

RUN apt-get update && apt-get install ros-${ROS_DISTRO}-geometry-msgs
COPY control_node /ros2_ws/src/
COPY navigation_interfaces /ros2_ws/src/
WORKDIR /ros2_ws/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", ". install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/control_node/fastrtps-profile.xml ros2 run control_node controller"]