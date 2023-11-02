FROM ros:foxy

RUN apt-get update && apt-get install ros-${ROS_DISTRO}-geometry-msgs
COPY control_node/ /ros2_ws/src/control_node/
COPY navigation_interfaces/ /ros2_ws/src/navigation_interfaces/
COPY fastrtps-profile.xml /ros2_ws/
WORKDIR /ros2_ws/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", "export FASTRTPS_DEFAULT_PROFILES_FILE=fastrtps-profile.xml && . install/setup.bash && ros2 run control_node controller"]