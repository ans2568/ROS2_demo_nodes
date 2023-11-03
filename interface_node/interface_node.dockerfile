FROM ros:foxy

RUN apt-get update && apt-get install python3-colcon-common-extensions python3 python3-pip -y
RUN pip3 install flask
WORKDIR /ros2_ws/src
COPY interface_node /ros2_ws/src/interface_node
COPY navigation_interfaces /ros2_ws/src/navigation_interfaces
COPY fastrtps-profile.xml /ros2_ws/
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", "export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/fastrtps-profile.xml && . install/setup.bash && ros2 run interface_node interface_node"]