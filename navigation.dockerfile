FROM ros:foxy

RUN apt-get update && apt-get install git ros-${ROS_DISTRO}-test-msgs ros-${ROS_DISTRO}-ompl ros-${ROS_DISTRO}-gazebo-ros-pkgs -y

WORKDIR /ros2_ws/src/
RUN git clone -b foxy-devel https://github.com/ros-planning/navigation2.git
COPY nav2_bringup/ /ros2_ws/src/navigation2/nav2_bringup/
COPY navigation_interfaces/ /ros2_ws/src/navigation_interfaces/
COPY navigation_node/ /ros2_ws/src/navigation_node/
WORKDIR /ros2_ws/
COPY fastrtps-profile.xml /ros2_ws/fastrtps-profile.xml
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

CMD ["/bin/bash", "-c", "export FASTRTPS_DEFAULT_PROFILES_FILE=fastrtps-profile.xml && . install/setup.bash && ros2 launch nav2_bringup bringup_launch.py"]