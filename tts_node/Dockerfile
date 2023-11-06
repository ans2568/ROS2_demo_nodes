FROM ros:foxy

RUN apt-get update && apt-get install python3-pip alsa-utils ffmpeg -y
RUN pip install --upgrade pip
RUN pip install google-auth google-cloud-texttospeech
RUN mkdir -p /ros2_ws/src
COPY ./src /ros2_ws/src
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", ". install/setup.bash && ros2 run tts service"]