FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-colcon-common-extensions \
    usbutils \
    libopenni2-0 \
    ros-humble-openni2-camera \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY ros_ws/src /ros_ws/src
RUN mkdir -p /root/.ros/camera_info/
COPY ros_ws/src/rpi_camera_streamer/config/rgb_PS1080_PrimeSense.yaml /root/.ros/camera_info/
COPY ros_ws/src/rpi_camera_streamer/config/depth_PS1080_PrimeSense.yaml /root/.ros/camera_info/

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rpi_camera_streamer", "camera.launch.py"]
