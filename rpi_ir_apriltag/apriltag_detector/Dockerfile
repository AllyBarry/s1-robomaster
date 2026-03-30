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
    ros-humble-image-proc \
    ros-humble-apriltag-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY ros_ws/src /ros_ws/src

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "apriltag_detector", "apriltag.launch.py"]
