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
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    ros-humble-image-transport-plugins \
    v4l-utils \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY ros_ws/src /ros_ws/src

# Camera calibration — v4l2_camera reads it at runtime via
# camera_info_url. Name it webcam.yaml to match the node name so the
# default URL (file:///root/.ros/camera_info/webcam.yaml) resolves.
RUN mkdir -p /root/.ros/camera_info/
COPY camera_info/webcam.yaml /root/.ros/camera_info/webcam.yaml

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/cyclone_dds.xml

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "apriltag_detector", "onboard.launch.py"]
