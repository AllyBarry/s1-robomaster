FROM ros:humble-desktop-jammy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Core GUI / GL / debugging utilities for RViz in Docker
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-image-view \
    ros-humble-image-view \
    ros-humble-tf2-tools \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libegl1-mesa \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    x11-apps \
    xauth \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Optional but useful for matching your live stack
RUN apt-get update && apt-get install -y \
    ros-humble-openni2-camera \
    ros-humble-apriltag-ros \
    ros-humble-image-proc \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1

WORKDIR /work

COPY rviz_entrypoint.sh /rviz_entrypoint.sh
RUN chmod +x /rviz_entrypoint.sh

ENTRYPOINT ["/rviz_entrypoint.sh"]
CMD ["rviz2"]
