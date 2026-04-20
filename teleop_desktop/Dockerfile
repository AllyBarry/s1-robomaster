# BASE_IMAGE selects between x86/standard and NVIDIA Jetson (L4T) base:
#   default:  ros:humble-ros-core
#   jetson:   dustynv/ros:humble-pytorch-l4t-r35.3.1
ARG BASE_IMAGE=ros:humble-ros-core
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Refresh ROS apt GPG key (rotated 2024-06) — old base images ship with expired key.
# Temporarily disable the ROS source list so we can install curl/gnupg from Ubuntu
# repos without tripping over the expired signature.
RUN mv /etc/apt/sources.list.d/ros2.list /tmp/ros2.list.bak 2>/dev/null || \
    mv /etc/apt/sources.list.d/ros2-latest.list /tmp/ros2.list.bak 2>/dev/null || true && \
    apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg.asc && \
    gpg --dearmor < /usr/share/keyrings/ros-archive-keyring.gpg.asc \
        > /usr/share/keyrings/ros-archive-keyring.gpg && \
    rm /usr/share/keyrings/ros-archive-keyring.gpg.asc && \
    . /etc/os-release && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
        > /etc/apt/sources.list.d/ros2.list && \
    rm -f /tmp/ros2.list.bak

# rqt + rqt_robot_steering (slider-based virtual joystick for cmd_vel)
RUN apt-get update && apt-get install -y \
    ros-humble-rqt \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-graph \
    ros-humble-rqt-topic \
    ros-humble-rqt-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-cyclonedds-cpp \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    x11-apps \
    xauth \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/cyclone_dds.xml
ENV QT_X11_NO_MITSHM=1

WORKDIR /ros_ws

COPY teleop_entrypoint.sh /ros_ws/teleop_entrypoint.sh
RUN chmod +x /ros_ws/teleop_entrypoint.sh

ENTRYPOINT ["/ros_ws/teleop_entrypoint.sh"]
CMD ["rqt_robot_steering"]
