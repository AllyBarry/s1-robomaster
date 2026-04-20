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
    && rm -rf /var/lib/apt/lists/*

# rqt + rqt_robot_steering (slider-based virtual joystick for cmd_vel)
RUN apt-get update && apt-get install -y \
    ros-humble-rqt \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-graph \
    ros-humble-rqt-topic \
    ros-humble-rqt-publisher \
    ros-humble-teleop-twist-keyboard \
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
