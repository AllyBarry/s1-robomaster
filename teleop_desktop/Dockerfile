FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

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
