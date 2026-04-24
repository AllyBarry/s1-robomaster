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
    v4l-utils \
    libopenni2-0 \
    ros-humble-openni2-camera \
    ros-humble-v4l2-camera \
    # ros-humble-camera-calibration \
    ros-humble-image-pipeline \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY ros_ws/src /ros_ws/src
RUN mkdir -p /root/.ros/camera_info/
COPY ros_ws/src/rpi_camera_streamer/config/rgb_PS1080_PrimeSense.yaml /root/.ros/camera_info/
COPY ros_ws/src/rpi_camera_streamer/config/depth_PS1080_PrimeSense.yaml /root/.ros/camera_info/

# Calibration files
COPY ros_ws/src/rpi_camera_streamer/config/webcam_calib.yaml /root/.ros/camera_info/logitech_brio.yaml

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/cyclone_dds.xml

ENTRYPOINT ["/entrypoint.sh"]
# CMD ["ros2", "launch", "rpi_camera_streamer", "camera.launch.py"]
CMD ["ros2", "launch", "rpi_camera_streamer", "webcam.launch.py"]
# CMD ["ros2", "run", "camera_calibration", "cameracalibrator", "--size", "4x6", "--square", "0.04", "--ros-args", "-r", "image:=/ir/image", "-r", "camera:=/ir"]
# ros2 run camera_calibration cameracalibrator --size 4x6 --square  0.04 --ros-args -r image:=/webcam/image_raw -r camera:=/webcam/image_info
# ros2 launch rpi_camera_streamer  webcam.launch.py

#ros2 run camera_calibration cameracalibrator \
#  --size 4x6 \
#  --square 0.04 \
#  --ros-args \
#  -r image:=/webcam/image_raw \
#  -r camera:=/webcam








