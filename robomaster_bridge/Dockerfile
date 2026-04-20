FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0

# Basic build tools and ROS dependency tooling
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        git \
        nano \
        vim \
        build-essential \
        cmake \
        pkg-config \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        iproute2 \
        net-tools \
        can-utils && \
    rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/cyclone_dds.xml

# Initialize rosdep once in the image
RUN rosdep init 2>/dev/null || true && rosdep update

# Create a proper colcon workspace
WORKDIR /opt/robomaster_ws/src

# Clone the package(s)
RUN git clone https://github.com/proroklab/robomaster_ros2_can.git --recurse-submodules

# Move to workspace root
WORKDIR /opt/robomaster_ws

# Install package dependencies from package.xml files
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# Override the upstream launch file that hardcodes hostname as namespace
COPY bridge.launch.py /opt/robomaster_ws/src/robomaster_ros2_can/robomaster_can_ros_bridge/launch/bridge.launch.py

# Build the workspace (limit parallel jobs to avoid OOM on Raspberry Pi)
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --parallel-workers 1 --executor sequential

# Source both underlay and overlay when a container starts
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /opt/robomaster_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /opt/robomaster_ws
CMD ["/bin/bash"]