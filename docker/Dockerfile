FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    libyaml-cpp-dev \
    libgtest-dev \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-pluginlib \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/src/amr_dispatcher
COPY . /workspace/src/amr_dispatcher

WORKDIR /workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch amr_dispatcher_ros full_system.launch.py"]
