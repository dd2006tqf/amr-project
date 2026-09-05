FROM robot-amr:latest

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace/src/amr_dispatcher
COPY . /workspace/src/amr_dispatcher

WORKDIR /workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch amr_dispatcher_ros full_system.launch.py"]
