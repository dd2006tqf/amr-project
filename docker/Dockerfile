FROM robot-amr:latest

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace/src
# 拷贝全部源码、launch 和 config
COPY . /workspace/src/amr_dispatcher_all/
RUN cp -r /workspace/src/amr_dispatcher_all/src/* /workspace/src/

WORKDIR /workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select amr_dispatcher_core amr_dispatcher_interfaces amr_interfaces_core amr_interfaces_fleet amr_interfaces_safety && \
    source /workspace/install/setup.bash && \
    colcon build --packages-select amr_dispatcher_ros amr_dispatcher_tools --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch /workspace/src/amr_dispatcher_all/launch/full_system.launch.py"]
