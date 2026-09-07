FROM robot-amr:latest

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace/src
# 拷贝工程全部源码、launch 和 config
COPY . /workspace/src/amr_dispatcher_all/
RUN cp -r /workspace/src/amr_dispatcher_all/src/* /workspace/src/

WORKDIR /workspace
# 第一阶段：编译核心算法库与接口定义包
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select amr_dispatcher_core amr_dispatcher_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source /workspace/install/setup.bash && \
    colcon build --packages-select amr_dispatcher_bt amr_dispatcher_ros amr_dispatcher_tools --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch /workspace/src/amr_dispatcher_all/launch/full_system.launch.py"]
