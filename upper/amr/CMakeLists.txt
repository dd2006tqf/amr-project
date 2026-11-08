cmake_minimum_required(VERSION 3.16)
project(amr_dispatcher_workspace)

# 支持直接 cmake -B build 编译 core，或通过 colcon 整体构建工作区
add_subdirectory(src/amr_dispatcher_core)
