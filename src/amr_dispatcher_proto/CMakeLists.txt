cmake_minimum_required(VERSION 3.16)
project(amr_dispatcher_proto)

# C++17 是硬性要求：
#   - protobuf 3.21 的 CMake 集成要求 17
#   - 本包的类型分发依赖 if constexpr（SILENT 分支）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ament_get_python_install_dir() 提供 PYTHON_INSTALL_DIR，用于把 pybind11
# 扩展装到 site-packages（而不是 ROS 的可执行文件目录 lib/<pkg>）。
find_package(ament_cmake_python REQUIRED)

# protobuf 用 CONFIG 模式优先，找不到再退回 MODULE。
# Ubuntu 24.04 / protobuf 3.21 装的是 Module 模式（FindProtobuf.cmake）。
# 若将来升级到 protobuf 4.x（开始提供 CONFIG 模式），这里仍然兼容。
find_package(Protobuf REQUIRED)

# ---------------------------------------------------------------------------
# protobuf 代码生成
# ---------------------------------------------------------------------------
# 所有 proto 放在同一个 PROTO_PATH 下，互相 import 才能解析。
set(PROTO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)

set(PROTO_FILES
  ${PROTO_SRC_DIR}/common.proto
  ${PROTO_SRC_DIR}/chassis.proto
  ${PROTO_SRC_DIR}/dispatcher.proto
  ${PROTO_SRC_DIR}/safety.proto
)

# 生成到 build 目录，不污染源码树。
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_GENERATED_SRCS)
set(PROTO_GENERATED_HDRS)

foreach(proto_file ${PROTO_FILES})
  get_filename_component(proto_name ${proto_file} NAME_WE)

  set(gen_src ${PROTO_GEN_DIR}/${proto_name}.pb.cc)
  set(gen_hdr ${PROTO_GEN_DIR}/${proto_name}.pb.h)

  # DEPENDS 里要带上 common.proto：其余文件都 import 它，
  # 否则增量构建时可能先编 chassis.pb.cc 再生成 common.pb.h，导致找不到头文件。
  add_custom_command(
    OUTPUT ${gen_src} ${gen_hdr}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      --cpp_out=${PROTO_GEN_DIR}
      --proto_path=${PROTO_SRC_DIR}
      ${proto_file}
    DEPENDS ${proto_file} ${PROTO_SRC_DIR}/common.proto
    COMMENT "Generating C++ from ${proto_name}.proto"
    VERBATIM
  )

  list(APPEND PROTO_GENERATED_SRCS ${gen_src})
  list(APPEND PROTO_GENERATED_HDRS ${gen_hdr})
endforeach()

# ---------------------------------------------------------------------------
# 库：proto 消息 + typesupport 头文件
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  ${PROTO_GENERATED_SRCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROTO_GEN_DIR}>
  $<INSTALL_INTERFACE:include>
)

# typesupport 头里用到 fastcdr（CDR 编解码）、rcutils（sha256）、
# rosidl_runtime_c（type hash / description 结构体）。
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Protobuf_LIBRARIES}
  fastcdr
  rcutils::rcutils
  rosidl_runtime_c::rosidl_runtime_c
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
  rosidl_typesupport_fastrtps_cpp::rosidl_typesupport_fastrtps_cpp
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
)

# ---------------------------------------------------------------------------
# 描述符集 + typesupport 绑定生成
# ---------------------------------------------------------------------------
# 描述符集是 protoc 解析结果的权威序列化，供两处使用：
#   1) 生成 typesupport 绑定清单（本文件下方）
#   2) Python CLI 运行期构建描述符池（避免 CLI 依赖 protoc 可执行文件）
set(PROTO_DESCRIPTOR_SET ${PROTO_GEN_DIR}/descriptors.pb)

add_custom_command(
  OUTPUT ${PROTO_DESCRIPTOR_SET}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    --descriptor_set_out=${PROTO_DESCRIPTOR_SET}
    --include_imports
    --proto_path=${PROTO_SRC_DIR}
    ${PROTO_FILES}
  DEPENDS ${PROTO_FILES}
  COMMENT "Generating protobuf descriptor set"
  VERBATIM
)

# 从描述符集生成绑定清单。用 Python 而非正则，是为了正确处理嵌套消息。
set(TYPESUPPORT_INC ${PROTO_GEN_DIR}/proto_typesupport_registry.inc)

# Development 组件提供 Python3_INCLUDE_DIRS / Python3_LIBRARIES，
# pybind11 模块编译需要。
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Python 扩展模块的后缀，形如 .cpython-312-x86_64-linux-gnu.so。
# 不用 Python3_SOABI_SUFFIX——那不是标准变量。直接从解释器问：
#   sysconfig.get_config_var('EXT_SUFFIX')
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c
    "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX') or '.so')"
  OUTPUT_VARIABLE Python3_EXT_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_custom_command(
  OUTPUT ${TYPESUPPORT_INC}
  COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_typesupport_bindings.py
    ${PROTO_DESCRIPTOR_SET}
    ${TYPESUPPORT_INC}
  DEPENDS ${PROTO_DESCRIPTOR_SET}
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_typesupport_bindings.py
  COMMENT "Generating typesupport bindings"
  VERBATIM
)

# 描述符集也要安装：Python CLI 运行期用它构建描述符池，
# 这样 CLI 不依赖运行环境里是否有 protoc。
install(
  FILES ${PROTO_DESCRIPTOR_SET}
  DESTINATION share/${PROJECT_NAME}/proto
)

# ---------------------------------------------------------------------------
# Python 扩展：把 typesupport handle 以 PyCapsule 暴露给 rclpy
# ---------------------------------------------------------------------------
# 存在理由：rclpy 的 C 扩展要求 _TYPE_SUPPORT 是 PyCapsule，而 protobuf
# 类型没有 rosidl_generator_py 生成物。详见 src/py_typesupport_module.cpp。
find_package(pybind11 QUIET)

if(pybind11_FOUND)
  # 不用 pybind11_add_module：它内部调用 python3_add_library，而该命令只在
  # 显式 find_package(Python3) 后由 CMake 注入，在 ament 环境里容易缺失
  # （报错 "Unknown CMake command python3_add_library"）。
  #
  # 直接 add_library(MODULE) 完全等价且更可控：
  #   - PREFIX "" 去掉 lib 前缀，模块名与 import 名一致
  #   - SUFFIX 用 Python 的扩展后缀（.cpython-312-x86_64-linux-gnu.so）
  add_library(_amr_proto_typesupport MODULE
    src/py_typesupport_module.cpp
  )

  set_target_properties(_amr_proto_typesupport PROPERTIES
    PREFIX ""
    SUFFIX "${Python3_EXT_SUFFIX}"
  )

  target_include_directories(_amr_proto_typesupport PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROTO_GEN_DIR}
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
  )

  target_link_libraries(_amr_proto_typesupport PRIVATE
    ${PROJECT_NAME}
    ${Python3_LIBRARIES}
  )

  # 生成的 .inc 必须在这个 target 编译前就绪。
  set_source_files_properties(
    src/py_typesupport_module.cpp
    PROPERTIES OBJECT_DEPENDS ${TYPESUPPORT_INC}
  )

  # 安装到 site-packages，rclpy 侧可直接 import。
  #
  # 必须是 lib/python3.X/site-packages���不能是 lib/${PROJECT_NAME}：
  # 后者是 ROS **可执行文件** 目录（ros2 run 用的），不在 PYTHONPATH 里。
  #
  # 用 ament 的官方 helper 取路径，而不是自己拼 "lib/python3.X/site-packages"：
  # 后者在 Debian 打包（deb_system scheme）等场景下会算错。ament_get_python_install_dir
  # 内部就是复刻 colcon 的 get_python_install_path 逻辑，且 sourcing setup.bash
  # 时该目录会被自动加进 PYTHONPATH。
  #
  # 之前装到 lib/${PROJECT_NAME} 的后果：`ros2 amr echo` 报
  # "找不到 _amr_proto_typesupport"，只有手工设 PYTHONPATH 才能跑。
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)

  install(
    TARGETS _amr_proto_typesupport
    LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
  )

  install(
    FILES ${TYPESUPPORT_INC}
    DESTINATION include/amr_dispatcher_proto/generated
  )
else()
  message(WARNING
    "pybind11 未找到，跳过 _amr_proto_typesupport Python 扩展。"
    "`ros2 amr echo` 将不可用（C++ 节点间 pub/sub 不受影响）。"
    "安装方法: apt install pybind11-dev")
endif()

# ---------------------------------------------------------------------------
# 冒烟自检（骨架验证，不是正式测试）
# ---------------------------------------------------------------------------
add_executable(proto_smoke_test src/smoke_test.cpp)
target_link_libraries(proto_smoke_test ${PROJECT_NAME})
ament_target_dependencies(proto_smoke_test rclcpp)

install(TARGETS proto_smoke_test DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# .proto 也装上：下游包若要做跨语言（Python/Java）桥接会需要。
install(
  DIRECTORY proto/
  DESTINATION share/${PROJECT_NAME}/proto
)

# 生成的 .pb.h 必须安装，否则下游包 include 不到。
install(
  FILES ${PROTO_GENERATED_HDRS}
  DESTINATION include/amr_dispatcher_proto/generated
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  Protobuf
  fastcdr
  rcutils
  rosidl_runtime_c
  rosidl_typesupport_cpp
  rosidl_typesupport_fastrtps_cpp
  rosidl_typesupport_introspection_cpp
)

ament_package()
