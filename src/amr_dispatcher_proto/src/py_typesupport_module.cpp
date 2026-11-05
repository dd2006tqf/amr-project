// 把 C++ 侧的 protobuf typesupport handle 暴露给 rclpy。
//
// ============================================================================
// 为什么必须有这个模块
// ============================================================================
// rclpy 的 C 扩展（_rclpy）要求消息类型的 `_TYPE_SUPPORT` 是一个 PyCapsule，
// 其内部指针指向 `rosidl_message_type_support_t`。这个 capsule 正常由
// rosidl_generator_py 生成，而 protobuf 类型没有那套生成器，所以必须自己造。
//
// 验证过程（记录在此，避免后来者重走弯路）：
//   1. rclpy.node.create_subscription(..., raw=True) 会在 **raw 判断之前**
//      调用 check_is_valid_msg_type()，硬性要求 _TYPE_SUPPORT 存在。
//   2. 传一个假对象会失败：TypeError: Object of type 'object' is not an
//      instance of 'capsule'。
//   3. 真实 ROS 消息的 _TYPE_SUPPORT 实测是 PyCapsule，name 为 NULL，
//      pointer 指向 rosidl_message_type_support_t。
//
// ============================================================================
// 这个模块做什么
// ============================================================================
// 对每个已注册的 protobuf 类型，导出一个返回 PyCapsule 的函数：
//
//   _amr_proto_typesupport.type_support_chassis_link_health()
//     -> PyCapsule(rosidl_message_type_support_t*)
//
// Python 侧把它赋给动态类的 _TYPE_SUPPORT 即可。
//
// 重要：capsule 里装的是 **静态存储期** 的 handle 指针（GetProtobufTypeSupport
// 内部是 function-local static），所以 Python 端不需要管理生命周期。
#include <pybind11/pybind11.h>

#include <Python.h>

#include "amr_dispatcher_proto/protobuf_typesupport.hpp"
#include "amr_dispatcher_proto/py_bindings.hpp"

// 必须包含生成的 pb.h：宏体里用到的 amr_dispatcher::proto::X 是 protoc
// 生成的 C++ 类，没有这些头文件就只有前向声明，无法取 descriptor()。
#include "chassis.pb.h"
#include "common.pb.h"
#include "dispatcher.pb.h"
#include "safety.pb.h"

namespace py = pybind11;

namespace
{

/// 把 typesupport handle 包成 PyCapsule。
///
/// capsule 的 name 传 nullptr，与 rosidl_generator_py 生成的一致
/// （实测：PyCapsule_GetName 返回 None）。
///
/// 析构器传 nullptr：handle 是静态存储期，不能被释放。
PyObject * MakeTypeSupportCapsule(const rosidl_message_type_support_t * handle)
{
  return PyCapsule_New(
    const_cast<rosidl_message_type_support_t *>(handle),
    nullptr,
    nullptr);
}

}  // namespace

PYBIND11_MODULE(_amr_proto_typesupport, m)
{
  m.doc() = "Exposes protobuf-backed ROS 2 typesupport handles as PyCapsules "
            "so rclpy can subscribe to protobuf topics in raw mode.";

  // 生成的条目清单。每条展开成一次 m.def(...)。
  // .inc 由 CMake 的 gen_typesupport_bindings.py 从 FileDescriptorSet 生成。
  // 在这里 include 是合法的：预处理器只做文本插入，被插入的条目在函数体内
  // 语法合法（它们都是 m.def(...) 语句）。
#include "proto_typesupport_registry.inc"
}
