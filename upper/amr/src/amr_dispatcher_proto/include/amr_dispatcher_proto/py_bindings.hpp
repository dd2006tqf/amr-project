// typesupport 绑定的宏定义。
//
// 分工（简洁版）：
//   - 本头文件只定义一个宏 AMR_PROTO_BIND_TYPESUPPORT(cpp_type, func_name)
//   - 生成的 proto_typesupport_registry.inc 包含一串该宏的调用条目
//   - py_typesupport_module.cpp 在 PYBIND11_MODULE 体内 include .inc，
//     条目就地展开成 m.def(...)
//
// 之所以能"就地 include"，是因为预处理器的 #include 是纯文本插入，
// 不要求位于文件作用域——它可以在函数体内，只要被插入的内容语法合法。
// 这让生成器无需理解宏语义，只负责列类型清单。
#ifndef AMR_DISPATCHER_PROTO__PY_BINDINGS_HPP_
#define AMR_DISPATCHER_PROTO__PY_BINDINGS_HPP_

#include <Python.h>

#include <rosidl_runtime_c/message_type_support_struct.h>

/// 为一个 protobuf 类型注册 Python 可见的 typesupport 获取函数。
///
/// 依赖两个在展开点可见的名字：
///   - `m`：pybind11 的 module 对象（PYBIND11_MODULE 提供）
///   - `MakeTypeSupportCapsule`：把 handle 包成 PyCapsule 的辅助函数
///
/// 关于返回值类型：
///   不能直接把 lambda 的返回类型写成 `PyObject *`——pybind11 的类型转换器
///   不认识裸 PyObject*（会报 "Unregistered type : _object"）。
///   必须用 `py::object`（这里用 reinterpret_steal 接管新引用）告诉 pybind11
///   "这已经是一个 Python 对象，直接交出去"。
///
/// \param cpp_type  protoc 生成的 C++ 全限定类型名
/// \param func_name Python 侧可见的函数名（模块内唯一）
#define AMR_PROTO_BIND_TYPESUPPORT(cpp_type, func_name)                 \
  m.def(                                                               \
    #func_name,                                                        \
    []() -> py::object {                                               \
      /* GetProtobufTypeSupport 返回 function-local static 的引用， */ \
      /* 生命周期到进程结束，因此 capsule 无需析构器。 */              \
      return py::reinterpret_steal<py::object>(                        \
        MakeTypeSupportCapsule(                                        \
          &::amr_dispatcher_proto::GetProtobufTypeSupport<cpp_type>())); \
    },                                                                 \
    "Returns a PyCapsule wrapping the rosidl_message_type_support_t* for " \
    #cpp_type);

#endif  // AMR_DISPATCHER_PROTO__PY_BINDINGS_HPP_
