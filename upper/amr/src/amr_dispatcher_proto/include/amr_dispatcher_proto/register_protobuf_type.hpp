// 具体的 protobuf 类型接入 ROS 2 需要两处特化：
//   1) rosidl_generator_traits::is_message<T>
//   2) rosidl_typesupport_cpp::get_message_type_support_handle<T>()
//
// 这个头文件提供宏，避免为每个 proto 类型手写样板代码。
//
// 用法（放在你自己的包里，一个 proto 类型一行）：
//
//   #include "amr_dispatcher_proto/register_protobuf_type.hpp"
//   #include "my_msgs.pb.h"
//
//   AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::ChassisLinkHealth)
//
// 注意：宏展开出的是**显式特化**，显式特化函数模板不会自动 inline，
// 因此宏内部已加上 inline，允许本头被多个 TU 包含。
#ifndef AMR_DISPATCHER_PROTO__REGISTER_PROTOBUF_TYPE_HPP_
#define AMR_DISPATCHER_PROTO__REGISTER_PROTOBUF_TYPE_HPP_

#include <type_traits>

#include "amr_dispatcher_proto/protobuf_typesupport.hpp"

/// 把一个 protobuf 类型注册为可用的 ROS 2 消息类型。
///
/// 展开后等价于：
///   namespace rosidl_generator_traits {
///     template<> struct is_message<T> : std::true_type {};
///     template<> inline const char * name<T>()      { return "…"; }
///     template<> inline const char * data_type<T>() { return "…"; }
///   }
///   namespace rosidl_typesupport_cpp {
///     template<> inline const rosidl_message_type_support_t *
///     get_message_type_support_handle<T>() { … }
///   }
///
/// 设计说明：为什么用特化而不是 TypeAdapter？
///   rclcpp::TypeAdapter<CustomT, RosT> 需要同时给出一个 ROS 原生类型作为
///   中转，那会把 protobuf 的字节先搬进一个 vector<uint8> 再交给序列化，
///   多一次拷贝且多一份 .msg 定义。直接特化 is_message 走的是 rosidl 生成
///   代码的同一条路，零中转。
#define AMR_PROTO_REGISTER_TYPE(ProtoT) \
  namespace rosidl_generator_traits \
  { \
  template<> \
  struct is_message<ProtoT> : std::true_type {}; \
  \
  template<> \
  inline const char * name<ProtoT>() \
  { \
    return #ProtoT; \
  } \
  \
  template<> \
  inline const char * data_type<ProtoT>() \
  { \
    return #ProtoT; \
  } \
  }  /* namespace rosidl_generator_traits */ \
  \
  namespace rosidl_typesupport_cpp \
  { \
  template<> \
  inline const rosidl_message_type_support_t * \
  get_message_type_support_handle<ProtoT>() \
  { \
    return &::amr_dispatcher_proto::GetProtobufTypeSupport<ProtoT>(); \
  } \
  }  /* namespace rosidl_typesupport_cpp */

#endif  // AMR_DISPATCHER_PROTO__REGISTER_PROTOBUF_TYPE_HPP_
