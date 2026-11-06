// 一次性注册所有已迁移的 protobuf 消息类型。
//
// 为什么要有这个聚合头
// --------------------
// AMR_PROTO_REGISTER_TYPE 展开出的 is_message<T> 特化是**类型定义**，允许在多个
// TU 中重复出现（完全相同的类定义符合 ODR）；两个函数模板特化已标注 inline。
// 因此这个头可以被任意多个源文件包含，不会重复定义。
//
// 但反过来不成立：特化必须在使用点**之前**可见。任何 create_publisher<ProtoT>
// / create_subscription<ProtoT> 所在的翻译单元都必须先看到对应的特化，否则
// rclcpp 会走 "不是消息类型" 的编译期分支，报一堆难以定位的模板错误。
// 统一从这个头引入，避免每个节点各自记得包含哪几个 pb.h。
//
// 只注册**作为话题发布**的 7 个类型。common.proto 里的 Time / Header 是
// 纯嵌套结构，不会被单独发布，注册它们反而会多出两个同名的 DDS 类型名。
#ifndef AMR_DISPATCHER_PROTO__REGISTER_ALL_TYPES_HPP_
#define AMR_DISPATCHER_PROTO__REGISTER_ALL_TYPES_HPP_

#include "amr_dispatcher_proto/register_protobuf_type.hpp"

// 安装后生成头位于 include/amr_dispatcher_proto/generated/，
// 构建期由 target_include_directories 提供同形路径（见 CMakeLists）。
#include "amr_dispatcher_proto/generated/chassis.pb.h"
#include "amr_dispatcher_proto/generated/dispatcher.pb.h"
#include "amr_dispatcher_proto/generated/safety.pb.h"

AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::ChassisLinkHealth)
AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::TrackingError)
AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::DispatcherState)
AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::VehicleLinkState)
AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::SafetyState)
AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::TopologyState)
AMR_PROTO_REGISTER_TYPE(amr_dispatcher::proto::MissionEvent)

#endif  // AMR_DISPATCHER_PROTO__REGISTER_ALL_TYPES_HPP_
