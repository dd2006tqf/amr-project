// 双发发布器（普通节点版）：一次调用同时发布 rosidl 与 protobuf 两路话题。
//
// ============================================================================
// 为什么需要这个封装，而不是在每个节点里写两个 publish
// ============================================================================
// 双发不是「多写一行 publish」那么简单，它有三个每次都必须处理对的细节：
//
//   1. 话题名派生。既有代码用 ResolveVehicleTopic 做 per-vehicle 派生，
//      proto 话题必须走同一套规则，否则多车场景下 __proto 只发全局名，
//      订阅方按车拿不到。这里统一交给调用方传「已解析好的 rosidl 话题名」，
//      内部只追加后缀，保证派生只做一次。
//
//   2. lifecycle 发布器的激活态。dispatcher_node 用的是
//      rclcpp_lifecycle::LifecyclePublisher，未激活时 publish 是静默空操作。
//      rosidl 侧靠 on_activate/on_deactivate 管理，proto 侧若不同步处理，
//      就会出现「节点 active 了但 proto 话题没数据」这种只在运行时可见的偏差。
//      ——这一半不在本文件里，见 proto_lifecycle_dual_publisher.hpp。
//
//   3. 总开关。过渡期必须能一键关掉 proto 侧（回滚、压测、排查），
//      而不必改代码或重新构建。开关做成构造参数，由节点参数注入。
//
// ============================================================================
// 为什么拆成两个头文件 / 两个类
// ============================================================================
// rclcpp 的 Publisher 有两种类型（普通 / lifecycle），且 lifecycle 版在
// rclcpp_lifecycle 包里。若在同一个类里用模板参数二选一，头文件就必然
// include <rclcpp_lifecycle/lifecycle_node.hpp>，让 chassis_driver /
// path_tracker / safety_gate / link_health 这四个**根本不是 lifecycle 节点**
// 的可执行目标凭空多一条 rclcpp_lifecycle 依赖——多一个包要在
// ament_target_dependencies 里声明，还得保证运行时能加载。
//
// 拆开之后：本文件只依赖 rclcpp，普通节点零成本；lifecycle 变体单独一份，
// 只有 dispatcher_node 引入。两者共用 ProtoTopicName 与 ProjectRosidlToProto，
// 所以两路投影逻辑仍只有一份实现。
#ifndef AMR_DISPATCHER_ROS__PROTO_DUAL_PUBLISHER_HPP_
#define AMR_DISPATCHER_ROS__PROTO_DUAL_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

#include "amr_dispatcher_ros/proto_projection.hpp"

// 让 create_publisher<ProtoT> 能通过 rclcpp 的消息类型检查。
#include "amr_dispatcher_proto/register_all_types.hpp"

namespace amr_dispatcher_ros
{

/// proto 话题名的后缀。
///
/// 选 `_proto` 而不是 `/proto/` 子命名空间：ROS 话题名里 `/proto/xxx` 会变成
/// 另一个层级，按前缀做 ros2 topic list 通配时会与既有话题分散；后缀形式
/// 让 `/chassis/link_health` 与 `/chassis/link_health_proto` 在列表里相邻，
/// 运维一眼能看出配对关系。
inline constexpr const char * kProtoTopicSuffix = "_proto";

/// 由 rosidl 话题名派生 proto 话题名。
///
/// 入参必须已经是**解析后的最终话题名**（即 ResolveVehicleTopic 的输出），
/// 这样 per-vehicle 派生只发生一次，不会出现 `/plan/amr_01_proto` 与
/// `/plan_proto/amr_01` 两种不一致的结果。
inline std::string ProtoTopicName(const std::string & rosidl_topic)
{
  return rosidl_topic + kProtoTopicSuffix;
}

/// 把 rosidl 消息投影成 proto 消息。
///
/// 用 if constexpr 做类型分发而不是要求 ProtoT 提供静态 Project 方法：
/// protobuf 生成的类不接受注入，而投影函数是自由函数，只能在此做分派。
/// 分支写错会在编译期落到 else 的 static_assert，不会静默走错路径。
///
/// 本函数被普通版与 lifecycle 版两个双发器共用，所以投影只此一份。
template<typename RosidlMsgT, typename ProtoT>
void ProjectRosidlToProto(const RosidlMsgT & in, ProtoT * out)
{
  using amr_dispatcher_interfaces::msg::ChassisLinkHealth;
  using amr_dispatcher_interfaces::msg::DispatcherState;
  using amr_dispatcher_interfaces::msg::MissionEvent;
  using amr_dispatcher_interfaces::msg::SafetyState;
  using amr_dispatcher_interfaces::msg::TopologyState;
  using amr_dispatcher_interfaces::msg::TrackingError;

  if constexpr (std::is_same_v<RosidlMsgT, ChassisLinkHealth> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::ChassisLinkHealth>)
  {
    proto_projection::ProjectChassisLinkHealth(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, TrackingError> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::TrackingError>)
  {
    proto_projection::ProjectTrackingError(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, DispatcherState> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::DispatcherState>)
  {
    proto_projection::ProjectDispatcherState(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, SafetyState> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::SafetyState>)
  {
    proto_projection::ProjectSafetyState(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, TopologyState> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::TopologyState>)
  {
    proto_projection::ProjectTopologyState(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, MissionEvent> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::MissionEvent>)
  {
    proto_projection::ProjectMissionEvent(in, out);
  } else {
    static_assert(
      !std::is_same_v<RosidlMsgT, RosidlMsgT>,
      "no proto projection registered for this message pair; "
      "add it to proto_projection.{hpp,cpp} and to this dispatch chain");
  }
}

/// 双发器：持有 proto 发布器，提供从 rosidl 消息投影并发布的单一入口。
///
/// 模板参数 ProtoT 必须是已在 register_all_types.hpp 注册过的类型。
/// 普通 rclcpp::Node 用本类；lifecycle 节点用 ProtoLifecycleDualPublisher。
template<typename RosidlMsgT, typename ProtoT>
class ProtoDualPublisher
{
public:
  /// 构造即创建发布器，也立即可发——普通发布器没有激活态。
  ///
  /// \param node        宿主节点（用于 create_publisher 与日志）
  /// \param rosidl_name 已解析的 rosidl 话题名（proto 名由它派生）
  /// \param qos_depth   QoS 深度。两侧必须一致：若 proto 侧深度更小，
  ///                    突发时会先丢 proto 帧，一致性检查器会报「proto 缺帧」
  ///                    而不是「消息不一致」，掩盖真正的问题。
  /// \param enabled     总开关。false 时完全不创建发布器，
  ///                    因此也不会在 DDS 里留下一个空话题。
  ProtoDualPublisher(
    rclcpp::Node * node, const std::string & rosidl_name, std::size_t qos_depth, bool enabled)
  : rosidl_topic_(rosidl_name),
    proto_topic_(ProtoTopicName(rosidl_name)),
    enabled_(enabled)
  {
    if (!enabled_) {
      return;
    }
    pub_ = node->create_publisher<ProtoT>(proto_topic_, rclcpp::QoS(qos_depth));
  }

  /// 是否真正启用了 proto 侧。调用方用它决定是否值得构造 proto 消息。
  bool enabled() const {return static_cast<bool>(pub_);}

  const std::string & proto_topic() const {return proto_topic_;}

  /// 从 rosidl 消息投影并发布。未启用时为空操作。
  ///
  /// 返回是否真的投递到底层发布器，便于调用方在测试中断言。
  bool Publish(const RosidlMsgT & msg)
  {
    if (!pub_) {
      return false;
    }
    ProtoT proto_msg;
    ProjectRosidlToProto(msg, &proto_msg);
    pub_->publish(proto_msg);
    return true;
  }

private:
  std::string rosidl_topic_;
  std::string proto_topic_;
  bool enabled_{false};
  typename rclcpp::Publisher<ProtoT>::SharedPtr pub_;
};

}  // namespace amr_dispatcher_ros

#endif  // AMR_DISPATCHER_ROS__PROTO_DUAL_PUBLISHER_HPP_
