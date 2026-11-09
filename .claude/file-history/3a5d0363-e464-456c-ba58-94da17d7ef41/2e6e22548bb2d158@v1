// 双发发布器（lifecycle 节点版）。
//
// 与 proto_dual_publisher.hpp 的唯一差别：proto 侧用
// rclcpp_lifecycle::LifecyclePublisher 而不是 rclcpp::Publisher，因此
// 需要节点在 on_activate / on_deactivate 里显式调用 Activate() / Deactivate()。
//
// ============================================================================
// 为什么这个差别不能省
// ============================================================================
// dispatcher_node 的状态发布器是 LifecyclePublisher：节点处于 unconfigured /
// inactive 时，rosidl 侧的 publish 是**静默空操作**（连消息都不序列化）。
// 如果 proto 侧用的是普通 Publisher，那么节点 inactive 期间会出现
// 「rosidl 不发、proto 照发」——订阅方看到一条有数据流的 proto 话题，
// 会认为调度器在正常工作；一致性检查器也会报 proto 多帧。
// 这类偏差只在运行时可见，编译期和常规单测都抓不到。
//
// 所以 lifecycle 节点的 proto 侧必须逐条跟随 rosidl 侧的激活态，
// 本类把 on_activate/on_deactivate 的转发封进来，节点侧只需成对调用。
#ifndef AMR_DISPATCHER_ROS__PROTO_LIFECYCLE_DUAL_PUBLISHER_HPP_
#define AMR_DISPATCHER_ROS__PROTO_LIFECYCLE_DUAL_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "amr_dispatcher_ros/proto_dual_publisher.hpp"

namespace amr_dispatcher_ros
{

/// lifecycle 版双发器：proto 侧是 LifecyclePublisher，需显式激活。
///
/// 构造后 proto 发布器处于未激活态，与 rosidl 侧一致；节点 on_activate 时
/// 调 Activate()，on_deactivate 时调 Deactivate()，on_cleanup 时释放本对象。
template<typename RosidlMsgT, typename ProtoT>
class ProtoLifecycleDualPublisher
{
public:
  /// \param node       必须是 LifecycleNode（create_publisher 才会返回 LifecyclePublisher）
  /// \param rosidl_name 已解析的 rosidl 话题名（proto 名由它派生）
  /// \param qos_depth   QoS 深度，必须与 rosidl 侧一致
  /// \param enabled     总开关，false 时连发布器都不创建
  ProtoLifecycleDualPublisher(
    rclcpp_lifecycle::LifecycleNode * node, const std::string & rosidl_name,
    std::size_t qos_depth, bool enabled)
  : rosidl_topic_(rosidl_name),
    proto_topic_(ProtoTopicName(rosidl_name)),
    enabled_(enabled)
  {
    if (!enabled_) {
      return;
    }
    pub_ = node->create_publisher<ProtoT>(proto_topic_, rclcpp::QoS(qos_depth));
  }

  bool enabled() const {return static_cast<bool>(pub_);}

  const std::string & proto_topic() const {return proto_topic_;}

  /// 从 rosidl 消息投影并发布。未启用时为空操作。
  ///
  /// 未激活时由 LifecyclePublisher 自行丢弃并打印一次日志——与 rosidl 侧
  /// 的行为逐位一致，因此调用方不需要额外判断激活态。
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

  /// 与 rosidl 侧 on_activate 成对调用。
  void Activate()
  {
    if (pub_) {
      pub_->on_activate();
    }
  }

  /// 与 rosidl 侧 on_deactivate 成对调用。
  void Deactivate()
  {
    if (pub_) {
      pub_->on_deactivate();
    }
  }

private:
  std::string rosidl_topic_;
  std::string proto_topic_;
  bool enabled_{false};
  typename rclcpp_lifecycle::LifecyclePublisher<ProtoT>::SharedPtr pub_;
};

}  // namespace amr_dispatcher_ros

#endif  // AMR_DISPATCHER_ROS__PROTO_LIFECYCLE_DUAL_PUBLISHER_HPP_
