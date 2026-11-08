// proto 发布器（lifecycle 节点版）。
//
// 与 proto_dual_publisher.hpp 的唯一差别：proto 侧用
// rclcpp_lifecycle::LifecyclePublisher 而不是 rclcpp::Publisher，因此
// 需要节点在 on_activate / on_deactivate 里显式调用 Activate() / Deactivate()。
//
// ============================================================================
// 为什么这个差别不能省
// ============================================================================
// dispatcher_node 的发布器是 LifecyclePublisher：节点处于 unconfigured /
// inactive 时，publish 是**静默空操作**（连消息都不序列化）。如果 proto 侧
// 用的是普通 Publisher，那么节点 inactive 期间会出现「rosidl 不发、proto 照发」
// ——订阅方看到一条有数据流的 proto 话题，会认为调度器在正常工作；一致性
// 检查器也会报 proto 多帧。这类偏差只在运行时可见，编译期和常规单测都抓不到。
//
// 所以 lifecycle 节点的 proto 侧必须逐条跟随节点的激活态，本类把
// on_activate/on_deactivate 的转发封进来，节点侧只需成对调用。
//
// 阶段 4c：rosidl 侧发布器已移除，「两侧激活态必须一致」的说法不再成立；
// 但「proto 侧必须跟随节点激活态」这条要求本身没有变——它防的是
// inactive 节点仍然发热数据，与本类是否还发 rosidl 无关。
#ifndef AMR_DISPATCHER_ROS__PROTO_LIFECYCLE_DUAL_PUBLISHER_HPP_
#define AMR_DISPATCHER_ROS__PROTO_LIFECYCLE_DUAL_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "amr_dispatcher_ros/proto_dual_publisher.hpp"

namespace amr_dispatcher_ros
{

/// lifecycle 版发布器：proto 侧是 LifecyclePublisher，需显式激活。
///
/// 构造后 proto 发布器处于未激活态；节点 on_activate 时调 Activate()，
/// on_deactivate 时调 Deactivate()，on_cleanup 时释放本对象。
///
/// 阶段 4c 之后本类只发 proto；名字沿用 4b，与普通版的理由相同。
template<typename RosidlMsgT, typename ProtoT>
class ProtoLifecycleDualPublisher
{
public:
  /// \param node       必须是 LifecycleNode（create_publisher 才会返回 LifecyclePublisher）
  /// \param rosidl_name 已解析的 rosidl 话题名（proto 名由它派生）
  /// \param qos_depth   QoS 深度
  /// \param enabled     proto 侧总开关，false 时连 proto 发布器都不创建
  ProtoLifecycleDualPublisher(
    rclcpp_lifecycle::LifecycleNode * node, const std::string & rosidl_name,
    std::size_t qos_depth, bool enabled)
  : rosidl_topic_(rosidl_name),
    proto_topic_(ProtoTopicName(rosidl_name))
  {
    if (!enabled) {
      return;
    }
    pub_ = node->create_publisher<ProtoT>(proto_topic_, rclcpp::QoS(qos_depth));
  }

  bool enabled() const {return static_cast<bool>(pub_);}

  const std::string & proto_topic() const {return proto_topic_;}
  const std::string & rosidl_topic() const {return rosidl_topic_;}

  /// 打一行启动状态。
  ///
  /// 阶段 4c 之后只剩两种状态，但**不能**把「关闭」写成一句平淡的 INFO：
  /// `proto_dual_publish=false` 意味着这个节点一条消息都不发，DDS 里也看不到
  /// 任何话题。运维若把它当成正常启动，会花很久才能定位「为什么没有数据」。
  ///
  /// 放在这里而不是各节点各写一份，是因为文案会随开关语义变化，
  /// 分散在多处必然漂移。dispatcher_node 有 3 个实例，调用方按需合并输出。
  void LogStatus(const rclcpp::Logger & logger) const
  {
    if (!pub_) {
      RCLCPP_ERROR(logger,
        "proto 侧已关闭：本节点不发布 %s（对应旧话题 %s 也已随阶段 4c 移除）。"
        "proto_dual_publish=false 仅用于测试/压测",
        proto_topic_.c_str(), rosidl_topic_.c_str());
      return;
    }
    RCLCPP_INFO(logger, "发布 proto 话题 %s（旧话题 %s 已于阶段 4c 停止发布）",
      proto_topic_.c_str(), rosidl_topic_.c_str());
  }

  /// 发布入口，名字沿用 4b。现在是「投 proto 侧」。
  ///
  /// 未激活时由 LifecyclePublisher 自行丢弃并打印一次日志——这个行为与
  /// 4b 之前 rosidl 侧的行为逐位一致，因此调用方不需要额外判断激活态。
  bool PublishAll(const RosidlMsgT & msg)
  {
    return Publish(msg);
  }

  /// 从 rosidl 消息投影并发布。未启用时为空操作。
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

  /// 与节点的 on_activate 成对调用。
  void Activate()
  {
    if (pub_) {
      pub_->on_activate();
    }
  }

  /// 是否处于激活态。
  ///
  /// 供调用点替代原先的 `xxx_pub_->is_activated()` 守卫。proto 侧未创建时
  /// 返回 false，让所有守卫保持关闭——这正是「不发消息」应有的表现。
  bool IsActivated() const
  {
    if (pub_) {
      return pub_->is_activated();
    }
    return false;
  }

  /// 与节点的 on_deactivate 成对调用。
  void Deactivate()
  {
    if (pub_) {
      pub_->on_deactivate();
    }
  }

private:
  std::string rosidl_topic_;
  std::string proto_topic_;
  typename rclcpp_lifecycle::LifecyclePublisher<ProtoT>::SharedPtr pub_;
};

}  // namespace amr_dispatcher_ros

#endif  // AMR_DISPATCHER_ROS__PROTO_LIFECYCLE_DUAL_PUBLISHER_HPP_
