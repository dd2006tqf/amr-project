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
  /// \param enabled     proto 侧总开关，false 时连 proto 发布器都不创建
  /// \param rosidl_enabled rosidl 侧开关（阶段 4b）。**默认 true**，
  ///                    既有调用点行为逐位不变。false 时连 rosidl 发布器都不创建，
  ///                    DDS 里不会留下一个没人订阅的 rosidl 话题。
  ProtoLifecycleDualPublisher(
    rclcpp_lifecycle::LifecycleNode * node, const std::string & rosidl_name,
    std::size_t qos_depth, bool enabled, bool rosidl_enabled = true)
  : rosidl_topic_(rosidl_name),
    proto_topic_(ProtoTopicName(rosidl_name)),
    enabled_(enabled),
    rosidl_enabled_(rosidl_enabled)
  {
    if (rosidl_enabled_) {
      rosidl_pub_ = node->create_publisher<RosidlMsgT>(
        rosidl_topic_, rclcpp::QoS(qos_depth));
    }
    if (!enabled_) {
      return;
    }
    pub_ = node->create_publisher<ProtoT>(proto_topic_, rclcpp::QoS(qos_depth));
  }

  bool enabled() const {return static_cast<bool>(pub_);}

  /// rosidl 侧是否在发布（阶段 4b）。false 表示「只留 proto」。
  bool rosidl_enabled() const {return static_cast<bool>(rosidl_pub_);}

  const std::string & proto_topic() const {return proto_topic_;}
  const std::string & rosidl_topic() const {return rosidl_topic_;}

  /// 打一行启动状态。四类组合各有明确说法，**不能**只判断一侧：
  /// 两个开关都关时若仍说「仅发布 proto」，运维会以为还有数据流，
  /// 而实际这个节点一条消息都不发——误导性日志比沉默更难排查。
  ///
  /// 放在双发器里而不是各节点各写一份，是因为文案会随开关语义变化，
  /// 分散在多处必然漂移。dispatcher_node 有 3 个实例，调用方按需合并输出。
  void LogStatus(const rclcpp::Logger & logger) const
  {
    if (pub_ && rosidl_pub_) {
      RCLCPP_INFO(logger, "双发已启用：%s + %s",
        rosidl_topic_.c_str(), proto_topic_.c_str());
      return;
    }
    if (!pub_ && !rosidl_pub_) {
      RCLCPP_ERROR(logger,
        "两侧都已关闭：本节点既不发布 %s 也不发布 %s。"
        "此组合仅用于测试，请检查 proto_dual_publish / proto_rosidl_publish",
        rosidl_topic_.c_str(), proto_topic_.c_str());
      return;
    }
    if (rosidl_pub_) {
      RCLCPP_INFO(logger, "proto 侧已关闭，仅发布 rosidl 话题 %s",
        rosidl_topic_.c_str());
      return;
    }
    // 只剩 proto：切流后的正常态。第三方若还订阅着 rosidl 话题就是静默失联，
    // DDS 不报错、订阅端也不超时退出，因此这条 WARN 是唯一的归因线索。
    RCLCPP_WARN(logger,
      "rosidl 侧已关闭，仅发布 proto 话题 %s；请确认没有订阅方仍依赖 %s",
      proto_topic_.c_str(), rosidl_topic_.c_str());
  }

  /// 同时发布两侧，激活态由两个 LifecyclePublisher 各自管理。
  ///
  /// 这是过渡期结束后各节点应当使用的**唯一**入口。
  bool PublishAll(const RosidlMsgT & msg)
  {
    bool published = false;
    if (rosidl_pub_) {
      rosidl_pub_->publish(msg);
      published = true;
    }
    if (Publish(msg)) {
      published = true;
    }
    return published;
  }

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
    // 两侧都要激活：rosidl 侧在 4b 之前一直由节点自己管理，收进来之后
    // 必须同样跟随，否则会出现「proto 激活了、rosidl 没激活」的镜像问题
    // ——正是本类头部注释描述的那个偏差的反向版本。
    if (rosidl_pub_) {
      rosidl_pub_->on_activate();
    }
    if (pub_) {
      pub_->on_activate();
    }
  }

  /// 是否处于激活态。
  ///
  /// 供调用点替代原先的 `xxx_pub_->is_activated()` 守卫。两侧由
  /// Activate()/Deactivate() 成对驱动，正常情况下状态一致；但两侧**可能只存在
  /// 一侧**（proto_dual_publish / proto_rosidl_publish 任一关闭），所以不能写死
  /// 读 rosidl 侧——那样在「只发 proto」时会让所有守卫恒假，节点看起来活着
  /// 却一条消息都不发。
  bool IsActivated() const
  {
    if (rosidl_pub_) {
      return rosidl_pub_->is_activated();
    }
    if (pub_) {
      return pub_->is_activated();
    }
    return false;
  }

  /// 与 rosidl 侧 on_deactivate 成对调用。
  void Deactivate()
  {
    if (rosidl_pub_) {
      rosidl_pub_->on_deactivate();
    }
    if (pub_) {
      pub_->on_deactivate();
    }
  }

private:
  std::string rosidl_topic_;
  std::string proto_topic_;
  bool enabled_{false};
  bool rosidl_enabled_{true};
  typename rclcpp_lifecycle::LifecyclePublisher<ProtoT>::SharedPtr pub_;
  /// rosidl 侧发布器。仅在 rosidl_enabled 为 true 时创建。
  typename rclcpp_lifecycle::LifecyclePublisher<RosidlMsgT>::SharedPtr rosidl_pub_;
};

}  // namespace amr_dispatcher_ros

#endif  // AMR_DISPATCHER_ROS__PROTO_LIFECYCLE_DUAL_PUBLISHER_HPP_
