// 双订阅适配器（阶段 3）：在 rosidl 话题与 proto 话题之间做运行时切换，
// 对调用方暴露的仍然是 rosidl 类型的订阅句柄。
//
// ============================================================================
// 它解决什么问题
// ============================================================================
// 阶段 3 要把消费者从 rosidl 话题迁到 proto 话题。若让每个消费者直接
// create_subscription<ProtoT>，那么：
//
//   - 每个消费者的回调都要改写成读 protobuf getter；
//   - 「还没收到过消息」这类判据在 proto 侧无对应表达（enum 默认值不是空串）；
//   - 一旦发现问题想回滚，得改代码重新构建。
//
// 本适配器把切换收进一个模板：订阅哪一路由构造参数决定，回调签名始终是
// RosidlMsgT::SharedPtr。proto 分支内部先反投影回 rosidl，再调用同一个回调，
// 因此**业务代码零改动**。
//
// ============================================================================
// 为什么用 shared_ptr<const void> 做类型擦除
// ============================================================================
// 两条分支的订阅句柄类型不同（rclcpp::Subscription<RosidlMsgT> /
// rclcpp::Subscription<ProtoT>），成员无法同时声明成具体类型，
// 而 std::variant 需要知道两者——那样本头文件就得把 rclcpp 的订阅模板
// 全部实例化，编译期开销大。shared_ptr<const void> 只做「保持句柄存活」，
// 正好是本类的唯一职责：订阅器的生命周期跟着本对象走。
//
// 代价：无法直接调用句柄的方法。实际用到的只有 dispatcher_node 的
// reset()（生命周期收口），本类显式提供 reset() 覆盖这一用例——已 grep
// 核实全项目只有这一处对订阅句柄调用方法。
//
// ============================================================================
// 使用方式
// ============================================================================
//   ProtoDualSubscriber<ChassisLinkHealth, amr_dispatcher::proto::ChassisLinkHealth> sub;
//   sub = decltype(sub)(this, "/chassis/link_health", 10, proto_dual_subscribe,
//                       [this](const ChassisLinkHealth::SharedPtr msg) { ... });
//
// 或直接成员初始化。回调参数用 SharedPtr 形式，与 rclcpp 原生一致，
// 因此既有的 std::bind(&Node::Cb, this, _1) 也能直接用。
#ifndef AMR_DISPATCHER_PROTO_BRIDGE__PROTO_DUAL_SUBSCRIBER_HPP_
#define AMR_DISPATCHER_PROTO_BRIDGE__PROTO_DUAL_SUBSCRIBER_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "amr_dispatcher_proto_bridge/proto_reverse_projection.hpp"

// 让 create_subscription<ProtoT> 能通过 rclcpp 的消息类型检查。
#include "amr_dispatcher_proto/register_all_types.hpp"

namespace amr_dispatcher_proto_bridge
{

/// proto 话题名的后缀。
///
/// **必须**与 amr_dispatcher_ros::kProtoTopicSuffix 逐字相同：发布端用那份
/// 定义生成话题名，订阅端用这份还原。两处不一致会得到一个「订阅成功但永远
/// 收不到消息」的静默故障——DDS 不会报错，只会安静地没有匹配的发布者。
///
/// 之所以不共享同一个常量：那需要 bridge 依赖 amr_dispatcher_ros（正向
/// 投影与双发器所在包），而 tools 必须能不依赖 ros 包。常量重复的代价由
/// 单测（topic-name parity）守住。
inline constexpr const char * kProtoTopicSuffix = "_proto";

/// 由 rosidl 话题名派生 proto 话题名。规则与发布端完全一致。
inline std::string ProtoTopicName(const std::string & rosidl_topic)
{
  return rosidl_topic + kProtoTopicSuffix;
}

/// 把 proto 消息反投影成 rosidl 消息。
///
/// 用 if constexpr 做类型分发，与正向投影的 ProjectRosidlToProto 对称。
/// 分支写错会在编译期落到 else 的 static_assert。
template<typename RosidlMsgT, typename ProtoT>
void RestoreProtoToRosidl(const ProtoT & in, RosidlMsgT * out)
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
    reverse_projection::RestoreChassisLinkHealth(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, TrackingError> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::TrackingError>)
  {
    reverse_projection::RestoreTrackingError(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, DispatcherState> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::DispatcherState>)
  {
    reverse_projection::RestoreDispatcherState(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, SafetyState> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::SafetyState>)
  {
    reverse_projection::RestoreSafetyState(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, TopologyState> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::TopologyState>)
  {
    reverse_projection::RestoreTopologyState(in, out);
  } else if constexpr (std::is_same_v<RosidlMsgT, MissionEvent> &&
    std::is_same_v<ProtoT, amr_dispatcher::proto::MissionEvent>)
  {
    reverse_projection::RestoreMissionEvent(in, out);
  } else {
    static_assert(
      !std::is_same_v<RosidlMsgT, RosidlMsgT>,
      "no proto reverse projection registered for this message pair; "
      "add it to proto_reverse_projection.{hpp,cpp} and to this dispatch chain");
  }
}

/// 双订阅器：按开关订阅 rosidl 话题或 proto 话题，回调统一收 rosidl 消息。
///
/// 模板参数 ProtoT 必须已在 register_all_types.hpp 注册。
///
/// 注意本类是**移动-only**的容器语义：它持有订阅句柄，复制没有意义。
/// 但默认构造 + 移动赋值让「先声明成员、在构造函数体内赋值」的既有写法
/// 也能用（dispatcher_node 就是这么写的）。
template<typename RosidlMsgT, typename ProtoT>
class ProtoDualSubscriber
{
public:
  using Callback = std::function<void(const typename RosidlMsgT::SharedPtr)>;

  ProtoDualSubscriber() = default;

  /// 构造即订阅。
  ///
  /// \param node        宿主节点。**模板参数**而不是 rclcpp::Node*：
  ///                    rclcpp_lifecycle::LifecycleNode **并不继承** rclcpp::Node
  ///                    （两者都持有 NodeBaseInterface，是平行关系）。
  ///                    早先按「LifecycleNode 继承自 Node」写死成 Node* 的版本
  ///                    在 dispatcher_node（唯一的 lifecycle 消费者）处编译失败，
  ///                    因此改成模板——两者都提供同名 create_subscription。
  /// \param rosidl_name 已解析的 rosidl 话题名（与发布端传的是同一个名字）
  /// \param qos_depth   QoS 深度。**必须**与发布端一致：proto 侧深度更小会
  ///                    在突发时先丢 proto 帧，表现为「消费者偶尔漏帧」，
  ///                    而不是明显的配置错误。
  /// \param use_proto   true = 订阅 "<rosidl_name>_proto"（收 proto 后反投影）；
  ///                    false = 订阅 rosidl 话题本身（行为与迁移前逐字相同）。
  /// \param callback    回调。形参是 RosidlMsgT::SharedPtr，与 rclcpp 原生一致，
  ///                    因此既有的成员函数 + std::bind 写法可直接沿用。
  template<typename NodeT>
  ProtoDualSubscriber(
    NodeT * node, const std::string & rosidl_name, std::size_t qos_depth,
    bool use_proto, Callback callback)
  : rosidl_topic_(rosidl_name),
    proto_topic_(ProtoTopicName(rosidl_name)),
    use_proto_(use_proto)
  {
    if (!use_proto_) {
      // rosidl 分支：直接把 rclcpp 的订阅句柄交出去，回调语义与原生完全一致
      // （包括 SharedPtr 的共享语义——这点在 proto 分支只能近似，见下）。
      // 注意 `template` 关键字：node 的类型 NodeT 是模板参数，编译器在这里
      // 不假定 create_subscription 是模板，缺了它会把 `<` 解析成小于号。
      rosidl_sub_ = node->template create_subscription<RosidlMsgT>(
        rosidl_topic_, rclcpp::QoS(qos_depth), std::move(callback));
      return;
    }

    // proto 分支：收到 protobuf 后反投影，再以 SharedPtr 形式交给回调。
    //
    // 每次回调新建一个 rosidl 消息并 `std::make_shared`，不给生产者复用
    // 缓冲——消费者可能把 SharedPtr 存起来（rest_gateway 就是存到成员），
    // 复用会让它们全部指向同一份被覆盖的数据。
    proto_sub_ = node->template create_subscription<ProtoT>(
      proto_topic_, rclcpp::QoS(qos_depth),
      [cb = std::move(callback)](const ProtoT & msg) {
        auto restored = std::make_shared<RosidlMsgT>();
        RestoreProtoToRosidl(msg, restored.get());
        cb(restored);
      });
  }

  ProtoDualSubscriber(const ProtoDualSubscriber &) = delete;
  ProtoDualSubscriber & operator=(const ProtoDualSubscriber &) = delete;
  ProtoDualSubscriber(ProtoDualSubscriber &&) noexcept = default;
  ProtoDualSubscriber & operator=(ProtoDualSubscriber &&) noexcept = default;

  /// 是否真的走 proto 分支。
  bool using_proto() const {return use_proto_;}

  const std::string & rosidl_topic() const {return rosidl_topic_;}

  const std::string & proto_topic() const {return proto_topic_;}

  /// 是否已建立订阅。未启用时（默认构造）返回 false。
  bool active() const {return static_cast<bool>(rosidl_sub_) || static_cast<bool>(proto_sub_);}

  /// 释放订阅句柄。
  ///
  /// 存在的唯一理由：dispatcher_node 在生命周期收口时对订阅句柄调 reset()。
  /// 保持该行为不变，避免「重新 configure 后旧订阅还在」这一类只在
  /// 重启路径上出现的缺陷。
  void reset()
  {
    rosidl_sub_.reset();
    proto_sub_.reset();
  }

private:
  std::string rosidl_topic_;
  std::string proto_topic_;
  bool use_proto_{false};
  typename rclcpp::Subscription<RosidlMsgT>::SharedPtr rosidl_sub_;
  typename rclcpp::Subscription<ProtoT>::SharedPtr proto_sub_;
};

}  // namespace amr_dispatcher_proto_bridge

#endif  // AMR_DISPATCHER_PROTO_BRIDGE__PROTO_DUAL_SUBSCRIBER_HPP_
