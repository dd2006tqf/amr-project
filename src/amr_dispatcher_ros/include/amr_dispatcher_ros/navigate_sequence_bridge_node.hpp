#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amr_dispatcher_interfaces/action/navigate_sequence.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"

// Nav2 分支条件编译：镜像缺 nav2_msgs 时该宏未定义，use_nav2:=true 会降级 fallback
#ifdef AMR_HAVE_NAV2_MSGS
#include <nav2_msgs/action/navigate_to_pose.hpp>
#endif

namespace amr_dispatcher_ros {

// NavigateSequence 桥接节点：把 /navigate_sequence 语义接到既有执行链。
//
// - fallback（默认，use_nav2:=false）：当前下标航点转单点 nav_msgs/Path 发
//   /plan（复用 path_tracker 执行链），订阅 /tracking/error 以 arrived 作为
//   该航点到达信号逐点推进；loop:=true 到达终点后从头重发，直到 cancel。
// - Nav2 模式（use_nav2:=true 且编译期 AMR_HAVE_NAV2_MSGS）：按航点顺序逐个
//   向 /navigate_to_pose 发 goal，单点失败即整 goal abort，cancel 级联取消。
//
// 普通 rclcpp::Node（非 lifecycle）：桥接只做话题/Action 搬运，无需生命周期编排。
class NavigateSequenceBridgeNode : public rclcpp::Node {
 public:
  explicit NavigateSequenceBridgeNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using NavigateSequence = amr_dispatcher_interfaces::action::NavigateSequence;
  using GoalHandleNavSeq = rclcpp_action::ServerGoalHandle<NavigateSequence>;

  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const NavigateSequence::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleNavSeq> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandleNavSeq> goal_handle);

  // fallback：把 goal_waypoints_[current_index_] 作为单点路径发到 /plan
  void PublishCurrentWaypoint();
  // /tracking/error 到达信号（仅 fallback 消费，Nav2 模式由 navigate_to_pose 结果驱动）
  void TrackingErrorCb(const amr_dispatcher_interfaces::msg::TrackingError::SharedPtr msg);
  // 当前航点到达后推进下标 / 收尾 succeed / loop 回绕
  void AdvanceAfterArrival();
  // 发布 NavigateSequence feedback（current_index/total/current_station/state）
  void PublishSeqFeedback(const std::string& state, double distance_m);
  // 轮询已受理的 cancel 请求并终结 goal：
  // rclcpp_action 在 HandleCancel 返回 ACCEPT 之后才把 goal 迁移到 CANCELING，
  // 而 canceled() 只允许在 CANCELING 态调用，因此终结动作必须延后一拍。
  void CancelPollTick();

#ifdef AMR_HAVE_NAV2_MSGS
  void SendNav2GoalForCurrentIndex();
  void Nav2FeedbackCb(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& gh,
      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>& fb,
      const std::weak_ptr<GoalHandleNavSeq>& seq_goal);
  void Nav2ResultCb(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result,
      const std::weak_ptr<GoalHandleNavSeq>& seq_goal);
#endif

  bool use_nav2_{false};
  bool nav2_compiled_{false};  // 编译期是否存在 AMR_HAVE_NAV2_MSGS
  std::string nav2_action_name_;
  std::string plan_topic_;

  rclcpp_action::Server<NavigateSequence>::SharedPtr action_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::TrackingError>::SharedPtr
      tracking_error_sub_;
  rclcpp::TimerBase::SharedPtr cancel_poll_timer_;
#ifdef AMR_HAVE_NAV2_MSGS
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
#endif

  // 当前执行中的 NavigateSequence goal（单车模型：同时只执行一个）
  std::shared_ptr<GoalHandleNavSeq> current_goal_;
  // 预计算航点位姿（frame_id=map，yaw 由相邻点 atan2 得出，单点为 0）
  std::vector<geometry_msgs::msg::PoseStamped> nav_poses_;
  std::vector<std::string> station_names_;
  bool loop_{false};
  std::size_t current_index_{0};
  bool waiting_arrival_{false};  // fallback：已发 /plan，等待 arrived
  bool nav2_pending_{false};     // Nav2：单点 goal 在途，等待 result
  int completed_waypoints_{0};
  int failed_waypoints_{0};
};

}  // namespace amr_dispatcher_ros
