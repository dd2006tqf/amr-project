#include "amr_dispatcher_ros/navigate_sequence_bridge_node.hpp"

#include <cmath>
#include <chrono>
#include <utility>

#include "amr_dispatcher_ros/nav_pose_builder.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

NavigateSequenceBridgeNode::NavigateSequenceBridgeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("navigate_sequence_bridge_node", options) {
  declare_parameter("use_nav2", false);
  declare_parameter("nav2_action_name", "/navigate_to_pose");
  declare_parameter("plan_topic", "/plan");

  use_nav2_ = get_parameter("use_nav2").as_bool();
  nav2_action_name_ = get_parameter("nav2_action_name").as_string();
  plan_topic_ = get_parameter("plan_topic").as_string();

#ifdef AMR_HAVE_NAV2_MSGS
  nav2_compiled_ = true;
  nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, nav2_action_name_);
#else
  nav2_compiled_ = false;
#endif
  if (use_nav2_ && !nav2_compiled_) {
    // 镜像缺 nav2_msgs：use_nav2:=true 时打 FATAL 并降级 fallback，容器构建永不因 Nav2 挂掉
    RCLCPP_FATAL(get_logger(),
                 "use_nav2=true but nav2_msgs was not available at build time; "
                 "degrading to /plan fallback");
    use_nav2_ = false;
  }

  action_server_ = rclcpp_action::create_server<NavigateSequence>(
      this, "/navigate_sequence",
      std::bind(&NavigateSequenceBridgeNode::HandleGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&NavigateSequenceBridgeNode::HandleCancel, this, std::placeholders::_1),
      std::bind(&NavigateSequenceBridgeNode::HandleAccepted, this, std::placeholders::_1));

  plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic_, 1);
  tracking_error_sub_ = create_subscription<amr_dispatcher_interfaces::msg::TrackingError>(
      "/tracking/error", 10,
      std::bind(&NavigateSequenceBridgeNode::TrackingErrorCb, this, std::placeholders::_1));

  // cancel 终结轮询：HandleCancel 受理后需要等到 goal 进入 CANCELING 才能 canceled()
  cancel_poll_timer_ = create_wall_timer(
      100ms, std::bind(&NavigateSequenceBridgeNode::CancelPollTick, this));

  RCLCPP_INFO(get_logger(), "NavigateSequence bridge up: backend=%s, plan_topic=%s",
              use_nav2_ ? "nav2" : "plan(fallback)", plan_topic_.c_str());
}

rclcpp_action::GoalResponse NavigateSequenceBridgeNode::HandleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const NavigateSequence::Goal> goal) {
  if (!goal || goal->waypoints.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting NavigateSequence goal: waypoints empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (current_goal_ && current_goal_->is_active()) {
    RCLCPP_WARN(get_logger(),
                "Rejecting NavigateSequence goal: previous goal still active (single-vehicle model)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // DEFER + HandleAccepted 里显式 execute()：不受框架是否隐式 execute 影响的唯一稳妥路径
  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse NavigateSequenceBridgeNode::HandleCancel(
    const std::shared_ptr<GoalHandleNavSeq> goal_handle) {
  if (goal_handle != current_goal_) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  // 只受理取消请求：rclcpp_action 在本回调返回 ACCEPT 之后才把 goal 迁移到
  // CANCELING，而 canceled() 只允许在 CANCELING 态调用——终结交给 CancelPollTick。
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateSequenceBridgeNode::HandleAccepted(
    const std::shared_ptr<GoalHandleNavSeq> goal_handle) {
  const auto goal = goal_handle->get_goal();
  // ACCEPTED -> EXECUTING：publish_feedback/succeed/abort 的前置条件
  goal_handle->execute();

  current_goal_ = goal_handle;
  station_names_ = goal->station_names;
  loop_ = goal->loop;
  current_index_ = 0;
  completed_waypoints_ = 0;
  failed_waypoints_ = 0;
  waiting_arrival_ = false;
  nav2_pending_ = false;

  // 预计算航点位姿：单航点保留调用方原始四元数（站点朝向约束），
  // 多航点才按相邻点 atan2 重写 orientation（B3 修复：单点 yaw 不再被覆盖为 0）
  nav_poses_ = BuildNavPoses(goal->waypoints, "map");
  const std::size_t n = nav_poses_.size();

  RCLCPP_INFO(get_logger(), "NavigateSequence goal accepted: %zu waypoint(s), loop=%d, backend=%s",
              n, loop_ ? 1 : 0, use_nav2_ ? "nav2" : "plan(fallback)");

  if (use_nav2_) {
#ifdef AMR_HAVE_NAV2_MSGS
    SendNav2GoalForCurrentIndex();
#endif
  } else {
    PublishCurrentWaypoint();
  }
}

void NavigateSequenceBridgeNode::PublishCurrentWaypoint() {
  if (!current_goal_ || current_index_ >= nav_poses_.size()) {
    return;
  }
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = "map";
  path.poses.push_back(nav_poses_[current_index_]);
  path.poses.back().header = path.header;
  plan_pub_->publish(path);
  waiting_arrival_ = true;
  PublishSeqFeedback("NAVIGATING", 0.0);
  RCLCPP_INFO(get_logger(), "Publishing waypoint %zu/%zu to %s (station=%s)",
              current_index_ + 1, nav_poses_.size(), plan_topic_.c_str(),
              current_index_ < station_names_.size() ? station_names_[current_index_].c_str()
                                                     : "-");
}

void NavigateSequenceBridgeNode::TrackingErrorCb(
    const amr_dispatcher_interfaces::msg::TrackingError::SharedPtr msg) {
  if (!msg->arrived || use_nav2_) {
    return;  // Nav2 模式由 navigate_to_pose 的结果驱动，不消费 /tracking/error
  }
  if (!current_goal_ || !waiting_arrival_) {
    return;  // 与本桥当前航点无关的到达信号（如 dispatcher 自己的 /plan 链）直接忽略
  }
  waiting_arrival_ = false;
  PublishSeqFeedback("AT_STATION", 0.0);
  RCLCPP_INFO(get_logger(), "Waypoint %zu/%zu reached (station=%s)", current_index_ + 1,
              nav_poses_.size(),
              current_index_ < station_names_.size() ? station_names_[current_index_].c_str()
                                                     : "-");
  AdvanceAfterArrival();
}

void NavigateSequenceBridgeNode::AdvanceAfterArrival() {
  if (!current_goal_) {
    return;
  }
  completed_waypoints_++;
  current_index_++;
  if (current_index_ >= nav_poses_.size()) {
    if (loop_) {
      current_index_ = 0;
      RCLCPP_INFO(get_logger(), "Loop: sequence finished (%d waypoint(s) total), restarting from 0",
                  completed_waypoints_);
    } else {
      auto result = std::make_shared<NavigateSequence::Result>();
      result->success = true;
      result->message = "all waypoints reached";
      result->completed_waypoints = completed_waypoints_;
      result->failed_waypoints = failed_waypoints_;
      current_goal_->succeed(result);
      current_goal_.reset();
      waiting_arrival_ = false;
      nav2_pending_ = false;
      RCLCPP_INFO(get_logger(),
                  "NavigateSequence goal succeeded: %d waypoint(s) completed, %d failed",
                  completed_waypoints_, failed_waypoints_);
      return;
    }
  }
  if (use_nav2_) {
#ifdef AMR_HAVE_NAV2_MSGS
    SendNav2GoalForCurrentIndex();
#endif
  } else {
    PublishCurrentWaypoint();
  }
}

void NavigateSequenceBridgeNode::PublishSeqFeedback(const std::string& state,
                                                    double distance_m) {
  if (!current_goal_ || !current_goal_->is_active()) {
    return;
  }
  auto fb = std::make_shared<NavigateSequence::Feedback>();
  fb->current_index = static_cast<std::int32_t>(current_index_);
  fb->total_waypoints = static_cast<std::int32_t>(nav_poses_.size());
  fb->current_station =
      current_index_ < station_names_.size() ? station_names_[current_index_] : std::string{};
  fb->state = state;
  fb->distance_to_current_goal_m = distance_m;
  current_goal_->publish_feedback(fb);
}

void NavigateSequenceBridgeNode::StopFallbackMotion() {
  if (use_nav2_ || !plan_pub_) {
    return;  // Nav2 模式没有 /plan 链，无需空路径停车
  }
  nav_msgs::msg::Path empty;
  empty.header.stamp = now();
  empty.header.frame_id = "map";
  plan_pub_->publish(empty);
  RCLCPP_INFO(get_logger(), "Published empty %s to stop fallback motion", plan_topic_.c_str());
}

void NavigateSequenceBridgeNode::CancelPollTick() {
  if (!current_goal_ || !current_goal_->is_canceling()) {
    return;
  }
#ifdef AMR_HAVE_NAV2_MSGS
  if (use_nav2_ && nav2_client_) {
    // cancel 级联：本 goal 取消时同步取消 Nav2 全部在途 goal
    nav2_client_->async_cancel_all_goals();
  }
#endif
  // fallback 运动中被取消：发空 /plan 停车，否则 path_tracker 继续追旧航点（审查 I3）
  StopFallbackMotion();
  auto result = std::make_shared<NavigateSequence::Result>();
  result->success = false;
  result->message = "canceled by client";
  result->completed_waypoints = completed_waypoints_;
  result->failed_waypoints = failed_waypoints_;
  current_goal_->canceled(result);
  current_goal_.reset();
  waiting_arrival_ = false;
  nav2_pending_ = false;
  RCLCPP_INFO(get_logger(), "NavigateSequence goal canceled (%d waypoint(s) completed)",
              completed_waypoints_);
}

#ifdef AMR_HAVE_NAV2_MSGS
void NavigateSequenceBridgeNode::SendNav2GoalForCurrentIndex() {
  if (!current_goal_ || current_index_ >= nav_poses_.size()) {
    return;
  }
  if (!nav2_client_ || !nav2_client_->action_server_is_ready()) {
    // 单点失败 -> 整 goal abort
    failed_waypoints_++;
    auto result = std::make_shared<NavigateSequence::Result>();
    result->success = false;
    result->message = "nav2 action server not available: " + nav2_action_name_;
    result->completed_waypoints = completed_waypoints_;
    result->failed_waypoints = failed_waypoints_;
    RCLCPP_ERROR(get_logger(), "%s (waypoint %zu/%zu)", result->message.c_str(),
                 current_index_ + 1, nav_poses_.size());
    StopFallbackMotion();  // 终结路径统一停车（use_nav2_ 下为空操作）
    current_goal_->abort(result);
    current_goal_.reset();
    waiting_arrival_ = false;
    nav2_pending_ = false;
    return;
  }

  nav2_msgs::action::NavigateToPose::Goal nav_goal;
  nav_goal.pose = nav_poses_[current_index_];
  nav_goal.pose.header.stamp = now();
  nav2_pending_ = true;
  const std::weak_ptr<GoalHandleNavSeq> seq_goal = current_goal_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions opts;
  opts.feedback_callback =
      [this, seq_goal](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& gh,
                       const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>& fb) {
        Nav2FeedbackCb(gh, fb, seq_goal);
      };
  opts.result_callback =
      [this, seq_goal](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
        Nav2ResultCb(result, seq_goal);
      };
  nav2_client_->async_send_goal(nav_goal, opts);
  PublishSeqFeedback("NAVIGATING", 0.0);
  RCLCPP_INFO(get_logger(), "Sending waypoint %zu/%zu to %s", current_index_ + 1,
              nav_poses_.size(), nav2_action_name_.c_str());
}

void NavigateSequenceBridgeNode::Nav2FeedbackCb(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& /*gh*/,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>& fb,
    const std::weak_ptr<GoalHandleNavSeq>& seq_goal) {
  // 身份校验：只处理仍属于当前 NavigateSequence goal 的在途反馈
  const auto locked = seq_goal.lock();
  if (!locked || locked != current_goal_ || !nav2_pending_) {
    return;
  }
  PublishSeqFeedback("NAVIGATING", fb ? static_cast<double>(fb->feedback.distance_remaining) : 0.0);
}

void NavigateSequenceBridgeNode::Nav2ResultCb(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result,
    const std::weak_ptr<GoalHandleNavSeq>& seq_goal) {
  const auto locked = seq_goal.lock();
  if (!locked || locked != current_goal_ || !nav2_pending_) {
    return;  // 已被 cancel 终结 / 已换 goal：丢弃过期结果
  }
  nav2_pending_ = false;
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
      result.result->success) {
    PublishSeqFeedback("AT_STATION", 0.0);
    AdvanceAfterArrival();
    return;
  }
  // 单点失败 -> 整 goal abort
  failed_waypoints_++;
  auto res = std::make_shared<NavigateSequence::Result>();
  res->success = false;
  res->message = "navigate_to_pose failed at waypoint " + std::to_string(current_index_ + 1);
  res->completed_waypoints = completed_waypoints_;
  res->failed_waypoints = failed_waypoints_;
  RCLCPP_ERROR(get_logger(), "%s (code=%d)", res->message.c_str(),
               static_cast<int>(result.code));
  StopFallbackMotion();  // 终结路径统一停车（use_nav2_ 下为空操作）
  current_goal_->abort(res);
  current_goal_.reset();
  waiting_arrival_ = false;
}
#endif  // AMR_HAVE_NAV2_MSGS

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::NavigateSequenceBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
