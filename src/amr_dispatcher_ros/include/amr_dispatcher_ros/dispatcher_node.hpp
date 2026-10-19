#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"
#include "amr_dispatcher_core/dispatcher/mission_comparator.hpp"
#include "amr_dispatcher_core/dispatcher/mission_event_log.hpp"
#include "amr_dispatcher_core/dispatcher/mission_queue.hpp"
#include "amr_dispatcher_core/dispatcher/recovery_policy.hpp"
#include "amr_dispatcher_core/dispatcher/station_topology.hpp"
#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"
#include "amr_dispatcher_core/path_tracking/geometry_utils.hpp"
#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"
#include "amr_dispatcher_core/planning/path_planner_interface.hpp"
#include "amr_dispatcher_core/planning/mock_linear_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "amr_dispatcher_interfaces/action/execute_mission.hpp"
#include "amr_dispatcher_interfaces/action/navigate_sequence.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"
#include "amr_dispatcher_interfaces/srv/cancel_order.hpp"
#include "amr_dispatcher_interfaces/srv/pause_mission.hpp"
#include "amr_dispatcher_interfaces/srv/release_resource.hpp"
#include "amr_dispatcher_interfaces/srv/reserve_resource.hpp"
#include "amr_dispatcher_interfaces/srv/resume_mission.hpp"
#include "amr_dispatcher_interfaces/srv/submit_order.hpp"
#include "amr_dispatcher_interfaces/srv/enqueue_mission.hpp"
#include "amr_dispatcher_interfaces/srv/cancel_queued_mission.hpp"
#include "amr_dispatcher_interfaces/srv/preempt_mission.hpp"
#include "amr_dispatcher_interfaces/srv/reprioritize_queued_mission.hpp"
#include "amr_dispatcher_interfaces/srv/estimate_mission_cost.hpp"
#include "amr_dispatcher_interfaces/srv/block_station_route.hpp"
#include "amr_dispatcher_interfaces/srv/detect_traffic_deadlock.hpp"
#include "amr_dispatcher_interfaces/srv/list_stations.hpp"
#include "amr_dispatcher_interfaces/srv/list_traffic_reservations.hpp"
#include "amr_dispatcher_interfaces/srv/reserve_facility_resource.hpp"
#include "amr_dispatcher_interfaces/srv/get_operator_snapshot.hpp"
#include "amr_dispatcher_interfaces/srv/validate_site_config.hpp"

// 行为树编排层：订单路由、出队派发门控、恢复与结果决策
#include "amr_dispatcher_bt/mission_recovery_behavior_tree.hpp"
#include "amr_dispatcher_bt/mission_result_behavior_tree.hpp"
#include "amr_dispatcher_bt/mission_runtime_behavior_tree.hpp"
#include "amr_dispatcher_bt/queued_mission_start_behavior_tree.hpp"
#include "amr_dispatcher_bt/submit_order_behavior_tree.hpp"

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/catalog/scenario_catalog.hpp"
#include "amr_dispatcher_core/catalog/business_order_catalog.hpp"
#include "amr_dispatcher_core/catalog/zone_catalog.hpp"
#include "amr_dispatcher_core/facility/facility_catalog.hpp"
#include "amr_dispatcher_core/facility/facility_reservation.hpp"
#include "amr_dispatcher_core/fleet/fleet_catalog.hpp"
#include "amr_dispatcher_core/fleet/fleet_runtime_state.hpp"
#include "amr_dispatcher_core/workflow/mission_profile.hpp"
#include "amr_dispatcher_core/workflow/mission_estimator.hpp"
#include "amr_dispatcher_core/workflow/queued_mission.hpp"
#include "amr_dispatcher_core/workflow/operator_snapshot.hpp"

namespace amr_dispatcher_ros {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DispatcherNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit DispatcherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DispatcherNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
  void SubmitOrderCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::SubmitOrder::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::SubmitOrder::Response> resp);
  void CancelOrderCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::CancelOrder::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::CancelOrder::Response> resp);
  void PauseMissionCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::PauseMission::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::PauseMission::Response> resp);
  void ResumeMissionCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ResumeMission::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ResumeMission::Response> resp);
  void ReserveResourceCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ReserveResource::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ReserveResource::Response> resp);
  void ReleaseResourceCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ReleaseResource::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ReleaseResource::Response> resp);

  // 12 Core Services
  void EnqueueMissionCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::EnqueueMission::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::EnqueueMission::Response> resp);
  void CancelQueuedMissionCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::CancelQueuedMission::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::CancelQueuedMission::Response> resp);
  void PreemptMissionCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::PreemptMission::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::PreemptMission::Response> resp);
  void ReprioritizeQueuedMissionCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ReprioritizeQueuedMission::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ReprioritizeQueuedMission::Response> resp);
  void EstimateMissionCostCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::EstimateMissionCost::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::EstimateMissionCost::Response> resp);
  void BlockStationRouteCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::BlockStationRoute::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::BlockStationRoute::Response> resp);
  void DetectTrafficDeadlockCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::DetectTrafficDeadlock::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::DetectTrafficDeadlock::Response> resp);
  void ListStationsCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ListStations::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ListStations::Response> resp);
  void ListTrafficReservationsCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ListTrafficReservations::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ListTrafficReservations::Response> resp);
  void ReserveFacilityResourceCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ReserveFacilityResource::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ReserveFacilityResource::Response> resp);
  void GetOperatorSnapshotCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::GetOperatorSnapshot::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::GetOperatorSnapshot::Response> resp);
  void ValidateSiteConfigCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ValidateSiteConfig::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ValidateSiteConfig::Response> resp);

  using ExecuteMission = amr_dispatcher_interfaces::action::ExecuteMission;
  using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecuteMission>;
  using NavigateSequence = amr_dispatcher_interfaces::action::NavigateSequence;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateSequence>;

  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ExecuteMission::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleExecute> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandleExecute> goal_handle);

  // ExecuteMission goal 终结方式：成功 succeed / 失败 abort / 取消 canceled
  enum class ExecuteFinish { kSucceed, kAbort, kCanceled };
  // 在 CompleteMission/FailMission/取消路径里查表终结 execute_goal_handles_ 中的 handle。
  // rclcpp_action 状态机约束：succeed/abort 仅 EXECUTING、canceled 仅 CANCELING，
  // 因此服务侧取消（未受理 action cancel）降级为 abort(error_code)。
  void FinishExecuteGoal(const std::string& mission_id, ExecuteFinish kind,
                         const std::string& message, std::int32_t error_code);
  // 服务侧/批量取消路径共用收口：终结 handle + 释放 HandleGoal 预占 +
  // 清 action 模式在途 NavigateSequence goal（CancelOrder/CancelQueued/整单批量复用）
  void CleanupMissionCancel(const std::string& mission_id, const std::string& reason);
  // 终态收口：遗忘任务的路由锁预留记账与派发起点
  //（Complete/Fail/取消/死锁取消/预占判负等路径共用，幂等）
  void ForgetMissionDispatchState(const std::string& mission_id);
  // 受理过的 action 取消（HandleCancel 返回 ACCEPT 后 goal 才进入 CANCELING）的延迟处理
  void ProcessActionCancels();

  // NavigateSequence 客户端回调（navigation_backend=action 派发链）
  void NavGoalResponseCb(const std::string& mission_id, const GoalHandleNavigate::SharedPtr& goal_handle);
  void NavFeedbackCb(const std::string& mission_id, const GoalHandleNavigate::SharedPtr& goal_handle,
                     const std::shared_ptr<const NavigateSequence::Feedback> feedback);
  void NavResultCb(const std::string& mission_id, const GoalHandleNavigate::WrappedResult& result);

  void ScheduleTick();
  void DeadlockTick();
  // 死锁快照组包（DeadlockTick 与 DetectTrafficDeadlockCb 共用，保证两处结论不分叉）：
  // 清点排队等待记录 + 活跃任务/进度 + 预约表 WFG 数据；不自带锁，调用方沿用各自约定
  amr_dispatcher_core::dispatcher::DeadlockSnapshot BuildDeadlockSnapshot();
  void PublishState();
  // 结果/取消/恢复后重新武装队列启动请求（行为树 runtime 门控的输入）
  void ReArmQueueStartRequest();
  void TrackingErrorCb(const amr_dispatcher_interfaces::msg::TrackingError::SharedPtr msg);
  void OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CompleteMission(const std::string& mission_id, const std::string& reason);
  void FailMission(const std::string& mission_id, const std::string& reason);

  // M7：线程模型约束——queue_（MissionQueue 非线程安全）、reservation_table_、
  // active_missions_ 与参数热更回调共享状态，安全性依赖 main() 的单线程
  // rclcpp::spin（所有回调串行执行）。若未来改用 MultiThreadedExecutor，
  // 必须先为上述共享状态补齐互斥。
  std::unique_ptr<amr_dispatcher_core::dispatcher::MissionQueue> queue_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::TrafficReservationTable> reservation_table_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::MissionEventLog> event_log_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::DeadlockDetector> deadlock_detector_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::RecoveryPolicy> recovery_policy_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::StationTopologyGraph> topology_graph_;

  std::unordered_map<std::string, amr_dispatcher_core::dispatcher::Mission> active_missions_;
  std::unordered_map<std::string, std::size_t> mission_attempts_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> mission_last_progress_;
  std::mutex active_missions_mutex_;
  std::uint64_t next_sequence_ = 1;
  bool deadlock_flag_ = false;
  // 最近一次 DeadlockTick 的环路径快照（findings[].cycle_path；无死锁时清空），
  // PublishState 据此投影 /dispatcher/topology_state 的死锁输入
  std::vector<std::vector<std::string>> last_deadlock_findings_;
  // 出队派发门控：由提单/入队置位，runtime 行为树消费（派发时清除）
  bool queue_start_requested_ = false;
  // 恢复策略回坞目标站点（由参数配置，缺失时回坞降级为仅事件记录）
  std::string recovery_dock_station_;

  // 任务执行闭环：站点目录（mtime 热重载） + 规划器 + 最新里程计位姿
  std::unique_ptr<amr_dispatcher_core::IPathPlanner> planner_;
  std::unique_ptr<amr_dispatcher_core::catalog::StationCatalogReloader> catalog_reloader_;
  amr_dispatcher_core::path_tracking::Pose2D current_pose_{};
  bool has_odom_ = false;
  std::string tracking_mission_id_;  // 当前正在跟踪 /plan 的任务（单车一次一个）

  rclcpp::Service<amr_dispatcher_interfaces::srv::SubmitOrder>::SharedPtr submit_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::CancelOrder>::SharedPtr cancel_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::PauseMission>::SharedPtr pause_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ResumeMission>::SharedPtr resume_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReserveResource>::SharedPtr reserve_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReleaseResource>::SharedPtr release_srv_;
  // ExecuteMission action server（HandleGoal 校验 + HandleAccepted 入队，不起线程）
  rclcpp_action::Server<ExecuteMission>::SharedPtr execute_action_server_;
  // NavigateSequence 客户端（navigation_backend=action 时向桥接节点发单点 goal）
  rclcpp_action::Client<NavigateSequence>::SharedPtr nav_client_;
  // mission_id -> 仍在执行的 ExecuteMission goal handle（终态时查表 finish 并移除）
  std::unordered_map<std::string, std::shared_ptr<GoalHandleExecute>> execute_goal_handles_;
  // mission_id -> 派发起点（PublishState 里按直线距离折算 percent_complete）
  std::unordered_map<std::string, amr_dispatcher_core::path_tracking::Pose2D> mission_dispatch_start_;
  // mission_id -> 派发时刻（ExecuteMission result.total_duration_sec 计算用，
  // 终态时在 FinishExecuteGoal/CompleteMission/FailMission/取消收口里遗忘）
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> mission_dispatch_time_;
  // 导航后端：plan（默认，原 /plan 路径不变）| action（发 /navigate_sequence 单点 goal）
  std::string navigation_backend_{"plan"};
  // action 模式在途导航 goal 的归属任务与句柄（取消/终态时级联 async_cancel_goal）
  std::string pending_nav_mission_id_;
  GoalHandleNavigate::SharedPtr pending_nav_goal_handle_;

  // Peek→Reserve→Pop：排队任务预留路由锁失败 -> 该任务仍在等的锁列表
  //（喂 DeadlockSnapshot.allocation_graph.waiting_for，终态/取消时遗忘）
  std::unordered_map<std::string, std::vector<std::string>> pending_wait_;
  // mission_id -> 路由锁预留成功时刻（喂 DeadlockSnapshot.reservation_acquired_at，
  // 长持锁启发式取证；仅任务派发预留记账，人工 ReserveResourceCb 不记，避免误报）
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> mission_reserved_at_;

  // 12 Core Services SharedPtrs
  rclcpp::Service<amr_dispatcher_interfaces::srv::EnqueueMission>::SharedPtr enqueue_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::CancelQueuedMission>::SharedPtr cancel_queued_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::PreemptMission>::SharedPtr preempt_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReprioritizeQueuedMission>::SharedPtr reprioritize_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::EstimateMissionCost>::SharedPtr estimate_cost_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::BlockStationRoute>::SharedPtr block_route_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::DetectTrafficDeadlock>::SharedPtr detect_deadlock_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ListStations>::SharedPtr list_stations_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ListTrafficReservations>::SharedPtr list_reservations_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReserveFacilityResource>::SharedPtr reserve_facility_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::GetOperatorSnapshot>::SharedPtr snapshot_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ValidateSiteConfig>::SharedPtr validate_site_srv_;

  rclcpp_lifecycle::LifecyclePublisher<amr_dispatcher_interfaces::msg::DispatcherState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<amr_dispatcher_interfaces::msg::MissionEvent>::SharedPtr event_pub_;
  rclcpp_lifecycle::LifecyclePublisher<amr_dispatcher_interfaces::msg::TopologyState>::SharedPtr topology_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

  rclcpp::Subscription<amr_dispatcher_interfaces::msg::TrackingError>::SharedPtr tracking_error_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr schedule_timer_;
  rclcpp::TimerBase::SharedPtr deadlock_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr reload_timer_;  // 1Hz 驱动站点目录 mtime 热重载
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace amr_dispatcher_ros
