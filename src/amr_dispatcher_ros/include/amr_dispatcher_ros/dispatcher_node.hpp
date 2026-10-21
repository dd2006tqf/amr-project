#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
#include "amr_dispatcher_core/fleet/force_clear_link.hpp"
#include "amr_dispatcher_core/fleet/fleet_catalog.hpp"
#include "amr_dispatcher_core/fleet/vehicle_assignment.hpp"
#include "amr_dispatcher_core/workflow/time_space_planning.hpp"
#include "amr_dispatcher_core/path_tracking/geometry_utils.hpp"
#include "amr_dispatcher_core/path_tracking/pure_pursuit.hpp"
#include "amr_dispatcher_core/planning/path_planner_interface.hpp"
#include "amr_dispatcher_core/planning/mock_linear_planner.hpp"
#include "amr_dispatcher_core/planning/skeleton_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "amr_dispatcher_interfaces/action/execute_mission.hpp"
#include "amr_dispatcher_interfaces/action/navigate_sequence.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"
#include "amr_dispatcher_interfaces/srv/cancel_order.hpp"
#include "amr_dispatcher_interfaces/srv/force_clear_vda5050_link.hpp"
#include "amr_dispatcher_interfaces/srv/restore_vda5050_link.hpp"
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

  // Task 7（终审 I1 闭合）：人工强制清除离线 VDA 5050 链路——摘除离线挂载
  //（解除 EnforceVda5050LinkGuard 挂起）并释放该车名下全部路权锁（Spec §5.2
  // "人工强解"半边）。仅受理 online==false 的受管车辆，防误杀活车。
  void ForceClearVda5050LinkCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::ForceClearVda5050Link::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::ForceClearVda5050Link::Response> resp);
  // Task 7（评审 P0.5）：解除隔离，使车辆重新参与自动分配。
  // 准入：须已隔离 + 在线 + 操作员声明已核实车辆物理位置。
  void RestoreVda5050LinkCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::RestoreVda5050Link::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::RestoreVda5050Link::Response> resp);

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
  // 依 planner_type_ + 当前站点目录重建 planner_（on_configure 与参数热更共用）
  void RebuildPlanner();
  void DeadlockTick();
  // 死锁快照组包（DeadlockTick 与 DetectTrafficDeadlockCb 共用，保证两处结论不分叉）：
  // 清点排队等待记录 + 活跃任务/进度 + 预约表 WFG 数据；不自带锁，调用方沿用各自约定
  amr_dispatcher_core::dispatcher::DeadlockSnapshot BuildDeadlockSnapshot();
  void PublishState();
  // 结果/取消/恢复后重新武装队列启动请求（行为树 runtime 门控的输入）
  void ReArmQueueStartRequest();
  void TrackingErrorCb(const amr_dispatcher_interfaces::msg::TrackingError::SharedPtr msg);
  void OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ---- P0.6 / A4：per-vehicle 话题与 TF 隔离（多车并发基础）----
  // vehicle_ids 参数（逗号分隔）列出本调度器管理的 local_sim 车辆；
  // 为每台车建立 /plan/{vid} 发布端、/odom/{vid} 与 /tracking/error/{vid} 订阅。
  // 兼容性红线：vehicle_ids 为空（默认）时，plan_pub_ 与全局订阅行为逐字不变。
  void SetupVehicleTopics(const std::string& vehicles_param);
  // 按车辆 id 取计划发布端：非空且已建 -> /plan/{vid} 的发布端；否则回落到 legacy /plan。
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr PlanPublisherForVehicle(
      const std::string& vehicle_id);
  // 发布计划到指定车辆：vehicle_id 非空且有专属发布端 -> /plan/{vid}；否则 -> legacy /plan。
  void PublishPlanForVehicle(const std::string& vehicle_id, const nav_msgs::msg::Path& path);
  // per-vehicle 里程计：只更新 vehicle_poses_，不动 legacy 的 current_pose_/has_odom_。
  void OdomCbForVehicle(const std::string& vehicle_id, const nav_msgs::msg::Odometry::SharedPtr msg);
  // per-vehicle 跟踪误差回调：把 arrived 信号路由到"该车正在执行的任务"并完成它。
  void TrackingErrorCbForVehicle(const std::string& vehicle_id,
                                 const amr_dispatcher_interfaces::msg::TrackingError::SharedPtr msg);
  // 派发目标车辆解析：显式指定优先（必须在受管清单内），否则由选车策略决定。
  std::string ResolveDispatchVehicleId(const amr_dispatcher_core::dispatcher::Mission& mission);
  // 向"执行该任务的那台车"补发空路径停车。
  // legacy 模式（vehicle_id 为空）-> 回落 plan_pub_，与改动前逐字等价。
  void PublishStopPathForMission(const std::string& mission_id);

  // ---- B3：时空窗交通协调 ----
  // 交通协调模式：static（静态排他锁，现网默认）| time_space（时空窗预约）
  std::string traffic_mode_{"static"};
  // 运动时序参数（把几何长度换算成时间占用区间）
  amr_dispatcher_core::workflow::MotionTimingConfig time_space_timing_{};
  // 时空窗占用表（仅 time_space 模式使用）：resource_id -> 区间列表
  amr_dispatcher_core::workflow::TimeSpaceOccupancy time_space_occupancy_;
  // mission_id -> 该单已预约的时间窗（延期/释放时用）
  std::unordered_map<std::string, amr_dispatcher_core::workflow::ResourceTimeWindowSet>
      time_space_windows_;
  // 是否启用时空窗模式
  bool UseTimeSpaceMode() const { return traffic_mode_ == "time_space"; }
  // 为一单规划并【整条路径原子提交】时空窗预约；失败则整体回滚。
  bool ReserveTimeSpaceForMission(const std::string& mission_id,
                                  const std::vector<std::string>& path_node_ids,
                                  std::string* message);
  // 车辆确认驶离某资源后释放其时间窗（窗口到期 != 资源已释放）。
  void ReleaseTimeSpaceForResources(const std::string& mission_id,
                                    const std::vector<std::string>& resource_ids);
  // 任务终态时全清其时空窗占用。
  void ReleaseAllTimeSpaceForMission(const std::string& mission_id);

  // ---- P0.6 / A6：受管车目录 + 选车上下文 ----
  // 车辆静态配置（fleet.yaml）+ 动态状态（里程计 / VDA 5050 state）合成的运行时视图。
  // 该表是选车（PlanVehicleAssignment）的唯一输入来源（评审问题 3：数据源闭合）。
  std::unordered_map<std::string, amr_dispatcher_core::fleet::VehicleRuntimeState> vehicle_states_;
  // 选车所需能力（默认 transport）。
  std::string required_capability_{"transport"};
  // 选车最低电量阈值（百分比）。
  double min_battery_percent_{0.0};
  // 选车状态最大时延（毫秒）：超龄 -> 状态不确定。
  int64_t vehicle_state_max_age_ms_{5000};
  // 当前是否处于"状态不确定"导致的自动分配暂停（kIndeterminate）。
  bool auto_assignment_paused_{false};

  // 选车永久拒绝的排队任务收口：出队 + 释放预留 + 事件 + 重新武装启动请求。
  void RejectedQueuedMission(const amr_dispatcher_core::dispatcher::Mission& mission,
                             const std::string& reason);
  // 组装选车上下文：从 vehicle_states_ + 站点目录算"车到取货点"的路网成本。
  amr_dispatcher_core::fleet::VehicleAssignmentRequest BuildAssignmentRequest(
      const amr_dispatcher_core::dispatcher::Mission& mission);
  // 按 mission.assigned_vehicle_id / 选车策略决定承接车辆。
  // 返回值语义与核心三态一致（kAssigned/kDeferred/kIndeterminate/kRejected）。
  amr_dispatcher_core::fleet::VehicleAssignmentDecision SelectVehicleForMission(
      const amr_dispatcher_core::dispatcher::Mission& mission);
  // 刷新车辆动态状态（里程计 / VDA 5050 state -> position_valid / state_time / 电量）。
  void RefreshVehicleStates();
  // 把车辆目录（fleet.yaml）合并进 vehicle_states_ 的静态字段。
  void LoadFleetCatalog();

  // ---- P0.6 / A5：per-vehicle 执行槽位 ----
  // 车辆 -> 正在执行的任务（key 为空串 = legacy 全局 /plan 槽位，单车兼容）。
  std::unordered_map<std::string, std::string> vehicle_current_mission_;
  // 任务 -> 执行该任务的车（与上表互为逆映射；终态时成对清除）。
  std::unordered_map<std::string, std::string> mission_tracking_vehicle_;
  // 占用执行槽位（派发时调用）：同一台车已有在途任务时返回 false（不得抢占）。
  bool AssignTrackingSlot(const std::string& mission_id, const std::string& vehicle_id);
  // 释放执行槽位（终态/取消时调用，幂等）；返回被释放的车辆 id（空 = 无槽位）。
  std::string ReleaseTrackingSlot(const std::string& mission_id);
  // 该任务当前是否占用执行槽位。
  bool IsMissionTracked(const std::string& mission_id) const;
  // 该车当前是否在执行任务。
  bool IsVehicleBusy(const std::string& vehicle_id) const;
  // 是否还有空闲的受管车（无可用槽位 -> mission_active=true）。
  // 无受管车清单时回落 legacy 单槽位语义。
  bool HasFreeVehicleSlot() const;
  // 是否配置了受管车清单（local_sim 或 vda5050）。空 = legacy 单车模式。
  bool HasAnyManagedVehicle() const;

  // Task 5：VDA 5050 状态/链路接入（Spec §5.2 断链守卫 + §5.3 逐段解锁）
  // 按参数 vda5050_vehicles（逗号分隔车辆 id 列表，空 = 关闭）为每台受管小车
  // 建立 /fleet/{vid}/state 与 /fleet/{vid}/online 订阅（on_configure 时一次建齐）。
  void SetupVda5050Links(const std::string& vehicles_param);
  void Vda5050StateCb(const std::string& vehicle_id,
                      const std_msgs::msg::String::SharedPtr msg);
  void Vda5050OnlineCb(const std::string& vehicle_id,
                       const std_msgs::msg::Bool::SharedPtr msg);
  // 每 tick 的链路守卫巡检：离线 -> 挂起新派发（返回 true 表示阻塞）；
  // 在线但 state 陈旧 -> 软看门狗告警（不阻塞、不释放）
  bool EnforceVda5050LinkGuard();
  // Task 6（Task 5 评审 I2）：派发时把存档行驶路径 + 本单名下持锁 构建成
  // VDA 5050 订单，经 /fleet/{vid}/send_order 交给网关下发 MQTT order 话题。
  // 车辆绑定 = vda5050_vehicle_ids_ 上 round-robin（保序向量，不依赖
  // unordered_map 遍历序）；orderId = mission_id，orderUpdateId 按车递增。
  void PublishVda5050Order(const std::string& mission_id,
                           const std::string& assigned_vehicle_id = "");
  // mission_id 按值接收：调用方可能传入 active_missions_ 的键引用，
  // 而本函数会 erase 该条目（引用会悬空）。
  void CompleteMission(std::string mission_id, const std::string& reason);
  void FailMission(std::string mission_id, const std::string& reason);

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
  // 路径规划器类型（配置参数 planner_type：skeleton | mock）。
  // skeleton -> SkeletonPlanner（混合图折线循迹），mock -> MockLinearPlanner（直线基线）。
  std::string planner_type_{"skeleton"};
  // 地图版本门禁：非空时要求 catalog.map_hash 与之相等，否则暂停出队派发
  std::string expected_map_hash_;
  // 版本门禁状态（每次 ScheduleTick 重算；false = 地图哈希不匹配，派发已暂停）
  bool map_version_valid_{true};
  amr_dispatcher_core::path_tracking::Pose2D current_pose_{};
  bool has_odom_ = false;

  rclcpp::Service<amr_dispatcher_interfaces::srv::SubmitOrder>::SharedPtr submit_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::CancelOrder>::SharedPtr cancel_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::PauseMission>::SharedPtr pause_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ResumeMission>::SharedPtr resume_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReserveResource>::SharedPtr reserve_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReleaseResource>::SharedPtr release_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ForceClearVda5050Link>::SharedPtr
      force_clear_vda5050_link_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::RestoreVda5050Link>::SharedPtr
      restore_vda5050_link_srv_;
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

  // ---- Task 5：逐段路权动态释放与断链安全守卫（Spec §5.2/§5.3）----
  // 单台受管 VDA 5050 小车的链路态（单线程 spin，无并发；M7 约束适用）
  struct Vda5050Link {
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr online_sub;
    bool online = true;  // 乐观初值：online=false 到达前不阻塞派发（与网关 transient_local 重放收敛）
    bool seen_state = false;
    std::chrono::steady_clock::time_point last_state_time{};  // state 陈旧检测
    // 最近一帧 state 的定位与电量（选车输入：位置不可用即不合格）
    bool position_valid = false;
    double position_x = 0.0;
    double position_y = 0.0;
    double battery_percent = 100.0;
  };
  // vehicle_id -> 链路态（key 即 vda5050_vehicles 解析后的 id，保序去重）
  std::unordered_map<std::string, Vda5050Link> vda5050_links_;
  // vehicle_id -> 隔离态集合（人工强清后置入；恢复时移除）。
  // 隔离车仍可见/可审计，但不参与自动分配（选车时视为不合格）。
  std::unordered_set<std::string> vda5050_quarantined_;
  // mission_id -> 派发时刻的行驶路径（travel order；Peek 阶段只留锁列表，路径在此存档，
  // 供 state 回推身后锁；终态经 ForgetMissionDispatchState 遗忘）
  std::unordered_map<std::string, std::vector<std::string>> mission_route_paths_;
  // mission_id -> 承接该单的受管车辆（PublishVda5050Order 下发时登记）。
  // 人工强清离线链路时按此反查"该车名下"的 mission —— 不能靠"路径存档存在"推断，
  // 那会把其他车辆的 mission 锁一并误判为该车名下（多车场景误释放）。
  std::unordered_map<std::string, std::string> vda5050_mission_vehicle_;
  // state 陈旧阈值（秒）：在线但超时未上报 -> 软看门狗告警（不释放、不阻塞）
  double vda5050_state_stale_sec_{5.0};

  // ---- Task 6：VDA 5050 订单发布端（补 Task 5 评审 I2 的发布链路缺口）----
  // 保序受管车辆清单（SetupVda5050Links 与 vda5050_links_ 同步填充；
  // round-robin 绑定需要确定性顺序，unordered_map 遍历序不可依赖）
  std::vector<std::string> vda5050_vehicle_ids_;
  // vehicle_id -> send_order 发布端（on_configure 按清单一次建齐，on_activate 激活；
  // 网关侧订阅是懒建的，ROS 2 发布端不依赖对端存在）
  std::unordered_map<std::string, rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr>
      vda5050_order_pubs_;
  // vehicle_id -> 下一单 order_update_id（同车每派一次递增，VDA 5050 更新语义）
  std::unordered_map<std::string, std::uint32_t> vda5050_next_order_update_id_;
  // round-robin 游标（按车推进；键缺失时回绕到 0）
  std::size_t vda5050_rr_cursor_ = 0;

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

  // ---- P0.6 / A4：per-vehicle 话题隔离 ----
  // vehicle_ids 清单（保序去重，来自参数 vehicle_ids；空 = 单车兼容模式）。
  std::vector<std::string> vehicle_ids_;
  // vehicle_id -> /plan/{vid} 发布端（on_configure 建，on_activate 激活，on_cleanup 释放）
  std::unordered_map<std::string, rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr>
      vehicle_plan_pubs_;
  // vehicle_id -> /odom/{vid} 订阅
  std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
      vehicle_odom_subs_;
  // vehicle_id -> /tracking/error/{vid} 订阅
  std::unordered_map<std::string, rclcpp::Subscription<amr_dispatcher_interfaces::msg::TrackingError>::SharedPtr>
      vehicle_tracking_error_subs_;
  // vehicle_id -> 最新里程计位姿（Pose2D + has_odom 标志）。
  // 与 legacy 的 current_pose_/has_odom_ 并存：全局 /odom 仍只喂 legacy 字段。
  struct VehiclePose {
    amr_dispatcher_core::path_tracking::Pose2D pose{};
    bool has_odom = false;
  };
  std::unordered_map<std::string, VehiclePose> vehicle_poses_;

  rclcpp::TimerBase::SharedPtr schedule_timer_;
  rclcpp::TimerBase::SharedPtr deadlock_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr reload_timer_;  // 1Hz 驱动站点目录 mtime 热重载
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace amr_dispatcher_ros
