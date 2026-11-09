#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"
#include "amr_dispatcher_core/dispatcher/dispatch_state_snapshot.hpp"
#include "amr_dispatcher_core/dispatcher/mission_comparator.hpp"
#include "amr_dispatcher_core/dispatcher/mission_event_log.hpp"
#include "amr_dispatcher_core/dispatcher/mission_queue.hpp"
#include "amr_dispatcher_core/dispatcher/recovery_policy.hpp"
#include "amr_dispatcher_core/dispatcher/station_topology.hpp"
#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"
#include "amr_dispatcher_core/fleet/force_clear_link.hpp"
#include "amr_dispatcher_core/fleet/fleet_catalog.hpp"
#include "amr_dispatcher_core/fleet/vda5050_link_state.hpp"
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
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "amr_dispatcher_interfaces/action/execute_mission.hpp"
#include "amr_dispatcher_interfaces/action/navigate_sequence.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"
#include "amr_dispatcher_ros/proto_lifecycle_dual_publisher.hpp"
// 阶段 3：TrackingError 双订阅（proto 分支内部反投影回 rosidl）。
#include "amr_dispatcher_proto_bridge/proto_dual_subscriber.hpp"
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
#include "amr_dispatcher_interfaces/srv/block_edge.hpp"
#include "amr_dispatcher_interfaces/srv/topology_hot_update.hpp"
#include "amr_dispatcher_interfaces/srv/get_topology_overlay.hpp"

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
#include "amr_dispatcher_core/catalog/topology_connectivity.hpp"
#include "amr_dispatcher_core/catalog/topology_overlay.hpp"
#include "amr_dispatcher_core/workflow/topology_change_impact.hpp"
#include "amr_dispatcher_core/facility/facility_adapter_interface.hpp"
#include "amr_dispatcher_core/facility/facility_catalog.hpp"
#include "amr_dispatcher_core/facility/facility_interlocking_planning.hpp"
#include "amr_dispatcher_core/facility/facility_reservation.hpp"
#include "amr_dispatcher_core/workflow/cbs_traffic_coordinator.hpp"
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

  // ---- 相位 11：在线拓扑热生效（目录感知封路 / 全量叠加层 / 现状查询）----
  // 与 BlockStationRouteCb 的区别：老接口只在预约表置 traffic_block 锁，规划层
  // 仍认为这条路存在；新接口同时改**生效目录**（enabled/bidirectional），
  // 使规划、骨架折线、距离估算、时空窗/CBS 全部一致地绕开该边。
  void BlockEdgeCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::BlockEdge::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::BlockEdge::Response> resp);
  void TopologyHotUpdateCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::TopologyHotUpdate::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::TopologyHotUpdate::Response> resp);
  void GetTopologyOverlayCb(
      const std::shared_ptr<amr_dispatcher_interfaces::srv::GetTopologyOverlay::Request> req,
      std::shared_ptr<amr_dispatcher_interfaces::srv::GetTopologyOverlay::Response> resp);
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
  // 批次 6.2：低电量自动插充电单（每 tick 检查一次，去重按"车"而非"站点"）。
  // 决策在 core（PlanAutoCharging，纯函数）；这里只做采集与入队。
  void PlanAutoChargingTick();
  // 该车当前是否已有充电单（排队或在途）——去重键
  bool HasPendingChargeMission(const std::string& vehicle_id) const;
  // ScheduleTick 的三段（机械切分，语义与切分前逐字一致）：
  //   1. ScheduleGatesPass：链路守卫 / 容量 / 空闲槽位 / 地图版本 / BT 启动请求
  //   2. SelectReadyCandidate：PeekN 扫描 + 三态处理 + 队首阻塞规避 + 锁预留
  //   3. ExecuteDispatch：出队落定 + 起终点解析 + 下发（/plan、VDA 订单、action）
  bool ScheduleGatesPass();
  // 已选到车且预留成功的候选（三段之间传递的唯一载荷）
  struct ReadyCandidate {
    amr_dispatcher_core::dispatcher::Mission mission;
    std::string vehicle_id;
    std::vector<std::string> route_locks;
  };
  std::optional<ReadyCandidate> SelectReadyCandidate();
  void ExecuteDispatch(ReadyCandidate candidate);
  // 依 planner_type_ + 当前站点目录重建 planner_（on_configure 与参数热更共用）
  void RebuildPlanner();

  // ---- 相位 11：生效目录与拓扑叠加层 ----
  // 生效目录 = 基础目录（map_ingest 写的 stations.yaml）+ 运行时叠加层（施工区/单向/临时站点）。
  // 无叠加层时直接返回 reloader 的目录引用（零拷贝，行为逐字不变）；
  // 有叠加层时返回 effective_catalog_（ApplyTopologyOverlay 的产物）。
  // 红线：比较地图版本 / 写状态快照时必须用 **基础** hash（catalog_reloader_->catalog()），
  // 否则一次施工区变更就会让下次重启丢队列。
  const amr_dispatcher_core::catalog::StationCatalog& EffectiveCatalog() const;
  bool CatalogLoaded() const;
  // on_configure 时从磁盘加载叠加层并按结论改 effective_catalog_ ：
  //   * 文件不存在 -> no overlay（不是错误）
  //   * 文件损坏 -> 拒绝加载（保留基线，不改名，与"坏快照不清理现场"同一姿态）
  //   * base_map_hash 与新基线不一致 -> 失效（rename .stale.<epoch> + 审计 + 事件）
  //   * 成功 -> ApplyTopologyOverlay + ReconcileResourceBlocksWithOverlay
  // map_ingest 可能晚于 dispatcher 写完 stations.yaml：此时叠加层会
  // 被标为 overlay_deferred_，由 reload_timer_ 在下次 Tick 补加载。
  void LoadTopologyOverlayFromDisk();
  // 基础目录变化时（reload_timer_）：叠加层要么按新基线重应用，要么整体失效 ——
  // 绝不允许热重载静默丢掉施工区。
  void ReconcileOverlayAfterBaseReload();
  // 提交成功后把叠加层表达的封路同步进预约表（traffic_block:topology_overlay:* owner），
  // 使 list_traffic_reservations / 死锁检测能看到这批临时管制。
  void ReconcileResourceBlocksWithOverlay();
  // 把 reroute 决策真正下发到执行该任务的车（plan 话题或 VDA 5050 订单）。
  // 从 DeadlockTick 抽出：死锁日志文案留在 DeadlockTick 原位，本函数只负责发布段。
  void ApplyRerouteToVehicle(
      const std::string& mission_id,
      const amr_dispatcher_core::workflow::VehicleImpact& impact);

  // 叠加层提交计划（BuildTopologyCommitPlan 的产物；Build 零副作用，Commit 才变更）。
  struct TopologyCommitPlan {
    std::string error_code = "ok";        // ok | no_change | invalid_request | not_confirmed | ...
    std::string message;
    // 审计字段（操作员与原因必须原样进审计，不能拿时间戳顶替 —— 事后追责全靠它）
    std::string operator_id;
    std::string reason;
    // 审计动作名：单边管制与全量提交分开记，现场才能一眼分辨"封了一条"与"整份替换"
    std::string audit_action = "topology_hot_update";
    amr_dispatcher_core::catalog::TopologyOverlay candidate;  // 已补 base_map_hash/revision/updated_at
    amr_dispatcher_core::catalog::StationCatalog new_catalog;
    amr_dispatcher_core::catalog::ReachabilityReport connectivity;
    amr_dispatcher_core::workflow::TopologyChangeImpact impact;
    // 要置为 traffic_block 的新边（含 reason），以及要解封的叠加层锁
    std::vector<std::pair<std::string, std::string>> edges_to_block;
    std::vector<std::string> edges_to_unblock;
    // allow_edge_under_vehicle=true 时"车可能正压在上面"的边：跳过 Block，锁留给在途车
    std::vector<std::string> retained_edge_locks;
    bool no_change = false;
  };
  // 零副作用：功能开关 -> 目录可用性 -> map_hash 乐观并发 -> revision/时间戳 -> 幂等 ->
  // 校验 -> 应用 + 连通性 -> 在途影响 -> 锁预检。任何一步失败立即返回，不碰内存/磁盘。
  TopologyCommitPlan BuildTopologyCommitPlan(
      const amr_dispatcher_core::catalog::TopologyOverlay& requested,
      const std::string& expected_map_hash, const std::string& operator_id,
      const std::string& reason, bool allow_edge_under_vehicle);
  // 唯一变更函数。顺序即安全铁律：先持久化（写前提交点），再预留新锁，再释放旧锁，
  // 再置叠加层锁（Block 在资源被持有时会失败，必须在 Release 之后），
  // 最后换目录 + 同步规划器 + 下发绕行 + 审计 + 事件。
  bool CommitTopologyPlan(TopologyCommitPlan* plan, std::string* error_code,
                          std::string* message);
  void DeadlockTick();
  // 死锁快照组包（DeadlockTick 与 DetectTrafficDeadlockCb 共用，保证两处结论不分叉）：
  // 清点排队等待记录 + 活跃任务/进度 + 预约表 WFG 数据；不自带锁，调用方沿用各自约定
  amr_dispatcher_core::dispatcher::DeadlockSnapshot BuildDeadlockSnapshot();
  void PublishState();
  // ---- 批次 2：调度状态持久化与审计落盘（全部默认关闭）----
  // 变更后打脏标记；ScheduleTick 里按 debounce 窗口合并写盘（不阻塞派发节拍）
  void MarkStateDirty();
  // 立即落盘（on_deactivate / on_shutdown / 恢复失败等需要确定性的场合）
  void FlushStateSnapshot();
  // 组一份当前状态的快照（不做 IO）
  amr_dispatcher_core::dispatcher::DispatchStateSnapshot BuildStateSnapshot();
  // 追加一条审计记录（audit_log_file 为空时只打日志）
  void AppendAudit(const std::string& action, const std::string& vehicle_id,
                   const std::string& mission_id, const std::string& operator_id,
                   const std::string& reason,
                   const std::vector<std::string>& released_resources = {});
  static std::string IsoTimestampNow();
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
  // 是否启用带时间窗模式 (包含 time_space 与 cbs)
  bool UsesTimeWindows() const { return traffic_mode_ == "time_space" || traffic_mode_ == "cbs"; }
  bool UseTimeSpaceMode() const { return traffic_mode_ == "time_space"; }
  bool UseCBSMode() const { return traffic_mode_ == "cbs"; }
  // 为一单规划并【整条路径原子提交】时空窗预约；失败则整体回滚。
  bool ReserveTimeSpaceForMission(const std::string& mission_id,
                                  const std::vector<std::string>& path_node_ids,
                                  std::string* message);
  // Phase 9: 为一单执行 CBS 协同规划并提交时空窗
  bool ReserveCBSForMission(const std::string& mission_id,
                            const std::string& pickup_station,
                            const std::string& dropoff_station,
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
  // 派发计数器（累积；供 /dispatcher/state 与 /metrics）
  struct DispatchCounters {
    std::uint64_t dispatch_total = 0;
    std::uint64_t dispatch_rejected_total = 0;
    std::uint64_t dispatch_deferred_total = 0;
    std::uint64_t lock_contention_total = 0;
  };
  DispatchCounters dispatch_counters_;
  // 自动选车打分策略（参数 vehicle_selection_strategy；未知值回落到 nearest）
  amr_dispatcher_core::fleet::VehicleSelectionStrategy vehicle_selection_strategy_{
      amr_dispatcher_core::fleet::VehicleSelectionStrategy::kNearest};
  // 批次 6.2：低电量自动插充电单（参数 auto_charging_*；默认关闭）
  bool auto_charging_enabled_{false};
  std::string auto_charging_station_;
  double auto_charging_opportunity_threshold_percent_{30.0};
  double auto_charging_critical_threshold_percent_{15.0};
  int auto_charging_opportunity_priority_{0};
  int auto_charging_critical_priority_{5};
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

  // ---- T1：电量链路（/battery_state -> vehicle_states_[].battery_percent）----
  // 此前 /battery_state 没有任何订阅者，local_sim 车的 battery_percent 恒为 100，
  // 于是 min_battery_percent 与 auto_charging_* 全部静默失效。
  // 这里按 per-vehicle 话题订阅底盘电压，换算成百分比写回 vehicle_states_，
  // 并保留"最后一次可信样本"的时间戳，供 RefreshVehicleStates 判新鲜度。
  void BatteryStateCbForVehicle(const std::string& vehicle_id,
                                const sensor_msgs::msg::BatteryState::SharedPtr msg);
  // 电量样本的保鲜期（毫秒）：超龄样本不再参与选车与自动充电决策。
  // 必须小于 vehicle_state_max_age_ms，否则会给调度一个"看起来很新"的陈旧电量。
  std::int64_t battery_state_timeout_ms_{3000};
  // vehicle_id -> /battery_state/{vid} 订阅
  std::unordered_map<std::string,
                     rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr>
      vehicle_battery_subs_;
  // vehicle_id -> 最近一帧可信电量样本
  struct BatterySample {
    double percent = 0.0;
    std::chrono::steady_clock::time_point stamp{};
    bool valid = false;
  };
  std::unordered_map<std::string, BatterySample> vehicle_battery_samples_;

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
  // ---- VDA 5050 链路态查询/变更（隔离位存在链路态表里，避免两处各存一份）----
  bool IsManagedVda5050Vehicle(const std::string& vehicle_id) const;
  bool IsVehicleOnline(const std::string& vehicle_id) const;
  bool IsVehicleQuarantined(const std::string& vehicle_id) const;
  // 返回 false 表示该车不在受管清单（快照里的陈旧隔离记录属于这种情况）
  bool SetVehicleQuarantined(const std::string& vehicle_id, bool quarantined);
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

  std::unordered_map<std::string, amr_dispatcher_core::dispatcher::Mission> active_missions_;
  std::unordered_map<std::string, std::size_t> mission_attempts_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> mission_last_progress_;
  std::mutex active_missions_mutex_;
  std::uint64_t next_sequence_ = 1;

  // ---- 批次 2：状态持久化（state_snapshot_file 为空 = 关闭，默认关闭）----
  // 变更打脏 + 防抖合并写盘：高频派发/完成不会被翻译成高频磁盘写
  std::string state_snapshot_file_;
  double state_snapshot_debounce_sec_ = 2.0;
  std::string audit_log_file_;
  bool state_snapshot_dirty_ = false;
  std::chrono::steady_clock::time_point state_snapshot_dirty_at_{};
  // 快照里的隔离车辆：on_configure 先记下，等 SetupVda5050Links 建完链路再灌回去
  std::vector<std::string> restored_quarantined_;
  std::uint64_t state_snapshot_writes_ = 0;
  std::uint64_t state_snapshot_failures_ = 0;
  std::string last_state_snapshot_error_;
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

  // ---- 相位 11：运行时拓扑叠加层状态 ----
  // effective_catalog_ 由 base + overlay_ 合成；无叠加层时留空，EffectiveCatalog()
  // 直接回落给 catalog_reloader_->catalog()。
  amr_dispatcher_core::catalog::StationCatalog effective_catalog_;
  amr_dispatcher_core::catalog::TopologyOverlay overlay_;
  bool overlay_active_{false};
  bool overlay_loaded_{false};
  bool overlay_deferred_{false};  // map_ingest 晚写 stations.yaml，由 reload_timer_ 补加载
  std::string topology_overlay_file_;  // 参数 topology_overlay_file（默认空 = 关闭）
  bool topology_allow_edge_under_vehicle_{false};  // 参数 topology_allow_edge_under_vehicle
  std::string last_overlay_error_;
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
  // ROS 订阅句柄：只放 rclcpp 对象，生命周期随 on_cleanup/SetupVda5050Links 收口
  struct Vda5050LinkSubs {
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr online_sub;
  };
  // 两个表的 key 集合必须一致（都由 vda5050_vehicle_ids_ 驱动），
  // 拆开是为了让"链路纯数据"能直接喂给 core 的纯函数（守卫/选车）。
  std::unordered_map<std::string, Vda5050LinkSubs> vda5050_link_subs_;
  std::unordered_map<std::string, amr_dispatcher_core::fleet::Vda5050LinkState>
      vda5050_link_states_;
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
  // 相位 11：在线拓扑热生效（目录感知封路 / 全量叠加层 / 叠加层状态读取）
  rclcpp::Service<amr_dispatcher_interfaces::srv::BlockEdge>::SharedPtr block_edge_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::TopologyHotUpdate>::SharedPtr topology_hot_update_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::GetTopologyOverlay>::SharedPtr get_topology_overlay_srv_;
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

  // ---- 阶段 2 双发：proto 副本发布端 ----
  //
  // 用 ProtoLifecycleDualPublisher —— dispatcher_node 是 lifecycle 节点，proto 侧
  // 必须与 rosidl 侧同生共死：未激活时 rosidl 静默、proto 也必须静默。
  // 若这里用普通 Publisher，节点在 inactive 状态下会出现「rosidl 不发、proto 照发」
  // 的假数据流，一致性检查器与订阅方都会被误导。
  //
  // 三者的 QoS 深度必须与上面 rosidl 侧逐一对齐（10 / 20 / 10）：
  // 深度不一致时突发丢帧会被误判成「消息不一致」。
  std::unique_ptr<ProtoLifecycleDualPublisher<
      amr_dispatcher_interfaces::msg::DispatcherState,
      amr_dispatcher::proto::DispatcherState>>
      state_proto_pub_;
  std::unique_ptr<ProtoLifecycleDualPublisher<
      amr_dispatcher_interfaces::msg::MissionEvent,
      amr_dispatcher::proto::MissionEvent>>
      event_proto_pub_;
  std::unique_ptr<ProtoLifecycleDualPublisher<
      amr_dispatcher_interfaces::msg::TopologyState,
      amr_dispatcher::proto::TopologyState>>
      topology_proto_pub_;

  // 阶段 2 双发总开关（参数 proto_dual_publish，默认开）
  bool proto_dual_publish_{true};
  // 阶段 3：双订阅总开关。与 proto_dual_publish_ 独立——发布端与订阅端
  // 可以分批上线（先发布侧开着观察，再逐节点打开订阅侧）。
  bool proto_dual_subscribe_{false};

  // MissionEvent 双发的统一出口。
  //
  // 全文件有 18 处事件发布点，每处都是「构造 msg -> event_pub_->publish(msg)」。
  // 把双发放进这个函数而不是在每个调用点加第三行，是因为漏加一处就是一条
  // 静默不同步的事件流，而这种偏差不会让任何测试变红。
  // 调用方仍保留 is_activated() 守卫（见各调用点），本函数只负责两路投递。
  void PublishMissionEvent(const amr_dispatcher_interfaces::msg::MissionEvent& msg);

  // 阶段 3：/tracking/error 双订阅。注意本文件是全项目**唯一**对订阅句柄
  // 调用 reset() 的地方（生命周期收口，见 OnCleanup），适配器为此
  // 专门提供了 reset()，语义与 rclcpp 原生句柄一致。
  amr_dispatcher_proto_bridge::ProtoDualSubscriber<
      amr_dispatcher_interfaces::msg::TrackingError,
      amr_dispatcher::proto::TrackingError>
      tracking_error_sub_;
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
  // vehicle_id -> /tracking/error/{vid} 订阅（阶段 3：可按开关走 proto 话题）
  std::unordered_map<std::string, amr_dispatcher_proto_bridge::ProtoDualSubscriber<
      amr_dispatcher_interfaces::msg::TrackingError,
      amr_dispatcher::proto::TrackingError>>
      vehicle_tracking_error_subs_;
  // vehicle_id -> 最新里程计位姿（Pose2D + has_odom 标志）。
  // 与 legacy 的 current_pose_/has_odom_ 并存：全局 /odom 仍只喂 legacy 字段。
  struct VehiclePose {
    amr_dispatcher_core::path_tracking::Pose2D pose{};
    bool has_odom = false;
  };
  std::unordered_map<std::string, VehiclePose> vehicle_poses_;

  // ---- Phase 7: 在途任务等待表与交通协调管理 ----
  // mission_id -> 在途等待前方资源锁集合 (例如在道口等待 route_edge:B__C)
  std::unordered_map<std::string, std::vector<std::string>> in_flight_pending_wait_;
  // vehicle_id -> 最近一次确认到达的安全停靠节点 ID
  std::unordered_map<std::string, std::string> vehicle_safe_nodes_;

  // ---- Phase 8: 工业物理设施适配器集成与预约管理 ----
  std::unique_ptr<amr_dispatcher_core::facility::FacilityCatalogReloader> facility_reloader_;
  amr_dispatcher_core::facility::FacilityReservationMap facility_reservations_;
  amr_dispatcher_core::facility::MissionResourceMap mission_facility_resources_;
  std::unordered_map<std::string, std::shared_ptr<amr_dispatcher_core::facility::IFacilityAdapter>> facility_adapters_;

  rclcpp::TimerBase::SharedPtr schedule_timer_;
  rclcpp::TimerBase::SharedPtr deadlock_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr reload_timer_;  // 1Hz 驱动站点目录 mtime 热重载
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace amr_dispatcher_ros
