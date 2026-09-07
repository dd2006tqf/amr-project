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

#include "amr_dispatcher_interfaces/action/execute_mission.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"
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

  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ExecuteMission::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleExecute> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandleExecute> goal_handle);
  void ExecuteActionTask(const std::shared_ptr<GoalHandleExecute> goal_handle);

  void ScheduleTick();
  void DeadlockTick();
  void PublishState();

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

  rclcpp::Service<amr_dispatcher_interfaces::srv::SubmitOrder>::SharedPtr submit_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::CancelOrder>::SharedPtr cancel_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::PauseMission>::SharedPtr pause_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ResumeMission>::SharedPtr resume_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReserveResource>::SharedPtr reserve_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReleaseResource>::SharedPtr release_srv_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;

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

  rclcpp::TimerBase::SharedPtr schedule_timer_;
  rclcpp::TimerBase::SharedPtr deadlock_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace amr_dispatcher_ros
