#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "amr_dispatcher_core/dispatcher/deadlock_detector.hpp"
#include "amr_dispatcher_core/dispatcher/mission_event_log.hpp"
#include "amr_dispatcher_core/dispatcher/mission_queue.hpp"
#include "amr_dispatcher_core/dispatcher/recovery_policy.hpp"
#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"

#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/srv/cancel_order.hpp"
#include "amr_dispatcher_interfaces/srv/pause_mission.hpp"
#include "amr_dispatcher_interfaces/srv/release_resource.hpp"
#include "amr_dispatcher_interfaces/srv/reserve_resource.hpp"
#include "amr_dispatcher_interfaces/srv/resume_mission.hpp"
#include "amr_dispatcher_interfaces/srv/submit_order.hpp"

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

  void ScheduleTick();
  void DeadlockTick();
  void PublishState();

  std::unique_ptr<amr_dispatcher_core::dispatcher::MissionQueue> queue_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::TrafficReservationTable> reservation_table_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::MissionEventLog> event_log_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::DeadlockDetector> deadlock_detector_;
  std::unique_ptr<amr_dispatcher_core::dispatcher::RecoveryPolicy> recovery_policy_;

  std::unordered_map<std::string, amr_dispatcher_core::dispatcher::Mission> active_missions_;
  std::unordered_map<std::string, std::size_t> mission_attempts_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> mission_last_progress_;
  std::uint64_t next_sequence_ = 1;
  bool deadlock_flag_ = false;

  rclcpp::Service<amr_dispatcher_interfaces::srv::SubmitOrder>::SharedPtr submit_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::CancelOrder>::SharedPtr cancel_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::PauseMission>::SharedPtr pause_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ResumeMission>::SharedPtr resume_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReserveResource>::SharedPtr reserve_srv_;
  rclcpp::Service<amr_dispatcher_interfaces::srv::ReleaseResource>::SharedPtr release_srv_;

  rclcpp_lifecycle::LifecyclePublisher<amr_dispatcher_interfaces::msg::DispatcherState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<amr_dispatcher_interfaces::msg::MissionEvent>::SharedPtr event_pub_;

  rclcpp::TimerBase::SharedPtr schedule_timer_;
  rclcpp::TimerBase::SharedPtr deadlock_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace amr_dispatcher_ros
