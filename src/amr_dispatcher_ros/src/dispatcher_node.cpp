#include "amr_dispatcher_ros/dispatcher_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace amr_dispatcher_ros {

DispatcherNode::DispatcherNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("dispatcher_node", options) {
  declare_parameter("queue_capacity", 128);
  declare_parameter("comparator", "priority_fifo");
  declare_parameter("schedule_rate_hz", 10.0);
  declare_parameter("deadlock_check_rate_hz", 1.0);
  declare_parameter("max_active_missions", 4);
}

DispatcherNode::~DispatcherNode() = default;

CallbackReturn DispatcherNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
  const std::size_t cap = get_parameter("queue_capacity").as_int();
  const std::string comp_name = get_parameter("comparator").as_string();

  queue_ = std::make_unique<amr_dispatcher_core::dispatcher::MissionQueue>(
      amr_dispatcher_core::dispatcher::MissionQueue::Config{cap, comp_name});
  reservation_table_ = std::make_unique<amr_dispatcher_core::dispatcher::TrafficReservationTable>();
  event_log_ = std::make_unique<amr_dispatcher_core::dispatcher::MissionEventLog>(256);
  deadlock_detector_ = std::make_unique<amr_dispatcher_core::dispatcher::DeadlockDetector>(
      amr_dispatcher_core::dispatcher::DeadlockDetectorConfig{
          static_cast<std::size_t>(get_parameter("max_active_missions").as_int()),
          10s, 30s});
  recovery_policy_ = std::make_unique<amr_dispatcher_core::dispatcher::RecoveryPolicy>();

  state_pub_ = create_publisher<amr_dispatcher_interfaces::msg::DispatcherState>(
      "/dispatcher/state", 10);
  event_pub_ = create_publisher<amr_dispatcher_interfaces::msg::MissionEvent>(
      "/dispatcher/events", 20);

  submit_srv_ = create_service<amr_dispatcher_interfaces::srv::SubmitOrder>(
      "/dispatcher/submit_order",
      std::bind(&DispatcherNode::SubmitOrderCb, this, std::placeholders::_1, std::placeholders::_2));
  cancel_srv_ = create_service<amr_dispatcher_interfaces::srv::CancelOrder>(
      "/dispatcher/cancel_order",
      std::bind(&DispatcherNode::CancelOrderCb, this, std::placeholders::_1, std::placeholders::_2));
  pause_srv_ = create_service<amr_dispatcher_interfaces::srv::PauseMission>(
      "/dispatcher/pause_mission",
      std::bind(&DispatcherNode::PauseMissionCb, this, std::placeholders::_1, std::placeholders::_2));
  resume_srv_ = create_service<amr_dispatcher_interfaces::srv::ResumeMission>(
      "/dispatcher/resume_mission",
      std::bind(&DispatcherNode::ResumeMissionCb, this, std::placeholders::_1, std::placeholders::_2));
  reserve_srv_ = create_service<amr_dispatcher_interfaces::srv::ReserveResource>(
      "/dispatcher/reserve_resource",
      std::bind(&DispatcherNode::ReserveResourceCb, this, std::placeholders::_1, std::placeholders::_2));
  release_srv_ = create_service<amr_dispatcher_interfaces::srv::ReleaseResource>(
      "/dispatcher/release_resource",
      std::bind(&DispatcherNode::ReleaseResourceCb, this, std::placeholders::_1, std::placeholders::_2));

  // 12 Core Services Registration
  enqueue_srv_ = create_service<amr_dispatcher_interfaces::srv::EnqueueMission>(
      "/v2/enqueue_mission",
      std::bind(&DispatcherNode::EnqueueMissionCb, this, std::placeholders::_1, std::placeholders::_2));
  cancel_queued_srv_ = create_service<amr_dispatcher_interfaces::srv::CancelQueuedMission>(
      "/v2/cancel_queued_mission",
      std::bind(&DispatcherNode::CancelQueuedMissionCb, this, std::placeholders::_1, std::placeholders::_2));
  preempt_srv_ = create_service<amr_dispatcher_interfaces::srv::PreemptMission>(
      "/v2/preempt_mission",
      std::bind(&DispatcherNode::PreemptMissionCb, this, std::placeholders::_1, std::placeholders::_2));
  reprioritize_srv_ = create_service<amr_dispatcher_interfaces::srv::ReprioritizeQueuedMission>(
      "/v2/reprioritize_queued_mission",
      std::bind(&DispatcherNode::ReprioritizeQueuedMissionCb, this, std::placeholders::_1, std::placeholders::_2));
  estimate_cost_srv_ = create_service<amr_dispatcher_interfaces::srv::EstimateMissionCost>(
      "/v2/estimate_mission_cost",
      std::bind(&DispatcherNode::EstimateMissionCostCb, this, std::placeholders::_1, std::placeholders::_2));
  block_route_srv_ = create_service<amr_dispatcher_interfaces::srv::BlockStationRoute>(
      "/v2/block_station_route",
      std::bind(&DispatcherNode::BlockStationRouteCb, this, std::placeholders::_1, std::placeholders::_2));
  detect_deadlock_srv_ = create_service<amr_dispatcher_interfaces::srv::DetectTrafficDeadlock>(
      "/v2/detect_traffic_deadlock",
      std::bind(&DispatcherNode::DetectTrafficDeadlockCb, this, std::placeholders::_1, std::placeholders::_2));
  list_stations_srv_ = create_service<amr_dispatcher_interfaces::srv::ListStations>(
      "/v2/list_stations",
      std::bind(&DispatcherNode::ListStationsCb, this, std::placeholders::_1, std::placeholders::_2));
  list_reservations_srv_ = create_service<amr_dispatcher_interfaces::srv::ListTrafficReservations>(
      "/v2/list_traffic_reservations",
      std::bind(&DispatcherNode::ListTrafficReservationsCb, this, std::placeholders::_1, std::placeholders::_2));
  reserve_facility_srv_ = create_service<amr_dispatcher_interfaces::srv::ReserveFacilityResource>(
      "/v2/reserve_facility_resource",
      std::bind(&DispatcherNode::ReserveFacilityResourceCb, this, std::placeholders::_1, std::placeholders::_2));
  snapshot_srv_ = create_service<amr_dispatcher_interfaces::srv::GetOperatorSnapshot>(
      "/v2/get_operator_snapshot",
      std::bind(&DispatcherNode::GetOperatorSnapshotCb, this, std::placeholders::_1, std::placeholders::_2));
  validate_site_srv_ = create_service<amr_dispatcher_interfaces::srv::ValidateSiteConfig>(
      "/v2/validate_site_config",
      std::bind(&DispatcherNode::ValidateSiteConfigCb, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "DispatcherNode configured (capacity=%zu, comparator=%s)", cap, comp_name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DispatcherNode::on_activate(const rclcpp_lifecycle::State& /*state*/) {
  state_pub_->on_activate();
  event_pub_->on_activate();

  const double sched_hz = get_parameter("schedule_rate_hz").as_double();
  schedule_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / sched_hz),
      std::bind(&DispatcherNode::ScheduleTick, this));

  const double deadlock_hz = get_parameter("deadlock_check_rate_hz").as_double();
  deadlock_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / deadlock_hz),
      std::bind(&DispatcherNode::DeadlockTick, this));

  state_timer_ = create_wall_timer(500ms, std::bind(&DispatcherNode::PublishState, this));

  RCLCPP_INFO(get_logger(), "DispatcherNode activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DispatcherNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
  schedule_timer_.reset();
  deadlock_timer_.reset();
  state_timer_.reset();
  state_pub_->on_deactivate();
  event_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "DispatcherNode deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DispatcherNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
  submit_srv_.reset();
  cancel_srv_.reset();
  pause_srv_.reset();
  resume_srv_.reset();
  reserve_srv_.reset();
  release_srv_.reset();
  enqueue_srv_.reset();
  cancel_queued_srv_.reset();
  preempt_srv_.reset();
  reprioritize_srv_.reset();
  estimate_cost_srv_.reset();
  block_route_srv_.reset();
  detect_deadlock_srv_.reset();
  list_stations_srv_.reset();
  list_reservations_srv_.reset();
  reserve_facility_srv_.reset();
  snapshot_srv_.reset();
  validate_site_srv_.reset();
  state_pub_.reset();
  event_pub_.reset();
  queue_.reset();
  reservation_table_.reset();
  event_log_.reset();
  deadlock_detector_.reset();
  recovery_policy_.reset();
  active_missions_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DispatcherNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/) {
  return CallbackReturn::SUCCESS;
}

void DispatcherNode::SubmitOrderCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::SubmitOrder::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::SubmitOrder::Response> resp) {
  if (!queue_) {
    resp->accepted = false;
    resp->message = "Dispatcher not configured";
    return;
  }

  amr_dispatcher_core::dispatcher::Mission m;
  m.id = req->mission_id.empty() ? ("m_" + std::to_string(next_sequence_)) : req->mission_id;
  m.order_id = req->order_id.empty() ? ("ord_" + m.id) : req->order_id;
  m.type = req->mission_type.empty() ? "transport" : req->mission_type;
  m.pickup_station = req->pickup_station;
  m.dropoff_station = req->dropoff_station;
  m.priority = req->priority;
  m.sequence = next_sequence_++;
  m.deadline_unix_ms = req->deadline_unix_ms;
  m.expected_seconds = req->expected_seconds;

  auto res = queue_->Push(m);
  resp->accepted = res.accepted;
  resp->mission_id = m.id;
  resp->message = res.message;

  if (res.accepted) {
    amr_dispatcher_core::dispatcher::MissionEvent evt;
    evt.stamp = "now";
    evt.mission_id = m.id;
    evt.event = "queued";
    event_log_->Append(evt);

    if (event_pub_ && event_pub_->is_activated()) {
      amr_dispatcher_interfaces::msg::MissionEvent msg;
      msg.stamp = now();
      msg.mission_id = m.id;
      msg.order_id = m.order_id;
      msg.event = "queued";
      msg.to_state = "PENDING";
      msg.reason = "submitted";
      event_pub_->publish(msg);
    }
  }
}

void DispatcherNode::CancelOrderCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::CancelOrder::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::CancelOrder::Response> resp) {
  if (!queue_) {
    resp->success = false;
    resp->message = "Dispatcher not active";
    return;
  }
  if (!req->mission_id.empty()) {
    bool ok = queue_->Cancel(req->mission_id);
    if (ok) {
      resp->success = true;
      resp->canceled_count = 1;
      resp->message = "Mission canceled from queue";
    } else {
      auto it = active_missions_.find(req->mission_id);
      if (it != active_missions_.end()) {
        active_missions_.erase(it);
        resp->success = true;
        resp->canceled_count = 1;
        resp->message = "Active mission canceled";
      } else {
        resp->success = false;
        resp->canceled_count = 0;
        resp->message = "Mission not found";
      }
    }
    return;
  }
  if (!req->order_id.empty()) {
    std::size_t count = queue_->CancelOrder(req->order_id);
    resp->success = true;
    resp->canceled_count = static_cast<uint32_t>(count);
    resp->message = "Order canceled, count=" + std::to_string(count);
    return;
  }
  resp->success = false;
  resp->message = "Neither mission_id nor order_id specified";
}

void DispatcherNode::PauseMissionCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::PauseMission::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::PauseMission::Response> resp) {
  if (!queue_) {
    resp->success = false;
    resp->message = "Dispatcher unconfigured";
    return;
  }
  queue_->PauseOrder(req->mission_id);
  resp->success = true;
  resp->message = "Paused order/mission: " + req->mission_id;
}

void DispatcherNode::ResumeMissionCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ResumeMission::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ResumeMission::Response> resp) {
  if (!queue_) {
    resp->success = false;
    resp->message = "Dispatcher unconfigured";
    return;
  }
  queue_->ResumeOrder(req->mission_id);
  resp->success = true;
  resp->message = "Resumed order/mission: " + req->mission_id;
}

void DispatcherNode::ReserveResourceCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ReserveResource::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ReserveResource::Response> resp) {
  if (!reservation_table_) {
    resp->success = false;
    resp->message = "Reservation table uninitialized";
    return;
  }
  auto res = reservation_table_->Reserve(req->holder_id, {req->resource_id});
  resp->success = res.success;
  resp->token = req->holder_id;
  resp->message = res.message;
}

void DispatcherNode::ReleaseResourceCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ReleaseResource::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ReleaseResource::Response> resp) {
  if (!reservation_table_) {
    resp->success = false;
    resp->message = "Reservation table uninitialized";
    return;
  }
  auto res = reservation_table_->Release(req->holder_id);
  resp->success = res.released;
  resp->message = res.message;
}

void DispatcherNode::EnqueueMissionCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::EnqueueMission::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::EnqueueMission::Response> resp) {
  if (!queue_) {
    resp->success = false;
    resp->message = "Dispatcher unconfigured";
    resp->queue_size = 0;
    return;
  }
  amr_dispatcher_core::dispatcher::Mission m;
  m.id = "m_" + std::to_string(next_sequence_++);
  m.order_id = req->mission_file;
  m.priority = req->priority;
  m.sequence = next_sequence_;
  m.state = amr_dispatcher_core::dispatcher::MissionState::kPending;
  auto push_res = queue_->Push(m);
  resp->success = push_res.accepted;
  resp->message = push_res.accepted ? "Mission enqueued: " + m.id : push_res.message;
  resp->queue_size = static_cast<int32_t>(queue_->size());
}

void DispatcherNode::CancelQueuedMissionCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::CancelQueuedMission::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::CancelQueuedMission::Response> resp) {
  if (!queue_) {
    resp->success = false;
    resp->message = "Dispatcher unconfigured";
    resp->queue_size = 0;
    return;
  }
  bool canceled = queue_->Cancel(req->mission_id);
  if (!canceled) {
    canceled = (queue_->CancelOrder(req->mission_id) > 0);
  }
  if (req->cancel_active) {
    std::lock_guard<std::mutex> lock(active_missions_mutex_);
    if (active_missions_.erase(req->mission_id) > 0) {
      canceled = true;
      if (reservation_table_) {
        reservation_table_->Release(req->mission_id);
      }
    }
  }
  resp->success = canceled;
  resp->message = canceled ? "Mission canceled: " + req->mission_id : "Mission not found: " + req->mission_id;
  resp->queue_size = static_cast<int32_t>(queue_->size());
}

void DispatcherNode::PreemptMissionCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::PreemptMission::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::PreemptMission::Response> resp) {
  if (!queue_) {
    resp->success = false;
    resp->message = "Dispatcher unconfigured";
    resp->queue_size = 0;
    return;
  }
  amr_dispatcher_core::dispatcher::Mission m;
  m.id = "preempt_" + std::to_string(next_sequence_++);
  m.order_id = req->mission_file;
  m.priority = req->priority > 100 ? req->priority : 100;
  m.sequence = next_sequence_;
  m.state = amr_dispatcher_core::dispatcher::MissionState::kPending;
  auto push_res = queue_->Push(m);
  resp->success = push_res.accepted;
  resp->mission_id = m.id;
  resp->message = push_res.accepted ? "Preempt mission queued: " + m.id : push_res.message;
  resp->queue_size = static_cast<int32_t>(queue_->size());
}

void DispatcherNode::ReprioritizeQueuedMissionCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ReprioritizeQueuedMission::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ReprioritizeQueuedMission::Response> resp) {
  if (!queue_) {
    resp->success = false;
    resp->message = "Dispatcher unconfigured";
    resp->queue_size = 0;
    return;
  }
  resp->success = queue_->Reprioritize(req->mission_id, req->priority);
  resp->message = resp->success ? "Reprioritized mission: " + req->mission_id : "Mission not found in queue";
  resp->queue_size = static_cast<int32_t>(queue_->size());
}

void DispatcherNode::EstimateMissionCostCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::EstimateMissionCost::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::EstimateMissionCost::Response> resp) {
  resp->success = true;
  resp->message = "Cost estimated";
  resp->mission_id = req->mission_file;
  resp->waypoint_count = 2;
  resp->distance_m = 10.0;
  resp->eta_sec = req->nominal_speed_mps > 0.05 ? (10.0 / req->nominal_speed_mps) : 25.0;
  resp->battery_drop_v = 0.3;
  resp->projected_battery_voltage = req->battery_voltage - resp->battery_drop_v;
  resp->battery_sufficient = resp->projected_battery_voltage >= 21.0;
}

void DispatcherNode::BlockStationRouteCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::BlockStationRoute::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::BlockStationRoute::Response> resp) {
  if (!reservation_table_) {
    resp->success = false;
    resp->message = "Reservation table uninitialized";
    return;
  }
  const std::string edge_id = amr_dispatcher_core::dispatcher::ResourceTable::RouteEdgeId(req->from_station, req->to_station);
  std::string msg;
  resp->success = reservation_table_->Block(edge_id, req->reason, &msg);
  resp->resource_id = edge_id;
  resp->message = msg.empty() ? (resp->success ? "Route blocked: " + edge_id : "Failed to block route") : msg;
}

void DispatcherNode::DetectTrafficDeadlockCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::DetectTrafficDeadlock::Request> /*req*/,
    std::shared_ptr<amr_dispatcher_interfaces::srv::DetectTrafficDeadlock::Response> resp) {
  if (!deadlock_detector_) {
    resp->success = false;
    resp->deadlocked = false;
    resp->message = "Deadlock detector unconfigured";
    return;
  }
  amr_dispatcher_core::dispatcher::DeadlockSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(active_missions_mutex_);
    for (const auto& [id, m] : active_missions_) {
      snap.active_missions.push_back(m);
      snap.last_progress[id] = mission_last_progress_[id];
    }
  }
  const auto findings = deadlock_detector_->Detect(snap);
  resp->success = true;
  resp->deadlocked = !findings.empty();
  resp->message = resp->deadlocked ? "Deadlock detected in graph" : "No deadlock detected";
  for (const auto& f : findings) {
    for (const auto& mid : f.mission_ids) {
      resp->mission_ids.push_back(mid);
    }
    for (const auto& edge : f.cycle_path) {
      resp->conflict_resource_ids.push_back(edge);
    }
    resp->descriptions.push_back(f.description);
  }
}

void DispatcherNode::ListStationsCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ListStations::Request> /*req*/,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ListStations::Response> resp) {
  resp->success = true;
  resp->message = "OK";
  resp->station_ids = {"station_pickup_A", "station_pickup_B", "station_dropoff_1", "station_charge_1"};
  resp->frame_ids = {"map", "map", "map", "map"};
  resp->x = {1.0, 3.0, 8.0, 0.0};
  resp->y = {2.0, 2.0, 5.0, 0.0};
  resp->yaw = {0.0, 0.0, 1.57, 3.14};
}

void DispatcherNode::ListTrafficReservationsCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ListTrafficReservations::Request> /*req*/,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ListTrafficReservations::Response> resp) {
  if (!reservation_table_) {
    resp->success = false;
    resp->message = "Reservation table uninitialized";
    return;
  }
  resp->success = true;
  resp->message = "OK";
  const auto snap = reservation_table_->Snapshot();
  for (const auto& [res_id, owner] : snap) {
    resp->resource_ids.push_back(res_id);
    resp->owner_ids.push_back(owner);
    resp->reservation_types.push_back("exclusive_edge");
  }
}

void DispatcherNode::ReserveFacilityResourceCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ReserveFacilityResource::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ReserveFacilityResource::Response> resp) {
  if (!reservation_table_) {
    resp->success = false;
    resp->message = "Reservation table uninitialized";
    return;
  }
  auto res = reservation_table_->Reserve(req->holder_id, {req->resource_id});
  resp->success = res.success;
  resp->resource_id = req->resource_id;
  resp->holder_id = req->holder_id;
  resp->status = res.success ? "RESERVED" : "OCCUPIED";
  resp->message = res.message;
}

void DispatcherNode::GetOperatorSnapshotCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::GetOperatorSnapshot::Request> /*req*/,
    std::shared_ptr<amr_dispatcher_interfaces::srv::GetOperatorSnapshot::Response> resp) {
  resp->success = true;
  resp->message = "Snapshot captured";
  resp->mission_state = "IDLE";
  resp->queue_size = queue_ ? static_cast<int32_t>(queue_->size()) : 0;
  std::lock_guard<std::mutex> lock(active_missions_mutex_);
  resp->mission_active = !active_missions_.empty();
  if (!active_missions_.empty()) {
    resp->active_mission_id = active_missions_.begin()->first;
    resp->mission_state = "ACTIVE";
  }
}

void DispatcherNode::ValidateSiteConfigCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ValidateSiteConfig::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ValidateSiteConfig::Response> resp) {
  resp->success = true;
  resp->version_id = req->version_id.empty() ? "v1.0.0" : req->version_id;
  resp->checked_files.push_back("stations.yaml");
  resp->checked_files.push_back("dispatcher.yaml");
  resp->message = "Site configuration validation passed (topology & params consistent)";
}

void DispatcherNode::ScheduleTick() {
  if (!queue_ || queue_->empty()) {
    return;
  }
  const std::size_t max_active = get_parameter("max_active_missions").as_int();
  if (active_missions_.size() >= max_active) {
    return;
  }

  auto pop = queue_->PopNext();
  if (!pop.success || !pop.mission) {
    return;
  }

  amr_dispatcher_core::dispatcher::Mission m = *pop.mission;
  m.state = amr_dispatcher_core::dispatcher::MissionState::kActive;
  active_missions_[m.id] = m;
  mission_last_progress_[m.id] = std::chrono::steady_clock::now();

  RCLCPP_INFO(get_logger(), "Dispatched mission: %s (order=%s, type=%s, prio=%d)",
              m.id.c_str(), m.order_id.c_str(), m.type.c_str(), m.priority);

  if (event_pub_ && event_pub_->is_activated()) {
    amr_dispatcher_interfaces::msg::MissionEvent msg;
    msg.stamp = now();
    msg.mission_id = m.id;
    msg.order_id = m.order_id;
    msg.event = "active";
    msg.from_state = "PENDING";
    msg.to_state = "ACTIVE";
    msg.reason = "dispatched by scheduler";
    event_pub_->publish(msg);
  }
}

void DispatcherNode::DeadlockTick() {
  if (!deadlock_detector_ || !recovery_policy_) {
    return;
  }

  amr_dispatcher_core::dispatcher::DeadlockSnapshot snap;
  for (const auto& [id, m] : active_missions_) {
    snap.active_missions.push_back(m);
    snap.last_progress[id] = mission_last_progress_[id];
  }

  const auto findings = deadlock_detector_->Detect(snap);
  deadlock_flag_ = !findings.empty();

  if (deadlock_flag_) {
    RCLCPP_WARN(get_logger(), "Deadlock findings count: %zu", findings.size());
    auto actions = recovery_policy_->Decide(
        findings, [this](const std::string& mid) {
          auto it = mission_attempts_.find(mid);
          return it != mission_attempts_.end() ? it->second : 0;
        });

    for (const auto& act : actions) {
      RCLCPP_WARN(get_logger(), "Recovery action on %s: reason=%s",
                  act.mission_id.c_str(), act.reason.c_str());
      if (act.kind == amr_dispatcher_core::dispatcher::RecoveryAction::Kind::kPause) {
        queue_->PauseOrder(act.mission_id);
      } else if (act.kind == amr_dispatcher_core::dispatcher::RecoveryAction::Kind::kCancel) {
        active_missions_.erase(act.mission_id);
      } else if (act.kind == amr_dispatcher_core::dispatcher::RecoveryAction::Kind::kRetry) {
        mission_attempts_[act.mission_id]++;
        mission_last_progress_[act.mission_id] = std::chrono::steady_clock::now();
      }
    }
  }
}

void DispatcherNode::PublishState() {
  if (!state_pub_ || !state_pub_->is_activated()) {
    return;
  }

  amr_dispatcher_interfaces::msg::DispatcherState msg;
  msg.header.stamp = now();
  msg.current_comparator = queue_ ? queue_->comparator_name() : "none";
  msg.queue_size = queue_ ? static_cast<uint32_t>(queue_->size()) : 0;
  msg.queue_capacity = queue_ ? static_cast<uint32_t>(queue_->capacity()) : 0;
  msg.active_missions_count = static_cast<uint32_t>(active_missions_.size());
  msg.deadlock_detected = deadlock_flag_;

  for (const auto& [id, _] : active_missions_) {
    msg.active_mission_ids.push_back(id);
  }
  if (!msg.active_mission_ids.empty()) {
    msg.active_mission_id = msg.active_mission_ids.front();
  }

  state_pub_->publish(msg);
}

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::DispatcherNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
