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
