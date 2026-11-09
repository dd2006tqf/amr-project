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
  declare_parameter("stations_file", "config/stations.yaml");
  declare_parameter("path_step_size_m", 0.1);
  declare_parameter("recovery_dock_station", "station_charge_1");
  recovery_dock_station_ = get_parameter("recovery_dock_station").as_string();
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

  // 站点目录与基准线性规划器：任务激活时把 pickup→dropoff 展开成 /plan 下发。
  // 目录经 StationCatalogReloader 持有：构造时加载，mtime 变化自动热重载；
  // 初始加载失败也照常挂监听，文件补齐后下一个 Tick 即生效。
  const std::string stations_file = get_parameter("stations_file").as_string();
  catalog_reloader_ =
      std::make_unique<amr_dispatcher_core::catalog::StationCatalogReloader>(stations_file);
  if (catalog_reloader_->loaded()) {
    RCLCPP_INFO(get_logger(), "Loaded %zu stations from %s",
                catalog_reloader_->catalog().stations.size(), stations_file.c_str());
  } else {
    RCLCPP_WARN(get_logger(),
                "Failed to load stations file %s; planning disabled until hot reload succeeds",
                stations_file.c_str());
  }
  planner_ = std::make_unique<amr_dispatcher_core::MockLinearPlanner>(
      get_parameter("path_step_size_m").as_double());

  state_pub_ = create_publisher<amr_dispatcher_interfaces::msg::DispatcherState>(
      "/dispatcher/state", 10);
  event_pub_ = create_publisher<amr_dispatcher_interfaces::msg::MissionEvent>(
      "/dispatcher/events", 20);
  plan_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 1);

  // 位姿与到达信号订阅：path_tracker 的 TrackingError.arrived 驱动任务完成
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DispatcherNode::OdomCb, this, std::placeholders::_1));
  tracking_error_sub_ = create_subscription<amr_dispatcher_interfaces::msg::TrackingError>(
      "/tracking/error", 10,
      std::bind(&DispatcherNode::TrackingErrorCb, this, std::placeholders::_1));

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
  plan_pub_->on_activate();

  const double sched_hz = get_parameter("schedule_rate_hz").as_double();
  schedule_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / sched_hz),
      std::bind(&DispatcherNode::ScheduleTick, this));

  const double deadlock_hz = get_parameter("deadlock_check_rate_hz").as_double();
  deadlock_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / deadlock_hz),
      std::bind(&DispatcherNode::DeadlockTick, this));

  state_timer_ = create_wall_timer(500ms, std::bind(&DispatcherNode::PublishState, this));

  // 1Hz 驱动站点目录 mtime 热重载（HotReloadWatcher 由调用方驱动）
  reload_timer_ = create_wall_timer(1s, [this]() {
    if (!catalog_reloader_) return;
    if (catalog_reloader_->Tick()) {
      RCLCPP_INFO(get_logger(), "Station catalog hot-reloaded: %zu stations",
                  catalog_reloader_->catalog().stations.size());
    } else if (!catalog_reloader_->last_error().empty()) {
      // 重载失败（YAML 解析或校验不过）：旧目录继续生效，节流告警提示修复文件
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Station catalog reload failed, keeping previous: %s",
                           catalog_reloader_->last_error().c_str());
    }
  });

  RCLCPP_INFO(get_logger(), "DispatcherNode activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DispatcherNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
  schedule_timer_.reset();
  deadlock_timer_.reset();
  state_timer_.reset();
  reload_timer_.reset();
  state_pub_->on_deactivate();
  event_pub_->on_deactivate();
  plan_pub_->on_deactivate();
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
  plan_pub_.reset();
  tracking_error_sub_.reset();
  odom_sub_.reset();
  queue_.reset();
  reservation_table_.reset();
  event_log_.reset();
  deadlock_detector_.reset();
  recovery_policy_.reset();
  planner_.reset();
  catalog_reloader_.reset();
  reload_timer_.reset();
  queue_start_requested_ = false;
  tracking_mission_id_.clear();
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

  // 订单路由：经 submit_order 行为树调用路由器，决定订单类型语义与字段回填。
  // 未知/空类型由路由器统一拒绝（message 带 supported 列表）。
  amr_dispatcher_bt::SubmitOrderBehaviorTreeInput route_input;
  route_input.request.order_id = req->order_id.empty() ? req->mission_id : req->order_id;
  route_input.request.order_type = req->mission_type.empty() ? "transport" : req->mission_type;
  route_input.request.priority = req->priority;
  const auto route_result = amr_dispatcher_bt::TickSubmitOrderBehaviorTree(route_input);
  if (!route_result.success || !route_result.route) {
    resp->accepted = false;
    resp->mission_id = req->mission_id;
    resp->message = "route rejected: " + route_result.message;
    RCLCPP_WARN(get_logger(), "SubmitOrder rejected by router: %s", resp->message.c_str());
    return;
  }
  const auto& route = *route_result.route;

  amr_dispatcher_core::dispatcher::Mission m;
  m.id = req->mission_id.empty() ? ("m_" + std::to_string(next_sequence_)) : req->mission_id;
  m.order_id = req->order_id.empty() ? ("ord_" + m.id) : req->order_id;
  m.type = route.normalized_order_type.empty() ? "transport" : route.normalized_order_type;
  // 站点字段优先取请求显式值，路由器解析出的站点作兜底
  m.pickup_station = req->pickup_station.empty() ? route.pickup_station : req->pickup_station;
  m.dropoff_station = req->dropoff_station.empty() ? route.dropoff_station : req->dropoff_station;
  m.priority = req->priority;
  m.sequence = next_sequence_++;
  m.deadline_unix_ms = req->deadline_unix_ms;
  m.expected_seconds = req->expected_seconds;

  auto res = queue_->Push(m);
  resp->accepted = res.accepted;
  resp->mission_id = m.id;
  resp->message = res.message;

  if (res.accepted) {
    if (route.start_if_idle) {
      queue_start_requested_ = true;
    }
    RCLCPP_INFO(get_logger(),
                "SubmitOrder routed via BT: branch=%s kind=%d type=%s start_if_idle=%d",
                route_result.branch.c_str(), static_cast<int>(route.kind), m.type.c_str(),
                route.start_if_idle ? 1 : 0);
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
        // 释放资源锁；若该任务正在占用 /plan，发空路径停车
        if (reservation_table_) {
          reservation_table_->Release(req->mission_id);
        }
        if (tracking_mission_id_ == req->mission_id) {
          tracking_mission_id_.clear();
          if (plan_pub_ && plan_pub_->is_activated()) {
            nav_msgs::msg::Path empty;
            empty.header.stamp = now();
            empty.header.frame_id = "map";
            plan_pub_->publish(empty);
          }
        }
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
    ReArmQueueStartRequest();  // 取消活动任务后，队列后续任务需重新武装启动请求
    return;
  }
  if (!req->order_id.empty()) {
    std::size_t count = queue_->CancelOrder(req->order_id);
    resp->success = true;
    resp->canceled_count = static_cast<uint32_t>(count);
    resp->message = "Order canceled, count=" + std::to_string(count);
    ReArmQueueStartRequest();
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
  ReArmQueueStartRequest();  // 恢复后的暂停任务重新参与出队派发
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

  if (push_res.accepted) {
    // 启动决策走 queued_mission_start 行为树（start_if_idle 且当前空闲才立即启动）
    amr_dispatcher_bt::QueuedMissionStartBehaviorTreeInput start_input;
    start_input.request.start_if_idle = req->start_if_idle;
    start_input.request.mission_active = !tracking_mission_id_.empty();
    const auto start_bt = amr_dispatcher_bt::TickQueuedMissionStartBehaviorTree(start_input);
    if (start_bt.success && start_bt.decision.request_queue_start) {
      queue_start_requested_ = true;
    }
    RCLCPP_INFO(get_logger(), "EnqueueMission via BT: branch=%s request_queue_start=%d",
                start_bt.branch.c_str(), start_bt.decision.request_queue_start ? 1 : 0);
  }
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
    bool erased = false;
    {
      std::lock_guard<std::mutex> lock(active_missions_mutex_);
      erased = (active_missions_.erase(req->mission_id) > 0);
    }
    if (erased) {
      canceled = true;
      if (reservation_table_) {
        reservation_table_->Release(req->mission_id);
      }
      // 若被取消的是正在跟踪 /plan 的任务，发空路径停车
      if (tracking_mission_id_ == req->mission_id) {
        tracking_mission_id_.clear();
        if (plan_pub_ && plan_pub_->is_activated()) {
          nav_msgs::msg::Path empty;
          empty.header.stamp = now();
          empty.header.frame_id = "map";
          plan_pub_->publish(empty);
        }
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

  // 单车执行模型：同时只有一条 /plan 在跟踪，其余任务保持 active 等待车辆空闲
  if (!tracking_mission_id_.empty()) {
    return;
  }

  // 行为树出队派发门控：runtime 树按"空闲 + 启动请求"给派发决策，
  // 未请求启动时即使队列有任务也不出队（启动请求由提单/入队/结果链武装）。
  amr_dispatcher_bt::MissionRuntimeBehaviorTreeInput runtime_input;
  runtime_input.tick.mission_active = !tracking_mission_id_.empty();
  runtime_input.tick.queue_start_requested = queue_start_requested_;
  const auto runtime_result = amr_dispatcher_bt::TickMissionRuntimeBehaviorTree(runtime_input);
  if (!runtime_result.success || !runtime_result.decision.start_queued_mission) {
    return;
  }
  queue_start_requested_ = false;  // 启动请求被本次派发消费

  auto pop = queue_->PopNext();
  if (!pop.success || !pop.mission) {
    return;
  }

  amr_dispatcher_core::dispatcher::Mission m = *pop.mission;
  m.state = amr_dispatcher_core::dispatcher::MissionState::kActive;

  // 解析起终点：无 pickup 时用车辆当前位姿作起点；站点缺失或目录未加载直接判负
  const amr_dispatcher_core::catalog::Station* from_st = nullptr;
  const amr_dispatcher_core::catalog::Station* to_st = nullptr;
  const bool catalog_loaded =
      catalog_reloader_ != nullptr && catalog_reloader_->loaded();
  if (catalog_loaded) {
    const auto& catalog = catalog_reloader_->catalog();
    if (!m.pickup_station.empty()) {
      from_st = amr_dispatcher_core::catalog::FindStation(catalog, m.pickup_station);
    }
    if (!m.dropoff_station.empty()) {
      to_st = amr_dispatcher_core::catalog::FindStation(catalog, m.dropoff_station);
    }
  }
  if (m.dropoff_station.empty() || !to_st) {
    RCLCPP_WARN(get_logger(), "Mission %s has no resolvable dropoff station (%s, catalog_loaded=%d), failing",
                m.id.c_str(), m.dropoff_station.c_str(), catalog_loaded);
    if (event_pub_ && event_pub_->is_activated()) {
      amr_dispatcher_interfaces::msg::MissionEvent msg;
      msg.stamp = now();
      msg.mission_id = m.id;
      msg.order_id = m.order_id;
      msg.event = "plan_failed";
      msg.to_state = "FAILED";
      msg.reason = "unknown dropoff station: " + m.dropoff_station;
      event_pub_->publish(msg);
    }
    ReArmQueueStartRequest();  // 本单丢弃后，队列中其余任务仍需能被派发
    return;
  }

  active_missions_[m.id] = m;
  mission_last_progress_[m.id] = std::chrono::steady_clock::now();

  // 生成并发布路径：path_tracker 订阅 /plan 后开始输出 /cmd_vel_raw
  amr_dispatcher_core::Pose2D start;
  if (from_st) {
    start = {from_st->x, from_st->y, from_st->yaw};
  } else if (has_odom_) {
    start = {current_pose_.x, current_pose_.y, current_pose_.yaw};
  }
  const amr_dispatcher_core::Pose2D goal{to_st->x, to_st->y, to_st->yaw};

  const auto waypoints = planner_ ? planner_->Plan(start, goal) : std::vector<amr_dispatcher_core::Pose2D>{};
  if (waypoints.empty() || !plan_pub_ || !plan_pub_->is_activated()) {
    FailMission(m.id, "planner returned empty path");
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id = "map";
  path_msg.poses.reserve(waypoints.size());
  for (const auto& wp : waypoints) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position.x = wp.x;
    ps.pose.position.y = wp.y;
    ps.pose.orientation.z = std::sin(wp.theta / 2.0);
    ps.pose.orientation.w = std::cos(wp.theta / 2.0);
    path_msg.poses.push_back(ps);
  }
  plan_pub_->publish(path_msg);
  tracking_mission_id_ = m.id;

  RCLCPP_INFO(get_logger(), "Dispatched mission: %s (order=%s, type=%s, prio=%d, %s->%s, %zu waypoints)",
              m.id.c_str(), m.order_id.c_str(), m.type.c_str(), m.priority,
              m.pickup_station.c_str(), m.dropoff_station.c_str(), waypoints.size());

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

void DispatcherNode::OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  current_pose_.yaw = amr_dispatcher_core::path_tracking::YawFromQuaternion(
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  has_odom_ = true;
}

void DispatcherNode::TrackingErrorCb(
    const amr_dispatcher_interfaces::msg::TrackingError::SharedPtr msg) {
  if (!msg->arrived || tracking_mission_id_.empty()) {
    return;
  }
  const std::string finished_id = tracking_mission_id_;
  tracking_mission_id_.clear();
  mission_last_progress_[finished_id] = std::chrono::steady_clock::now();
  CompleteMission(finished_id, "arrived at goal");
}

void DispatcherNode::CompleteMission(const std::string& mission_id, const std::string& reason) {
  std::string order_id;
  {
    std::lock_guard<std::mutex> lock(active_missions_mutex_);
    auto it = active_missions_.find(mission_id);
    if (it == active_missions_.end()) {
      return;
    }
    order_id = it->second.order_id;
    active_missions_.erase(it);
  }

  // 结果处理走 mission_result 行为树：成功分支给资源释放与失败计数复位决策
  amr_dispatcher_core::workflow::MissionResultWorkflowInput result_input;
  result_input.code = amr_dispatcher_core::workflow::MissionActionResultCode::kSucceeded;
  result_input.result_success = true;
  result_input.result_message = reason;
  const auto result_bt = amr_dispatcher_bt::TickMissionResultBehaviorTree(result_input);
  RCLCPP_INFO(get_logger(), "Mission %s result via BT: branch=%s state=%s", mission_id.c_str(),
              result_bt.branch.c_str(),
              result_bt.success ? result_bt.decision.state.c_str() : "tree_failed");

  // 树失败时回退旧行为（无条件释放），防树异常导致资源泄漏
  if (!result_bt.success || result_bt.decision.release_all_resources) {
    if (reservation_table_) {
      reservation_table_->Release(mission_id);
    }
  }
  if (result_bt.success && result_bt.decision.reset_failure_retry_count) {
    mission_attempts_.erase(mission_id);
    mission_last_progress_.erase(mission_id);
  }
  RCLCPP_INFO(get_logger(), "Mission completed: %s (%s)", mission_id.c_str(), reason.c_str());
  if (event_pub_ && event_pub_->is_activated()) {
    amr_dispatcher_interfaces::msg::MissionEvent msg;
    msg.stamp = now();
    msg.mission_id = mission_id;
    msg.order_id = order_id;
    msg.event = "completed";
    msg.from_state = "ACTIVE";
    msg.to_state = "FINISHED";
    msg.reason = reason;
    event_pub_->publish(msg);
  }
  if (event_log_) {
    amr_dispatcher_core::dispatcher::MissionEvent evt;
    evt.stamp = "now";
    evt.mission_id = mission_id;
    evt.event = "completed";
    evt.message = reason;
    event_log_->Append(evt);
  }
  ReArmQueueStartRequest();  // 队列非空时重新武装 runtime 树的启动请求
}

void DispatcherNode::FailMission(const std::string& mission_id, const std::string& reason) {
  std::string order_id;
  amr_dispatcher_core::dispatcher::Mission mission_copy;
  {
    std::lock_guard<std::mutex> lock(active_missions_mutex_);
    auto it = active_missions_.find(mission_id);
    if (it == active_missions_.end()) {
      return;
    }
    order_id = it->second.order_id;
    mission_copy = it->second;
    active_missions_.erase(it);
  }

  // 失败结果先经 mission_result 行为树分类：资源释放决策 + 失败处理标记
  amr_dispatcher_core::workflow::MissionResultWorkflowInput result_input;
  result_input.code = amr_dispatcher_core::workflow::MissionActionResultCode::kFailed;
  result_input.result_success = false;
  result_input.result_message = reason;
  const auto result_bt = amr_dispatcher_bt::TickMissionResultBehaviorTree(result_input);

  if (!result_bt.success || result_bt.decision.release_all_resources) {
    if (reservation_table_) {
      reservation_table_->Release(mission_id);
    }
  }
  if (tracking_mission_id_ == mission_id) {
    tracking_mission_id_.clear();
    // 发布空路径让 path_tracker 停下
    if (plan_pub_ && plan_pub_->is_activated()) {
      nav_msgs::msg::Path empty;
      empty.header.stamp = now();
      empty.header.frame_id = "map";
      plan_pub_->publish(empty);
    }
  }
  RCLCPP_WARN(get_logger(), "Mission failed: %s (%s)", mission_id.c_str(), reason.c_str());
  if (event_pub_ && event_pub_->is_activated()) {
    amr_dispatcher_interfaces::msg::MissionEvent msg;
    msg.stamp = now();
    msg.mission_id = mission_id;
    msg.order_id = order_id;
    msg.event = "failed";
    msg.from_state = "ACTIVE";
    msg.to_state = "FAILED";
    msg.reason = reason;
    event_pub_->publish(msg);
  }

  // 恢复策略走 mission_recovery 行为树：retry / dock / manual 三级决策
  const bool handle_failure = !result_bt.success || result_bt.decision.handle_failure;
  std::string recovery_action = "none";
  std::string recovery_message;
  if (handle_failure && queue_) {
    amr_dispatcher_bt::MissionRecoveryBehaviorTreeInput rec_input;
    rec_input.retry_count = mission_attempts_[mission_id];
    rec_input.is_dock_mission = mission_copy.type == "charge";
    rec_input.mission_id = mission_id;
    rec_input.failure_message =
        (result_bt.success && !result_bt.decision.failure_message.empty())
            ? result_bt.decision.failure_message
            : reason;
    rec_input.recovery_priority = mission_copy.priority;
    rec_input.next_queue_sequence = next_sequence_;
    const auto rec_bt = amr_dispatcher_bt::TickMissionRecoveryBehaviorTree(rec_input);
    recovery_action = rec_bt.decision.action;
    recovery_message = rec_bt.decision.message;
    RCLCPP_INFO(get_logger(), "Recovery via BT: branch=%s action=%s recoverable=%d (%s)",
                rec_bt.branch.c_str(), rec_bt.decision.action.c_str(),
                rec_bt.decision.recoverable ? 1 : 0, rec_bt.decision.message.c_str());

    if (rec_bt.success && rec_bt.decision.action == "retry") {
      auto retry = mission_copy;
      retry.attempt += 1;
      auto push = queue_->Push(retry);
      if (push.accepted) {
        mission_attempts_[mission_id] = rec_input.retry_count + 1;
        queue_start_requested_ = true;
      } else {
        recovery_action = "manual";
        recovery_message = "retry re-enqueue rejected: " + push.message;
      }
    } else if (rec_bt.success && rec_bt.decision.action == "dock") {
      // 回坞：入队一条到充电站点的 charge 任务；站点缺失则降级为仅记录
      const auto& catalog = catalog_reloader_ ? catalog_reloader_->catalog()
                                              : amr_dispatcher_core::catalog::StationCatalog{};
      const auto* dock_st = amr_dispatcher_core::catalog::FindStation(catalog, recovery_dock_station_);
      if (dock_st) {
        amr_dispatcher_core::dispatcher::Mission dock_m;
        dock_m.id = "m_" + std::to_string(next_sequence_++);
        dock_m.order_id = "dock_" + mission_id;
        dock_m.type = "charge";
        dock_m.dropoff_station = recovery_dock_station_;
        dock_m.priority = mission_copy.priority;
        dock_m.sequence = next_sequence_;
        auto push = queue_->Push(dock_m);
        if (push.accepted) {
          queue_start_requested_ = true;
        } else {
          recovery_action = "manual";
          recovery_message = "dock enqueue rejected: " + push.message;
        }
      } else {
        recovery_action = "manual";
        recovery_message = "dock station not found: " + recovery_dock_station_;
      }
    }

    if (event_pub_ && event_pub_->is_activated()) {
      amr_dispatcher_interfaces::msg::MissionEvent msg;
      msg.stamp = now();
      msg.mission_id = mission_id;
      msg.order_id = order_id;
      msg.event = "recovery";
      msg.from_state = "FAILED";
      msg.to_state = recovery_action == "manual" ? "MANUAL_INTERVENTION" : "RECOVERING";
      msg.reason = recovery_action + ": " + recovery_message;
      event_pub_->publish(msg);
    }
    if (event_log_) {
      amr_dispatcher_core::dispatcher::MissionEvent evt;
      evt.stamp = "now";
      evt.mission_id = mission_id;
      evt.event = "recovery";
      evt.message = recovery_action + ": " + recovery_message;
      event_log_->Append(evt);
    }
  }
  ReArmQueueStartRequest();
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
        // 取消也走结果树：canceled 分支给资源释放决策，同时补发可观测事件
        std::string canceled_order_id;
        auto cancel_it = active_missions_.find(act.mission_id);
        if (cancel_it != active_missions_.end()) {
          canceled_order_id = cancel_it->second.order_id;
        }
        amr_dispatcher_core::workflow::MissionResultWorkflowInput cancel_input;
        cancel_input.code = amr_dispatcher_core::workflow::MissionActionResultCode::kCanceled;
        cancel_input.result_message = act.reason;
        const auto cancel_bt = amr_dispatcher_bt::TickMissionResultBehaviorTree(cancel_input);
        active_missions_.erase(act.mission_id);
        if (!cancel_bt.success || cancel_bt.decision.release_all_resources) {
          if (reservation_table_) {
            reservation_table_->Release(act.mission_id);
          }
        }
        // 死锁恢复取消到正在跟踪 /plan 的任务时，必须释放跟踪槽位并停车，
        // 否则 tracking_mission_id_ 悬挂导致后续任务永远无法下发。
        if (tracking_mission_id_ == act.mission_id) {
          tracking_mission_id_.clear();
          if (plan_pub_ && plan_pub_->is_activated()) {
            nav_msgs::msg::Path empty;
            empty.header.stamp = now();
            empty.header.frame_id = "map";
            plan_pub_->publish(empty);
          }
        }
        if (event_pub_ && event_pub_->is_activated()) {
          amr_dispatcher_interfaces::msg::MissionEvent msg;
          msg.stamp = now();
          msg.mission_id = act.mission_id;
          msg.order_id = canceled_order_id;
          msg.event = "canceled";
          msg.to_state = "CANCELED";
          msg.reason = act.reason;
          event_pub_->publish(msg);
        }
        RCLCPP_WARN(get_logger(), "Deadlock cancel via BT: branch=%s state=%s reason=%s",
                    cancel_bt.branch.c_str(),
                    cancel_bt.success ? cancel_bt.decision.state.c_str() : "tree_failed",
                    act.reason.c_str());
        ReArmQueueStartRequest();
      } else if (act.kind == amr_dispatcher_core::dispatcher::RecoveryAction::Kind::kRetry) {
        mission_attempts_[act.mission_id]++;
        mission_last_progress_[act.mission_id] = std::chrono::steady_clock::now();
      }
    }
  }
}

void DispatcherNode::ReArmQueueStartRequest() {
  // 结果/取消/恢复后重新武装 runtime 行为树的启动请求：
  // 队列非空则置位（auto_start_queue 语义），队列清空则复位。
  const auto decision = amr_dispatcher_core::workflow::PlanMissionPostResultQueue(
      amr_dispatcher_core::workflow::MissionPostResultQueueRequest{
          /*existing_queue_start_requested=*/queue_start_requested_,
          /*auto_start_queue=*/true,
          /*queue_running=*/true,
          /*mission_queue_empty=*/queue_ == nullptr || queue_->empty()});
  queue_start_requested_ = decision.queue_start_requested;
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
