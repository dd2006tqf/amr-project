#include "amr_dispatcher_ros/dispatcher_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_set>
#include <utility>
#include <vector>

#include "amr_dispatcher_core/dispatcher/topology_projection.hpp"
#include "amr_dispatcher_core/fleet/vda5050_order_builder.hpp"
#include "amr_dispatcher_core/fleet/vda5050_serializer.hpp"
#include "amr_dispatcher_core/workflow/dynamic_unlocking.hpp"
#include "amr_dispatcher_core/workflow/traffic_planning.hpp"
#include "amr_dispatcher_ros/mission_identity.hpp"
#include "amr_dispatcher_ros/vehicle_topic_names.hpp"

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
  declare_parameter("navigation_backend", "plan");
  // 路径规划器类型（skeleton|mock）与地图版本门禁期望哈希（空 = 关闭门禁）
  declare_parameter("planner_type", "skeleton");
  declare_parameter("expected_map_hash", "");
  // Task 5：受管 VDA 5050 小车清单（逗号分隔 vehicle_id；空 = 关闭逐段解锁/断链守卫）
  // 与 state 陈旧告警阈值（秒，在线但超时未上报 -> 仅告警不释放）
  declare_parameter("vda5050_vehicles", "");
  declare_parameter("vda5050_state_stale_sec", 5.0);
  // P0.6 / A4：本调度器管理的 local_sim 车辆清单（逗号分隔 vehicle_id；空 = 单车兼容模式）
  declare_parameter("vehicle_ids", "");
  recovery_dock_station_ = get_parameter("recovery_dock_station").as_string();
  planner_type_ = get_parameter("planner_type").as_string();
  expected_map_hash_ = get_parameter("expected_map_hash").as_string();
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
  // 依 planner_type 选规划器：skeleton -> 混合图折线循迹，mock -> 直线基线。
  // catalog_reloader_ 已就位（即使初始加载失败也持有空目录，热重载后由 reload_timer_ 同步）。
  RebuildPlanner();

  state_pub_ = create_publisher<amr_dispatcher_interfaces::msg::DispatcherState>(
      "/dispatcher/state", 10);
  event_pub_ = create_publisher<amr_dispatcher_interfaces::msg::MissionEvent>(
      "/dispatcher/events", 20);
  plan_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 1);
  // 拓扑状态：safety_gate（deadlock_active_）与 visualizer（绿锁边/红警戒柱）的死锁输入
  topology_pub_ = create_publisher<amr_dispatcher_interfaces::msg::TopologyState>(
      "/dispatcher/topology_state", 10);

  // 位姿与到达信号订阅：path_tracker 的 TrackingError.arrived 驱动任务完成
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DispatcherNode::OdomCb, this, std::placeholders::_1));
  tracking_error_sub_ = create_subscription<amr_dispatcher_interfaces::msg::TrackingError>(
      "/tracking/error", 10,
      std::bind(&DispatcherNode::TrackingErrorCb, this, std::placeholders::_1));

  // Task 5：VDA 5050 状态/链路接入（逐段解锁 + 断链守卫），按参数建订阅
  SetupVda5050Links(get_parameter("vda5050_vehicles").as_string());

  // P0.6 / A4：per-vehicle 计划发布端与位姿/到达订阅（空清单 = 单车兼容，零行为变化）
  SetupVehicleTopics(get_parameter("vehicle_ids").as_string());

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
  force_clear_vda5050_link_srv_ =
      create_service<amr_dispatcher_interfaces::srv::ForceClearVda5050Link>(
          "/dispatcher/force_clear_vda5050_link",
          std::bind(&DispatcherNode::ForceClearVda5050LinkCb, this,
                    std::placeholders::_1, std::placeholders::_2));
  restore_vda5050_link_srv_ =
      create_service<amr_dispatcher_interfaces::srv::RestoreVda5050Link>(
          "/dispatcher/restore_vda5050_link",
          std::bind(&DispatcherNode::RestoreVda5050LinkCb, this,
                    std::placeholders::_1, std::placeholders::_2));

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

  // 导航后端：默认 plan（行为与既有代码逐字节一致）；action 模式派发改发 NavigateSequence 单点 goal
  navigation_backend_ = get_parameter("navigation_backend").as_string();
  if (navigation_backend_ != "plan" && navigation_backend_ != "action") {
    RCLCPP_WARN(get_logger(), "Unknown navigation_backend '%s', falling back to 'plan'",
                navigation_backend_.c_str());
    navigation_backend_ = "plan";
  }
  // ExecuteMission action server：HandleGoal 校验（非空/不重复/资源可预占），
  // HandleAccepted 只做入队 + 存 goal handle（不起线程）
  execute_action_server_ = rclcpp_action::create_server<ExecuteMission>(
      this, "/execute_mission",
      std::bind(&DispatcherNode::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DispatcherNode::HandleCancel, this, std::placeholders::_1),
      std::bind(&DispatcherNode::HandleAccepted, this, std::placeholders::_1));
  if (navigation_backend_ == "action") {
    nav_client_ = rclcpp_action::create_client<NavigateSequence>(this, "/navigate_sequence");
  }

  // B6：参数热重载——比较器与三个调度节奏参数允许运行时改写。
  // 校验失败返回空 reason（rclcpp 按"拒绝"处理）；comparator 在此预校验，
  // 未知值直接拒绝（避免 SetComparator 对未知名静默失败导致实际策略与参数漂移）。
  param_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
          const std::string& name = p.get_name();
          if (name == "comparator") {
            const std::string v = p.as_string();
            auto comp = amr_dispatcher_core::dispatcher::CreateMissionComparator(v);
            if (!comp) {
              result.successful = false;
              result.reason = "unknown comparator: " + v;
              RCLCPP_WARN(get_logger(), "Parameter reload rejected: %s",
                          result.reason.c_str());
              continue;
            }
            if (queue_) {
              queue_->SetComparator(std::move(comp));
            }
            RCLCPP_INFO(get_logger(), "Parameter reload: comparator -> %s", v.c_str());
          } else if (name == "schedule_rate_hz") {
            const double v = p.as_double();
            if (v <= 0.0) {
              result.successful = false;
              result.reason = "schedule_rate_hz must be > 0";
              continue;
            }
            if (schedule_timer_) {
              schedule_timer_->reset();
              schedule_timer_ = create_wall_timer(
                  std::chrono::duration<double>(1.0 / v),
                  std::bind(&DispatcherNode::ScheduleTick, this));
            }
            RCLCPP_INFO(get_logger(), "Parameter reload: schedule_rate_hz -> %.2f", v);
          } else if (name == "deadlock_check_rate_hz") {
            const double v = p.as_double();
            if (v <= 0.0) {
              result.successful = false;
              result.reason = "deadlock_check_rate_hz must be > 0";
              continue;
            }
            if (deadlock_timer_) {
              deadlock_timer_->reset();
              deadlock_timer_ = create_wall_timer(
                  std::chrono::duration<double>(1.0 / v),
                  std::bind(&DispatcherNode::DeadlockTick, this));
            }
            RCLCPP_INFO(get_logger(), "Parameter reload: deadlock_check_rate_hz -> %.2f", v);
          } else if (name == "max_active_missions") {
            const int64_t v = p.as_int();
            if (v < 1) {
              result.successful = false;
              result.reason = "max_active_missions must be >= 1";
              continue;
            }
            RCLCPP_INFO(get_logger(), "Parameter reload: max_active_missions -> %lld",
                        static_cast<long long>(v));
          } else if (name == "planner_type") {
            const std::string v = p.as_string();
            if (v != "mock" && v != "skeleton") {
              result.successful = false;
              result.reason = "planner_type must be 'mock' or 'skeleton': " + v;
              RCLCPP_WARN(get_logger(), "Parameter reload rejected: %s",
                          result.reason.c_str());
              continue;
            }
            planner_type_ = v;
            RebuildPlanner();  // 单线程 spin：回调期间无规划请求在途，可安全换实例
            RCLCPP_INFO(get_logger(), "Parameter reload: planner_type -> %s", v.c_str());
          } else if (name == "expected_map_hash") {
            expected_map_hash_ = p.as_string();
            RCLCPP_INFO(get_logger(),
                        "Parameter reload: expected_map_hash -> %s",
                        expected_map_hash_.empty() ? "<disabled>"
                                                   : expected_map_hash_.c_str());
          }
          // 其余已声明参数不回执：rclcpp 仍正常赋值，仅不产生日志噪声
        }
        return result;
      });

  RCLCPP_INFO(get_logger(),
              "DispatcherNode configured (capacity=%zu, comparator=%s, navigation_backend=%s)",
              cap, comp_name.c_str(), navigation_backend_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DispatcherNode::on_activate(const rclcpp_lifecycle::State& /*state*/) {
  state_pub_->on_activate();
  event_pub_->on_activate();
  plan_pub_->on_activate();
  topology_pub_->on_activate();
  for (auto& [vehicle_id, pub] : vda5050_order_pubs_) {
    pub->on_activate();  // Task 6：send_order 发布端随节点激活（未激活则 publish 为空操作）
  }
  for (auto& [vehicle_id, pub] : vehicle_plan_pubs_) {
    pub->on_activate();  // P0.6：per-vehicle 计划发布端随节点激活
  }

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
      // SkeletonPlanner 持有目录副本：热重载后必须同步，否则仍按旧折线规划
      auto* skeleton_planner =
          dynamic_cast<amr_dispatcher_core::planning::SkeletonPlanner*>(planner_.get());
      if (skeleton_planner != nullptr) {
        skeleton_planner->UpdateCatalog(catalog_reloader_->catalog());
        RCLCPP_INFO(get_logger(), "SkeletonPlanner catalog synchronized (%zu stations)",
                    catalog_reloader_->catalog().stations.size());
      }
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

void DispatcherNode::RebuildPlanner() {
  const double step_size = get_parameter("path_step_size_m").as_double();
  if (planner_type_ == "skeleton") {
    planner_ = std::make_unique<amr_dispatcher_core::planning::SkeletonPlanner>(
        catalog_reloader_ ? catalog_reloader_->catalog()
                          : amr_dispatcher_core::catalog::StationCatalog{},
        amr_dispatcher_core::planning::SkeletonPlanner::Config{step_size});
    RCLCPP_INFO(get_logger(),
                "Path planner: SkeletonPlanner (mixed-graph polyline, step %.2f m)",
                step_size);
    return;
  }
  if (planner_type_ != "mock") {
    RCLCPP_WARN(get_logger(),
                "Unknown planner_type '%s', falling back to MockLinearPlanner",
                planner_type_.c_str());
  }
  planner_ = std::make_unique<amr_dispatcher_core::MockLinearPlanner>(step_size);
  RCLCPP_INFO(get_logger(), "Path planner: MockLinearPlanner (step %.2f m)", step_size);
}

CallbackReturn DispatcherNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
  schedule_timer_.reset();
  deadlock_timer_.reset();
  state_timer_.reset();
  reload_timer_.reset();
  state_pub_->on_deactivate();
  event_pub_->on_deactivate();
  plan_pub_->on_deactivate();
  topology_pub_->on_deactivate();
  for (auto& [vehicle_id, pub] : vda5050_order_pubs_) {
    pub->on_deactivate();
  }
  for (auto& [vehicle_id, pub] : vehicle_plan_pubs_) {
    pub->on_deactivate();
  }
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
  force_clear_vda5050_link_srv_.reset();
  restore_vda5050_link_srv_.reset();
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
  // B2：clean up 前先逐个收口在途 ExecuteMission goal——
  // 不复位的 handle 在 server 销毁后会让 action 客户端永久悬挂；
  // 同时释放 HandleGoal 阶段预占的 required_resources 锁。
  // rclcpp_action 状态机约束：canceled() 仅 CANCELING、abort() 仅 EXECUTING，
  // 其余状态（ACCEPTED 未执行）先 execute() 再 abort()，与 FinishExecuteGoal 同构。
  for (auto& [mission_id, handle] : execute_goal_handles_) {
    if (!handle) {
      continue;
    }
    if (reservation_table_) {
      reservation_table_->Release(mission_id);
    }
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->error_code = 1;
    result->message = "dispatcher cleaning up";
    try {
      if (handle->is_canceling()) {
        handle->canceled(result);
      } else if (handle->is_executing()) {
        handle->abort(result);
      } else {
        handle->execute();
        handle->abort(result);
      }
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "on_cleanup: finish goal %s failed: %s",
                   mission_id.c_str(), ex.what());
    }
  }
  execute_action_server_.reset();
  nav_client_.reset();
  execute_goal_handles_.clear();
  mission_dispatch_start_.clear();
  mission_dispatch_time_.clear();
  pending_wait_.clear();
  mission_reserved_at_.clear();
  pending_nav_mission_id_.clear();
  pending_nav_goal_handle_.reset();
  state_pub_.reset();
  event_pub_.reset();
  plan_pub_.reset();
  topology_pub_.reset();
  vda5050_order_pubs_.clear();   // Task 6：随生命周期收口，重 configure 时重建
  vda5050_vehicle_ids_.clear();
  vda5050_links_.clear();        // 终审 M4：与 SetupVda5050Links 开头清旧对齐，防残留链
  vda5050_next_order_update_id_.clear();
  vda5050_mission_vehicle_.clear();  // Task 7：mission->车辆绑定随生命周期收口
  vda5050_quarantined_.clear();      // Task 7：隔离态随生命周期收口
  // P0.6：per-vehicle 资源随生命周期收口，重 configure 时重建
  vehicle_plan_pubs_.clear();
  vehicle_odom_subs_.clear();
  vehicle_tracking_error_subs_.clear();
  vehicle_poses_.clear();
  vehicle_ids_.clear();
  last_deadlock_findings_.clear();
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
  tracking_vehicle_id_.clear();
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
  // B4：id/sequence 同源——自动 id 用戳记器保证 "m_<n>" 与 sequence=n 一致；
  // 调用方显式给 id 时仍消费一次序列号，保持全局单调。
  if (req->mission_id.empty()) {
    StampMissionIdentity(m, next_sequence_);
  } else {
    m.id = req->mission_id;
    m.sequence = next_sequence_++;
  }
  m.order_id = req->order_id.empty() ? ("ord_" + m.id) : req->order_id;
  m.type = route.normalized_order_type.empty() ? "transport" : route.normalized_order_type;
  // 站点字段优先取请求显式值，路由器解析出的站点作兜底
  m.pickup_station = req->pickup_station.empty() ? route.pickup_station : req->pickup_station;
  m.dropoff_station = req->dropoff_station.empty() ? route.dropoff_station : req->dropoff_station;
  m.priority = req->priority;
  m.deadline_unix_ms = req->deadline_unix_ms;
  m.expected_seconds = req->expected_seconds;
  // 显式指定承接车辆（空 = 由选车策略决定，见 core/fleet/vehicle_assignment.hpp）
  m.assigned_vehicle_id = req->assigned_vehicle_id;

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
      // action 提交的任务在排队中被取消：helper 终结 handle + 释放 HandleGoal 预占
      CleanupMissionCancel(req->mission_id, "mission canceled by operator");
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
          PublishStopPathForTrackedVehicle();
        }
        active_missions_.erase(it);
        // helper：终结 ExecuteMission goal（未受理 action cancel → 降级 abort(3)）
        // 并级联取消在途 NavigateSequence goal（清 pending_nav_*）
        CleanupMissionCancel(req->mission_id, "mission canceled by operator");
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
    // 整单批量取消：取消前对在队任务取证（仅存在 action handle 时取证，plan 模式零开销），
    // 取消成功后逐个收口 handle/预占/pending_nav（审查 I5）
    std::vector<std::string> action_mission_ids;
    if (!execute_goal_handles_.empty()) {
      for (const auto& qm : queue_->Snapshot()) {
        if (qm.order_id == req->order_id) {
          action_mission_ids.push_back(qm.id);
        }
      }
    }
    std::size_t count = queue_->CancelOrder(req->order_id);
    for (const auto& id : action_mission_ids) {
      CleanupMissionCancel(id, "mission canceled by operator");
    }
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

void DispatcherNode::ForceClearVda5050LinkCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ForceClearVda5050Link::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ForceClearVda5050Link::Response> resp) {
  // Task 7（终审 I1 闭合）：核心纯函数决策（fleet::PlanForceClearLink）+ 本地落地执行。
  // 决策规则集中在核心（零 ROS 依赖，可单测）：仅受理"受管清单内且当前离线"的车辆；
  // 锁归属必须经 mission->vehicle 绑定反查（严禁用"路径存档存在"之类的间接证据推断）。
  amr_dispatcher_core::fleet::ForceClearLinkRequest decision_req;
  decision_req.vehicle_id = req->vehicle_id;
  decision_req.confirm_vehicle_id = req->confirm_vehicle_id;
  decision_req.operator_id = req->operator_id;
  decision_req.reason = req->reason;
  decision_req.vehicle_in_managed_list =
      vda5050_links_.find(req->vehicle_id) != vda5050_links_.end();
  const auto link_it = vda5050_links_.find(req->vehicle_id);
  decision_req.vehicle_online = (link_it != vda5050_links_.end()) && link_it->second.online;
  decision_req.vehicle_quarantined =
      vda5050_quarantined_.find(req->vehicle_id) != vda5050_quarantined_.end();
  decision_req.mission_vehicle_bindings = vda5050_mission_vehicle_;
  if (reservation_table_) {
    for (const auto& [resource_id, owner] : reservation_table_->Snapshot()) {
      decision_req.held_locks[resource_id] = owner;
    }
  }
  const auto decision = amr_dispatcher_core::fleet::PlanForceClearLink(decision_req);
  if (!decision.allowed) {
    resp->success = false;
    resp->message = decision.reject_reason;
    return;
  }

  // ---- 1) 释放该车名下锁（先于任务收口，保证 released_resources 可取证）----
  std::vector<std::string> released;
  if (reservation_table_) {
    for (const auto& [mission_id, ids] : decision.locks_by_mission) {
      const auto freed = reservation_table_->ReleaseResources(mission_id, ids);
      released.insert(released.end(), freed.begin(), freed.end());
    }
  }
  resp->released_resources.assign(released.begin(), released.end());

  // ---- 2) 收口该车承接的在途任务 ----
  // 车辆已死，其承接的 mission 永远不会到达/完成，会一直占着 active_missions_ 与
  // 单车执行槽位，后续任务永远排不上（挂起守卫解除也派不出去）。
  // 人工强清 = 操作员宣告该任务作废：**全量释放 + 不重试 + 不回坞**，让队列立刻前进。
  // 不能走 FailMission：其恢复行为树可能判定 retry/dock（把死车任务重新入队），
  // 且资源释放取决于 BT 决策（可能保留锁），与"人工强解"语义相悖。
  for (const auto& mission_id : decision.missions_to_abort) {
    RCLCPP_WARN(get_logger(),
                "VDA 5050 force-clear: aborting in-flight mission %s bound to dead vehicle %s",
                mission_id.c_str(), req->vehicle_id.c_str());
    // 单车执行槽位释放 + 停车（否则槽位残留，后续任务永远无法派发）
    if (tracking_mission_id_ == mission_id) {
      tracking_mission_id_.clear();
      PublishStopPathForTrackedVehicle();
    }
    {
      std::lock_guard<std::mutex> lock(active_missions_mutex_);
      active_missions_.erase(mission_id);
    }
    if (reservation_table_) {
      reservation_table_->Release(mission_id);  // 全量释放该任务名下锁
    }
    FinishExecuteGoal(mission_id, ExecuteFinish::kAbort,
                      "operator force-cleared offline VDA 5050 link", 1);
    mission_dispatch_time_.erase(mission_id);
    ForgetMissionDispatchState(mission_id);
    if (event_pub_ && event_pub_->is_activated()) {
      amr_dispatcher_interfaces::msg::MissionEvent msg;
      msg.stamp = now();
      msg.mission_id = mission_id;
      msg.event = "failed";
      msg.from_state = "ACTIVE";
      msg.to_state = "FAILED";
      msg.reason = "operator force-cleared offline VDA 5050 link for vehicle " + req->vehicle_id;
      event_pub_->publish(msg);
    }
  }
  ReArmQueueStartRequest();  // 队列后续任务需重新武装启动请求

  // ---- 3) 车辆转入隔离态（**不摘除**受管清单）----
  // 评审 P0.5：通信失联不等于可以释放路权。强清后车辆仍然可见、可审计，
  // 但不再参与自动分配（选车时 quarantined -> 不合格），需显式恢复。
  vda5050_quarantined_.insert(req->vehicle_id);
  resp->quarantined = true;

  // ---- 4) 审计与响应 ----
  resp->success = true;
  resp->message = "Force-cleared offline link for " + req->vehicle_id + ", released " +
                  std::to_string(released.size()) + " resource(s) | reason: " + req->reason +
                  " | operator: " + req->operator_id +
                  " | vehicle QUARANTINED (not auto-assignable until restored; physical "
                  "position NOT machine-verified — operator bears that risk)";
  RCLCPP_WARN(get_logger(),
              "VDA 5050 link %s FORCE-CLEARED by operator %s (reason=\"%s\", released %zu "
              "resource(s) [%s]) -> vehicle QUARANTINED; physical position NOT verified",
              req->vehicle_id.c_str(), req->operator_id.c_str(), req->reason.c_str(),
              released.size(),
              [&released]() {
                std::string joined;
                for (std::size_t i = 0; i < released.size(); ++i) {
                  if (i) joined += ",";
                  joined += released[i];
                }
                return joined;
              }()
                  .c_str());
}

void DispatcherNode::RestoreVda5050LinkCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::RestoreVda5050Link::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::RestoreVda5050Link::Response> resp) {
  // 评审 P0.5：恢复 = 解除隔离，使车辆重新参与自动分配。
  // 准入校验下沉核心（PlanRestoreLink）：须已隔离 + 在线 + 操作员已核实位置。
  amr_dispatcher_core::fleet::RestoreLinkRequest decision_req;
  decision_req.vehicle_id = req->vehicle_id;
  decision_req.confirm_vehicle_id = req->confirm_vehicle_id;
  decision_req.operator_id = req->operator_id;
  decision_req.reason = req->reason;
  decision_req.vehicle_in_managed_list =
      vda5050_links_.find(req->vehicle_id) != vda5050_links_.end();
  decision_req.vehicle_quarantined =
      vda5050_quarantined_.find(req->vehicle_id) != vda5050_quarantined_.end();
  const auto link_it = vda5050_links_.find(req->vehicle_id);
  decision_req.vehicle_online = (link_it != vda5050_links_.end()) && link_it->second.online;
  decision_req.position_verified = req->position_verified;

  const auto decision = amr_dispatcher_core::fleet::PlanRestoreLink(decision_req);
  if (!decision.allowed) {
    resp->success = false;
    resp->message = decision.reject_reason;
    return;
  }
  vda5050_quarantined_.erase(req->vehicle_id);
  resp->success = true;
  resp->message = "Restored vehicle " + req->vehicle_id + " to dispatch pool | reason: " +
                  req->reason + " | operator: " + req->operator_id;
  RCLCPP_WARN(get_logger(),
              "VDA 5050 link %s RESTORED by operator %s (reason=\"%s\", position_verified=%d) "
              "-> vehicle re-enters automatic assignment",
              req->vehicle_id.c_str(), req->operator_id.c_str(), req->reason.c_str(),
              req->position_verified ? 1 : 0);
  ReArmQueueStartRequest();  // 池中多了一台车 -> 唤醒排队任务重试
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
  StampMissionIdentity(m, next_sequence_);  // B4：id/sequence 同源
  m.order_id = req->mission_file;
  m.priority = req->priority;
  m.state = amr_dispatcher_core::dispatcher::MissionState::kPending;
  // 显式指定承接车辆（空 = 由选车策略决定）
  m.assigned_vehicle_id = req->assigned_vehicle_id;
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
  if (canceled) {
    // 排队取消命中：收口该任务的 action goal/handle 预占/pending_nav（审查 I5）
    CleanupMissionCancel(req->mission_id, "mission canceled by operator");
  } else {
    // 按整单取消：取消前取证该订单在队任务，取消成功后逐个收口（审查 I5）
    std::vector<std::string> action_mission_ids;
    if (!execute_goal_handles_.empty()) {
      for (const auto& qm : queue_->Snapshot()) {
        if (qm.order_id == req->mission_id) {
          action_mission_ids.push_back(qm.id);
        }
      }
    }
    const std::size_t order_count = queue_->CancelOrder(req->mission_id);
    canceled = (order_count > 0);
    for (const auto& id : action_mission_ids) {
      CleanupMissionCancel(id, "mission canceled by operator");
    }
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
        PublishStopPathForTrackedVehicle();
      }
      // 活动任务取消同样收口 action goal 与在途导航 goal（cancel_active 路径，审查 I5）
      CleanupMissionCancel(req->mission_id, "mission canceled by operator");
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
  StampMissionIdentity(m, next_sequence_, "preempt_");  // B4：id/sequence 同源
  m.order_id = req->mission_file;
  m.priority = req->priority > 100 ? req->priority : 100;
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
  // B5：真实现——mission_file 作为站点对编码（"from->to" 或 "from:to"），
  // 用 StationCatalog 直线距离和喂 EstimateDistanceCost；电池参数无 srv 字段，
  // 沿用核心测试约定（0.1 V/m，最低 21.0 V 与 charging workflow 阈值一致）。
  resp->mission_id = req->mission_file;

  const std::string& spec = req->mission_file;
  std::string from;
  std::string to;
  const auto arrow = spec.find("->");
  if (arrow != std::string::npos) {
    from = spec.substr(0, arrow);
    to = spec.substr(arrow + 2);
  } else {
    const auto colon = spec.find(':');
    if (colon != std::string::npos) {
      from = spec.substr(0, colon);
      to = spec.substr(colon + 1);
    }
  }
  if (from.empty() || to.empty()) {
    resp->success = false;
    resp->message = "mission_file must encode a station pair as 'from->to' or 'from:to', got: " + spec;
    return;
  }
  if (!catalog_reloader_ || !catalog_reloader_->loaded()) {
    resp->success = false;
    resp->message = "station catalog not loaded";
    return;
  }
  const auto& catalog = catalog_reloader_->catalog();
  const auto* from_st = amr_dispatcher_core::catalog::FindStation(catalog, from);
  const auto* to_st = amr_dispatcher_core::catalog::FindStation(catalog, to);
  if (!from_st || !to_st) {
    resp->success = false;
    resp->message = "unknown station(s) in mission_file '" + spec + "'";
    return;
  }

  const double distance_m =
      std::hypot(to_st->x - from_st->x, to_st->y - from_st->y);
  constexpr int kTwoStationWaypoints = 2;  // 取/送 2 站点 = 1 航段
  constexpr double kBatteryDropPerMeter = 0.1;    // 单位里程压降（V/m）
  constexpr double kMinimumBatteryVoltage = 21.0; // 与 charging workflow 阈值一致

  const auto estimate = amr_dispatcher_core::workflow::EstimateDistanceCost(
      distance_m, kTwoStationWaypoints, req->nominal_speed_mps, req->battery_voltage,
      kBatteryDropPerMeter, kMinimumBatteryVoltage, "station_catalog_straight");

  resp->success = true;
  resp->message = "estimated via station catalog straight-line distance";
  resp->waypoint_count = estimate.waypoint_count;
  resp->distance_m = estimate.distance_m;
  resp->eta_sec = estimate.eta_sec;
  resp->battery_drop_v = estimate.battery_drop_v;
  resp->projected_battery_voltage = estimate.projected_battery_voltage;
  resp->battery_sufficient = estimate.battery_sufficient;
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
  // 快照组包与 DeadlockTick 共用 helper：服务入口与定时检测结论不分叉；
  // 沿用本回调既有的 active_missions_ 加锁约定（helper 自身不带锁）
  amr_dispatcher_core::dispatcher::DeadlockSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(active_missions_mutex_);
    snap = BuildDeadlockSnapshot();
  }
  const auto findings = deadlock_detector_->Detect(snap);
  resp->success = true;
  resp->deadlocked = !findings.empty();
  resp->message = resp->deadlocked ? "Deadlock detected in graph" : "No deadlock detected";
  for (const auto& f : findings) {
    for (const auto& mid : f.mission_ids) {
      resp->mission_ids.push_back(mid);
    }
    // cycle_path 为任务/资源交替闭合序列：奇数位才是资源 id（任务 id 不得混入）
    for (std::size_t i = 1; i < f.cycle_path.size(); i += 2) {
      resp->conflict_resource_ids.push_back(f.cycle_path[i]);
    }
    resp->descriptions.push_back(f.description);
  }
}

void DispatcherNode::ListStationsCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ListStations::Request> /*req*/,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ListStations::Response> resp) {
  // B5：真实现——从 StationCatalogReloader 投影，目录未加载即报不可用
  if (!catalog_reloader_ || !catalog_reloader_->loaded()) {
    resp->success = false;
    resp->message = "station catalog not loaded";
    return;
  }
  const auto projection = amr_dispatcher_core::catalog::BuildStationListProjection(
      catalog_reloader_->catalog());
  resp->success = true;
  resp->message = projection.message.empty() ? "OK" : projection.message;
  resp->station_ids = projection.station_ids;
  resp->frame_ids = projection.frame_ids;
  resp->x = projection.x;
  resp->y = projection.y;
  resp->yaw = projection.yaw;
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
    const std::shared_ptr<amr_dispatcher_interfaces::srv::GetOperatorSnapshot::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::GetOperatorSnapshot::Response> resp) {
  // B5：真实现——BuildOperatorSnapshotProjection 字段全量投影。
  // 无 fleet/facility/business 后端时对应字段传 nullptr 透传为空集合。
  amr_dispatcher_core::workflow::OperatorSnapshotRequest snap_req;
  {
    std::lock_guard<std::mutex> lock(active_missions_mutex_);
    snap_req.mission_active = !active_missions_.empty();
    if (!active_missions_.empty()) {
      snap_req.active_mission_id = active_missions_.begin()->first;
      snap_req.mission_state = "ACTIVE";
    } else {
      snap_req.mission_state = "IDLE";
    }
  }

  if (queue_) {
    for (const auto& m : queue_->Snapshot()) {
      amr_dispatcher_core::workflow::QueuedMission qm;
      qm.profile.mission_id = m.id;
      qm.mission_file = m.order_id;
      qm.priority = m.priority;
      qm.sequence = m.sequence;
      snap_req.mission_queue.push_back(qm);
      // 队列按 order_id 记暂停：探测本任务所属订单是否被暂停
      if (!m.order_id.empty() && queue_->IsOrderPaused(m.order_id)) {
        snap_req.paused_workflow_order_ids.insert(m.order_id);
      }
    }
    snap_req.paused = !snap_req.paused_workflow_order_ids.empty();
  }

  if (event_log_) {
    const std::size_t limit = amr_dispatcher_core::dispatcher::MissionEventLog::NormalizeLimit(
        req->event_limit, 10, 100);
    for (const auto& evt : event_log_->Recent(limit)) {
      amr_dispatcher_core::workflow::MissionEvent wfe;
      wfe.stamp = evt.stamp;
      wfe.mission_id = evt.mission_id;
      wfe.state = evt.event;          // dispatcher 事件名 -> workflow 状态字段
      wfe.previous_state = evt.previous_event;
      wfe.message = evt.message;
      wfe.recoverable = evt.recoverable;
      snap_req.mission_events.push_back(wfe);
    }
  }
  snap_req.event_limit = req->event_limit;

  const auto proj = amr_dispatcher_core::workflow::BuildOperatorSnapshotProjection(snap_req);
  resp->success = proj.success;
  resp->message = proj.message;
  resp->mission_state = proj.mission_state;
  resp->previous_mission_state = proj.previous_mission_state;
  resp->active_mission_id = proj.active_mission_id;
  resp->mission_message = proj.mission_message;
  resp->recoverable = proj.recoverable;
  resp->mission_active = proj.mission_active;
  resp->paused = proj.paused;
  resp->queue_size = proj.queue_size;
  resp->queued_mission_ids = proj.queued_mission_ids;
  resp->queued_priorities = proj.queued_priorities;
  resp->paused_workflow_order_ids = proj.paused_workflow_order_ids;
  resp->fleet_robot_ids = proj.fleet_robot_ids;
  resp->fleet_enabled = proj.fleet_enabled;
  resp->fleet_states = proj.fleet_states;
  resp->fleet_battery_voltage = proj.fleet_battery_voltage;
  resp->fleet_current_station_ids = proj.fleet_current_station_ids;
  resp->fleet_capabilities = proj.fleet_capabilities;
  resp->remote_task_ids = proj.remote_task_ids;
  resp->remote_task_robot_ids = proj.remote_task_robot_ids;
  resp->remote_task_states = proj.remote_task_states;
  resp->facility_resource_ids = proj.facility_resource_ids;
  resp->facility_resource_types = proj.facility_resource_types;
  resp->facility_station_ids = proj.facility_station_ids;
  resp->facility_available = proj.facility_available;
  resp->facility_status = proj.facility_status;
  resp->business_types = proj.business_types;
  resp->business_scenario_ids = proj.business_scenario_ids;
  resp->business_workflow_ids = proj.business_workflow_ids;
  resp->recent_event_stamp = proj.recent_event_stamp;
  resp->recent_event_mission_ids = proj.recent_event_mission_ids;
  resp->recent_event_states = proj.recent_event_states;
  resp->recent_event_messages = proj.recent_event_messages;
}

void DispatcherNode::ValidateSiteConfigCb(
    const std::shared_ptr<amr_dispatcher_interfaces::srv::ValidateSiteConfig::Request> req,
    std::shared_ptr<amr_dispatcher_interfaces::srv::ValidateSiteConfig::Response> resp) {
  // B5：真实现——真实尝试加载 stations_file；同时校验 dispatcher 最小参数集合。
  // bundle_dir 指定 bundle 时优先用 <bundle_dir>/stations.yaml，否则回退节点参数路径。
  resp->version_id = req->version_id.empty() ? "v1.0.0" : req->version_id;

  std::string stations_path = get_parameter("stations_file").as_string();
  if (!req->bundle_dir.empty()) {
    stations_path = req->bundle_dir + "/stations.yaml";
  }
  resp->checked_files.push_back(stations_path);

  const auto catalog_opt = amr_dispatcher_core::catalog::LoadStationCatalog(stations_path);
  if (!catalog_opt) {
    resp->success = false;
    resp->errors.push_back("stations_file load failed: " + stations_path);
    resp->message = "validation failed: " + stations_path + " not parseable";
    return;
  }
  if (catalog_opt->stations.empty()) {
    resp->success = false;
    resp->errors.push_back("stations catalog empty: " + stations_path);
    resp->message = "validation failed: no stations defined";
    return;
  }

  // 最小参数集合校验：任一失败即整体失败
  const int queue_capacity = get_parameter("queue_capacity").as_int();
  if (queue_capacity <= 0) {
    resp->errors.push_back("queue_capacity must be > 0, got " + std::to_string(queue_capacity));
  }
  const double sched_hz = get_parameter("schedule_rate_hz").as_double();
  if (sched_hz <= 0.0) {
    resp->errors.push_back("schedule_rate_hz must be > 0, got " + std::to_string(sched_hz));
  }
  const double deadlock_hz = get_parameter("deadlock_check_rate_hz").as_double();
  if (deadlock_hz <= 0.0) {
    resp->errors.push_back("deadlock_check_rate_hz must be > 0, got " + std::to_string(deadlock_hz));
  }
  const int max_active = get_parameter("max_active_missions").as_int();
  if (max_active < 1) {
    resp->errors.push_back("max_active_missions must be >= 1, got " + std::to_string(max_active));
  }
  const std::string comparator = get_parameter("comparator").as_string();
  if (!amr_dispatcher_core::dispatcher::CreateMissionComparator(comparator)) {
    resp->errors.push_back("unknown comparator: " + comparator);
  }

  resp->success = resp->errors.empty();
  resp->message = resp->success
                      ? "site configuration validation passed"
                      : "validation failed: " + std::to_string(resp->errors.size()) + " error(s)";
}

rclcpp_action::GoalResponse DispatcherNode::HandleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteMission::Goal> goal) {
  if (!goal || goal->mission_id.empty()) {
    RCLCPP_WARN(get_logger(), "ExecuteMission goal rejected: empty mission_id");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if ((queue_ && queue_->Contains(goal->mission_id)) ||
      active_missions_.count(goal->mission_id) > 0) {
    RCLCPP_WARN(get_logger(), "ExecuteMission goal rejected: duplicate mission_id %s",
                goal->mission_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // required_resources 可预占：Reserve 原子性（不落任何锁即失败），失败即 REJECT
  if (!goal->required_resources.empty()) {
    if (!reservation_table_) {
      RCLCPP_WARN(get_logger(),
                  "ExecuteMission goal rejected: reservation table unavailable for %s",
                  goal->mission_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    auto res = reservation_table_->Reserve(goal->mission_id, goal->required_resources);
    if (!res.success) {
      RCLCPP_WARN(get_logger(), "ExecuteMission goal rejected: reserve failed for %s: %s",
                  goal->mission_id.c_str(), res.message.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  // DEFER + HandleAccepted 里显式 execute()：不受框架是否隐式 execute 影响的稳妥路径
  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse DispatcherNode::HandleCancel(
    const std::shared_ptr<GoalHandleExecute> goal_handle) {
  if (execute_goal_handles_.find(goal_handle->get_goal()->mission_id) ==
      execute_goal_handles_.end()) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  // 只受理取消请求：rclcpp_action 在本回调返回 ACCEPT 之后才把 goal 迁移到
  // CANCELING，而 canceled() 只允许在 CANCELING 态调用——
  // 实际的任务取消与 goal 终结延后到 ProcessActionCancels（ScheduleTick 内）。
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DispatcherNode::HandleAccepted(const std::shared_ptr<GoalHandleExecute> goal_handle) {
  const auto goal = goal_handle->get_goal();
  // ACCEPTED -> EXECUTING：后续 publish_feedback/succeed/abort 的前置条件
  goal_handle->execute();

  if (!queue_) {
    if (reservation_table_) {
      reservation_table_->Release(goal->mission_id);
    }
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->error_code = 1;
    result->message = "dispatcher not configured";
    goal_handle->abort(result);
    return;
  }

  amr_dispatcher_core::dispatcher::Mission m;
  m.id = goal->mission_id;
  m.order_id = goal->mission_id;  // 订单号=任务号，保证整单取消可寻址
  m.priority = goal->priority;
  m.dropoff_station = goal->target_station;
  if (goal->deadline_seconds > 0.0) {
    m.deadline_unix_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count() +
        static_cast<std::int64_t>(goal->deadline_seconds * 1000.0);
  }
  m.state = amr_dispatcher_core::dispatcher::MissionState::kPending;
  m.sequence = next_sequence_++;

  auto push = queue_->Push(m);
  if (!push.accepted) {
    // 入队失败回滚 HandleGoal 的资源预占并 abort（死锁风险下的显式失败出口）
    if (reservation_table_) {
      reservation_table_->Release(m.id);
    }
    auto result = std::make_shared<ExecuteMission::Result>();
    result->success = false;
    result->error_code = 1;
    result->message = "enqueue rejected: " + push.message;
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(), "ExecuteMission enqueue rejected for %s: %s", m.id.c_str(),
                push.message.c_str());
    return;
  }

  execute_goal_handles_[m.id] = goal_handle;
  queue_start_requested_ = true;  // action 提交的任务武装 runtime 出队启动请求
  RCLCPP_INFO(get_logger(),
              "ExecuteMission accepted: %s -> queued (target=%s, prio=%d, resources=%zu)",
              m.id.c_str(), m.dropoff_station.c_str(), m.priority,
              goal->required_resources.size());
}

void DispatcherNode::FinishExecuteGoal(const std::string& mission_id, ExecuteFinish kind,
                                        const std::string& message, std::int32_t error_code) {
  auto it = execute_goal_handles_.find(mission_id);
  if (it == execute_goal_handles_.end()) {
    return;
  }
  auto handle = it->second;
  execute_goal_handles_.erase(it);

  // total_duration_sec：按派发时刻计时（排队中取消等未派发场景为 0），
  // 计时后即遗忘时间戳，防止 map 长期驻留
  double duration_sec = 0.0;
  auto dispatch_time = mission_dispatch_time_.find(mission_id);
  if (dispatch_time != mission_dispatch_time_.end()) {
    duration_sec = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - dispatch_time->second)
                       .count();
    mission_dispatch_time_.erase(dispatch_time);
  }
  if (!handle || !handle->is_active()) {
    return;
  }

  auto result = std::make_shared<ExecuteMission::Result>();
  result->success = (kind == ExecuteFinish::kSucceed);
  result->error_code = error_code;
  result->message = message;
  result->total_duration_sec = duration_sec;
  try {
    if (handle->is_canceling()) {
      // 已受理 action cancel：只能走 canceled()（rclcpp_action 状态机约束）
      handle->canceled(result);
    } else if (handle->is_executing()) {
      if (kind == ExecuteFinish::kSucceed) {
        handle->succeed(result);
      } else {
        // kAbort 与服务侧取消（未进入 CANCELING 无法 canceled()）降级为 abort
        handle->abort(result);
      }
    } else {
      // ACCEPTED 未执行的兜底：先 execute 再 abort，避免 rclcpp_action 状态断言
      handle->execute();
      handle->abort(result);
    }
    RCLCPP_INFO(get_logger(), "ExecuteMission goal finished: %s kind=%d (%s)",
                mission_id.c_str(), static_cast<int>(kind), message.c_str());
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "FinishExecuteGoal(%s) failed: %s", mission_id.c_str(), ex.what());
  }
}

void DispatcherNode::CleanupMissionCancel(const std::string& mission_id,
                                          const std::string& reason) {
  // HandleGoal 预占的 required_resources 随任务取消一并释放（plan 模式无 handle 时
  // Release 对无锁持有者是无操作，与既有行为一致）
  const bool has_handle = execute_goal_handles_.count(mission_id) > 0;
  if (has_handle && reservation_table_) {
    reservation_table_->Release(mission_id);
  }
  // action 模式级联：取消在途 NavigateSequence goal 并清 pending 标记
  if (navigation_backend_ == "action" && pending_nav_mission_id_ == mission_id) {
    if (nav_client_ && pending_nav_goal_handle_) {
      nav_client_->async_cancel_goal(pending_nav_goal_handle_);
    }
    pending_nav_mission_id_.clear();
    pending_nav_goal_handle_.reset();
  }
  if (has_handle) {
    // 服务侧取消未受理 action cancel（handle 不在 CANCELING），降级 abort(3 PREEMPTED)
    // （total_duration_sec 的时间戳由 FinishExecuteGoal 计时后遗忘）
    FinishExecuteGoal(mission_id, ExecuteFinish::kCanceled, reason, 3);
  } else {
    mission_dispatch_time_.erase(mission_id);
  }
  ForgetMissionDispatchState(mission_id);
}

void DispatcherNode::ForgetMissionDispatchState(const std::string& mission_id) {
  // 路由锁预留记账、排队等待记录与派发起点同步遗忘：
  // 幂等，供 Complete/Fail/取消/死锁取消/预占判负等终态路径共用
  mission_reserved_at_.erase(mission_id);
  pending_wait_.erase(mission_id);
  mission_dispatch_start_.erase(mission_id);
  mission_route_paths_.erase(mission_id);  // Task 5：行驶路径存档随终态遗忘
  vda5050_mission_vehicle_.erase(mission_id);  // Task 7：mission->车辆绑定随终态遗忘
}

void DispatcherNode::ProcessActionCancels() {
  if (execute_goal_handles_.empty()) {
    return;
  }
  std::vector<std::string> canceling;
  canceling.reserve(execute_goal_handles_.size());
  for (const auto& [id, handle] : execute_goal_handles_) {
    if (handle && handle->is_canceling()) {
      canceling.push_back(id);
    }
  }
  for (const auto& id : canceling) {
    RCLCPP_INFO(get_logger(), "ExecuteMission cancel accepted by action client: %s", id.c_str());
    bool canceled = false;
    if (queue_ && queue_->Cancel(id)) {
      // 排队中任务被 action 取消：连带释放 HandleGoal 预占的 required_resources
      if (reservation_table_) {
        reservation_table_->Release(id);
      }
      canceled = true;
    } else {
      auto it = active_missions_.find(id);
      if (it != active_missions_.end()) {
        if (reservation_table_) {
          reservation_table_->Release(id);
        }
        if (tracking_mission_id_ == id) {
          tracking_mission_id_.clear();
          PublishStopPathForTrackedVehicle();
        }
        active_missions_.erase(it);
        canceled = true;
      }
    }
    ForgetMissionDispatchState(id);  // 取消终态：路由锁记账/等待记录/派发起点遗忘
    // action 模式级联：取消在途的 NavigateSequence goal 并清 pending 标记
    if (navigation_backend_ == "action" && pending_nav_mission_id_ == id) {
      if (nav_client_ && pending_nav_goal_handle_) {
        nav_client_->async_cancel_goal(pending_nav_goal_handle_);
      }
      pending_nav_mission_id_.clear();
      pending_nav_goal_handle_.reset();
    }
    // 此时 handle 已确认 is_canceling()，canceled() 状态合法
    FinishExecuteGoal(id, ExecuteFinish::kCanceled, "mission canceled by action client", 3);
    if (canceled) {
      ReArmQueueStartRequest();
    }
  }
}

void DispatcherNode::NavGoalResponseCb(const std::string& mission_id,
                                       const GoalHandleNavigate::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "NavigateSequence goal rejected by bridge for mission %s",
                mission_id.c_str());
    if (pending_nav_mission_id_ == mission_id) {
      pending_nav_mission_id_.clear();
      pending_nav_goal_handle_.reset();
    }
    // FailMission 内部会清理 tracking 槽位与空路径停车
    FailMission(mission_id, "navigate_sequence goal rejected by bridge");
    return;
  }
  // goal 响应到达时任务可能已被取消/完结：孤儿 goal 直接取消，
  // 否则 bridge 盲跑 + pending 句柄残留（审查 I4）
  bool mission_alive;
  {
    std::lock_guard<std::mutex> lock(active_missions_mutex_);
    mission_alive = active_missions_.count(mission_id) > 0;
  }
  if (!mission_alive) {
    RCLCPP_WARN(get_logger(),
                "Stale NavigateSequence goal response for %s (mission gone); canceling orphan goal",
                mission_id.c_str());
    nav_client_->async_cancel_goal(goal_handle);
    if (pending_nav_mission_id_ == mission_id) {
      pending_nav_mission_id_.clear();
      pending_nav_goal_handle_.reset();
    }
    return;
  }
  pending_nav_goal_handle_ = goal_handle;
}

void DispatcherNode::NavFeedbackCb(
    const std::string& mission_id, const GoalHandleNavigate::SharedPtr& /*goal_handle*/,
    const std::shared_ptr<const NavigateSequence::Feedback> feedback) {
  mission_last_progress_[mission_id] = std::chrono::steady_clock::now();  // 喂死锁 idle 启发式
  if (!feedback) {
    return;
  }
  // NavigateSequence feedback -> ExecuteMission feedback 映射
  // （current_index/total -> state_code=2 EXECUTING + percent=index/total*100）
  auto hit = execute_goal_handles_.find(mission_id);
  if (hit == execute_goal_handles_.end() || !hit->second || !hit->second->is_executing()) {
    return;
  }
  auto fb = std::make_shared<ExecuteMission::Feedback>();
  fb->state_code = 2;  // EXECUTING
  fb->current_node = feedback->current_station;
  fb->percent_complete =
      feedback->total_waypoints > 0
          ? static_cast<float>(100.0 * static_cast<double>(feedback->current_index) /
                               static_cast<double>(feedback->total_waypoints))
          : 0.0f;
  fb->estimated_remaining_sec = 0.0;
  hit->second->publish_feedback(fb);
}

void DispatcherNode::NavResultCb(const std::string& mission_id,
                                 const GoalHandleNavigate::WrappedResult& result) {
  if (pending_nav_mission_id_ == mission_id) {
    pending_nav_mission_id_.clear();
    pending_nav_goal_handle_.reset();
  }
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
      result.result->success) {
    CompleteMission(mission_id, "navigate_sequence succeeded");
    return;
  }
  std::string msg;
  if (result.result && !result.result->message.empty()) {
    msg = result.result->message;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    msg = "navigate_sequence canceled";
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    msg = "navigate_sequence aborted";
  } else {
    msg = "navigate_sequence failed";
  }
  FailMission(mission_id, msg);
}

void DispatcherNode::ScheduleTick() {
  // 已受理的 ExecuteMission action 取消需在提前返回之前处理（10Hz 延迟 ≤100ms）
  ProcessActionCancels();

  // Task 5 Rule 2（Spec §5.2）：每 tick 先巡检受管小车链路。
  // 离线 -> 挂起新派发（不消费 queue_start_requested_，恢复后自动继续）、
  // 锁一律保留；在线但 state 陈旧 -> 仅软看门狗告警。受管清单为空时零开销跳过。
  if (EnforceVda5050LinkGuard()) {
    return;
  }

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

  // 地图版本门禁：配置了期望哈希且目录已加载时，
  // 若目录哈希缺失，或与期望哈希不一致，均判定为环境地图与部署目录不匹配——暂停出队派发。
  // 直接 return（不消费 queue_start_requested_），哈希恢复一致后派发自动继续。
  map_version_valid_ = true;
  if (!expected_map_hash_.empty() && catalog_reloader_ != nullptr &&
      catalog_reloader_->loaded()) {
    const auto& catalog = catalog_reloader_->catalog();
    if (catalog.map_hash.empty() || catalog.map_hash != expected_map_hash_) {
      map_version_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Map hash mismatch: expected %s, got %s, dispatching paused",
                           expected_map_hash_.c_str(),
                           catalog.map_hash.empty() ? "<missing>" : catalog.map_hash.c_str());
      return;
    }
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

  // Peek→Reserve→Pop（节点单线程，Peek 与 Pop 之间无竞态）：
  // 先为队首任务规划路由锁并预留，成功才出队继续派发；失败则任务留在队列、
  // 发 reservation_denied 事件、重新武装启动请求后本 tick 返回——既不丢任务也不违反封路。
  const auto* peeked = queue_->Peek();
  if (!peeked) {
    return;  // 队列任务全部暂停：与 PopNext 失败等价，本 tick 结束
  }
  const std::string candidate_id = peeked->id;
  const std::string candidate_order_id = peeked->order_id;
  std::vector<std::string> route_locks;
  if (catalog_reloader_ != nullptr && catalog_reloader_->loaded()) {
    route_locks = amr_dispatcher_core::workflow::PlanMissionRouteLocks(
        catalog_reloader_->catalog(), peeked->pickup_station, peeked->dropoff_station);
  }
  if (!route_locks.empty() && reservation_table_) {
    auto reserve = reservation_table_->Reserve(candidate_id, route_locks);
    if (!reserve.success) {
      pending_wait_[candidate_id] = route_locks;  // WFG：该任务在等这批锁
      RCLCPP_WARN(get_logger(), "Reservation denied for queued mission %s: %s",
                  candidate_id.c_str(), reserve.message.c_str());
      if (event_pub_ && event_pub_->is_activated()) {
        amr_dispatcher_interfaces::msg::MissionEvent msg;
        msg.stamp = now();
        msg.mission_id = candidate_id;
        msg.order_id = candidate_order_id;
        msg.event = "reservation_denied";
        msg.from_state = "PENDING";
        msg.to_state = "PENDING";  // 任务留在队列，状态不变
        msg.reason = reserve.message;
        event_pub_->publish(msg);
      }
      if (event_log_) {
        amr_dispatcher_core::dispatcher::MissionEvent evt;
        evt.stamp = "now";
        evt.mission_id = candidate_id;
        evt.event = "reservation_denied";
        evt.message = reserve.message;
        event_log_->Append(evt);
      }
      ReArmQueueStartRequest();  // 队列非空 -> 稍后锁释放时重试派发
      return;
    }
    mission_reserved_at_[candidate_id] = std::chrono::steady_clock::now();
  }
  pending_wait_.erase(candidate_id);  // 预留通过（或本轮无锁），历史等待记录作废

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
    // 本单已出队但被判负丢弃：Peek 阶段预留的路由锁须就地释放
    //（任务不进 active_missions_，走不到 Complete/Fail 的终态清理）
    if (reservation_table_) {
      reservation_table_->Release(m.id);
    }
    ForgetMissionDispatchState(m.id);
    // ExecuteMission goal 须就地 abort：该分支不经 Complete/Fail，不收口则
    // action 客户端永久悬挂、handle 残留在 execute_goal_handles_（终审 F8）
    //（mission_dispatch_time_ 此刻尚未写入，无需额外遗忘）
    FinishExecuteGoal(m.id, ExecuteFinish::kAbort,
                      "unknown dropoff station: " + m.dropoff_station, 1);
    ReArmQueueStartRequest();  // 本单丢弃后，队列中其余任务仍需能被派发
    return;
  }

  // 预留成功：持有锁列表（逗号连接）回填任务，供死锁检测长持锁启发式取证
  if (!route_locks.empty()) {
    std::string joined_locks;
    for (const auto& lock_id : route_locks) {
      if (!joined_locks.empty()) {
        joined_locks += ",";
      }
      joined_locks += lock_id;
    }
    m.reservation_id = joined_locks;
  }

  active_missions_[m.id] = m;
  mission_last_progress_[m.id] = std::chrono::steady_clock::now();

  // Task 5：存档本单行驶路径（travel order），供小车 state 逐段解锁回推身后锁。
  // Peek 阶段的 PlanMissionRouteLocks 只产出锁列表（BuildLockIds 顺序非行驶序），
  // 行驶序须在此按同一目录快照经 FindRoutePath 重算存档；无路径则该单不参与动态解锁。
  if (catalog_loaded && !m.pickup_station.empty()) {
    const auto route_path = amr_dispatcher_core::catalog::FindRoutePath(
        catalog_reloader_->catalog(), m.pickup_station, m.dropoff_station);
    if (route_path.has_value()) {
      mission_route_paths_[m.id] = *route_path;
    } else {
      RCLCPP_WARN(get_logger(),
                  "Mission %s: no topology path archived; dynamic unlock disabled for this mission",
                  m.id.c_str());
    }
  }

  // Task 6（Task 5 评审 I2）：受管 VDA 5050 车辆的订单发布——存档路径 + 本单持锁
  // 构建 VehicleOrder 并经 /fleet/{vid}/send_order 下发（网关转 MQTT order 话题）。
  // 失败不判负任务：plan/action 执行链照旧，订单缺口仅告警（发布端尽力而为）。
  PublishVda5050Order(m.id);

  // 生成并发布路径：path_tracker 订阅 /plan 后开始输出 /cmd_vel_raw
  amr_dispatcher_core::Pose2D start;
  if (from_st) {
    start = {from_st->x, from_st->y, from_st->yaw};
  } else if (has_odom_) {
    start = {current_pose_.x, current_pose_.y, current_pose_.yaw};
  }
  const amr_dispatcher_core::Pose2D goal{to_st->x, to_st->y, to_st->yaw};

  const auto waypoints = planner_ ? planner_->Plan(start, goal) : std::vector<amr_dispatcher_core::Pose2D>{};
  const std::string dispatch_vehicle_id = ResolveDispatchVehicleId(m);
  const auto plan_pub = PlanPublisherForVehicle(dispatch_vehicle_id);
  if (waypoints.empty() || !plan_pub || !plan_pub->is_activated()) {
    FailMission(m.id, "planner returned empty path");
    return;
  }

  // 记录派发起点：ExecuteMission feedback 按此直线距离折算 percent_complete
  mission_dispatch_start_[m.id] = {start.x, start.y, start.theta};
  // 派发时刻：ExecuteMission result.total_duration_sec 由此计时
  mission_dispatch_time_[m.id] = std::chrono::steady_clock::now();

  // 执行后端：action 模式不发 /plan，改为向 /navigate_sequence 发单点 goal
  //（waypoint=规划终点，station_names=[dropoff]）；默认 plan 模式以下路径不变。
  if (navigation_backend_ == "action") {
    if (!nav_client_) {
      FailMission(m.id, "navigate_sequence client not configured");
      return;
    }
    if (!nav_client_->action_server_is_ready()) {
      FailMission(m.id, "navigate_sequence action server not available (is bridge running?)");
      return;
    }
    const auto& wp = waypoints.back();
    NavigateSequence::Goal nav_goal;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = "map";
    ps.pose.position.x = wp.x;
    ps.pose.position.y = wp.y;
    ps.pose.orientation.z = std::sin(wp.theta / 2.0);
    ps.pose.orientation.w = std::cos(wp.theta / 2.0);
    nav_goal.waypoints.push_back(ps);
    nav_goal.station_names.push_back(m.dropoff_station);
    nav_goal.loop = false;

    rclcpp_action::Client<NavigateSequence>::SendGoalOptions opts;
    const std::string mission_id = m.id;
    opts.goal_response_callback = [this, mission_id](const GoalHandleNavigate::SharedPtr& gh) {
      NavGoalResponseCb(mission_id, gh);
    };
    opts.feedback_callback = [this, mission_id](const GoalHandleNavigate::SharedPtr& gh,
                                                const std::shared_ptr<const NavigateSequence::Feedback> fb) {
      NavFeedbackCb(mission_id, gh, fb);
    };
    opts.result_callback = [this, mission_id](const GoalHandleNavigate::WrappedResult& result) {
      NavResultCb(mission_id, result);
    };
    // 先占住单车执行槽位再发 goal，避免 goal 响应回调前的派发空窗
    pending_nav_mission_id_ = m.id;
    tracking_mission_id_ = m.id;
    nav_client_->async_send_goal(nav_goal, opts);

    RCLCPP_INFO(get_logger(),
                "Dispatched mission via NavigateSequence: %s (order=%s, type=%s, prio=%d, %s->%s)",
                m.id.c_str(), m.order_id.c_str(), m.type.c_str(), m.priority,
                m.pickup_station.c_str(), m.dropoff_station.c_str());

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
  PublishPlanForVehicle(dispatch_vehicle_id, path_msg);
  tracking_mission_id_ = m.id;
  tracking_vehicle_id_ = dispatch_vehicle_id;
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
  // action 模式的到达信号归 bridge 消费、结果由 NavResultCb 收口：
  // dispatcher 若自行 CompleteMission 会对同一 NavigateSequence goal 发
  // async_cancel_goal，把已完成 goal 打成 CANCELED（审查 I2）
  if (navigation_backend_ == "action") {
    return;
  }
  // plan 模式下必须按周期喂活 progress：path_tracker_node 以 control_rate_hz
  // 持续发布 /tracking/error，若仅 arrived=true 才刷新时间戳，长距离巡航
  // (>10s) 会触发 DeadlockDetector 的 active_idle_threshold 误判并取消任务
  if (!tracking_mission_id_.empty()) {
    mission_last_progress_[tracking_mission_id_] = std::chrono::steady_clock::now();
  }
  if (!msg->arrived || tracking_mission_id_.empty()) {
    return;
  }
  const std::string finished_id = tracking_mission_id_;
  tracking_mission_id_.clear();
  mission_last_progress_[finished_id] = std::chrono::steady_clock::now();
  CompleteMission(finished_id, "arrived at goal");
}

void DispatcherNode::CompleteMission(const std::string& mission_id, const std::string& reason) {
  // 单车执行槽位兜底释放：plan 模式由 TrackingErrorCb 先清（幂等）、
  // action 模式由 NavResultCb 收口——此处不清则槽位残留，后续任务永远无法派发（审查 C1）
  if (tracking_mission_id_ == mission_id) {
    tracking_mission_id_.clear();
  }
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
  // ExecuteMission goal 成功收口；action 模式清在途导航 goal（result 到达前的兜底取消）
  FinishExecuteGoal(mission_id, ExecuteFinish::kSucceed, reason, 0);
  mission_dispatch_time_.erase(mission_id);  // plan 模式无 handle 时的派发时刻遗忘
  ForgetMissionDispatchState(mission_id);    // 路由锁记账/等待记录/派发起点遗忘
  if (navigation_backend_ == "action" && pending_nav_mission_id_ == mission_id) {
    if (nav_client_ && pending_nav_goal_handle_) {
      nav_client_->async_cancel_goal(pending_nav_goal_handle_);
    }
    pending_nav_mission_id_.clear();
    pending_nav_goal_handle_.reset();
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
  // ExecuteMission goal 失败收口（error_code=1）；action 模式取消在途导航 goal
  FinishExecuteGoal(mission_id, ExecuteFinish::kAbort, reason, 1);
  mission_dispatch_time_.erase(mission_id);  // plan 模式无 handle 时的派发时刻遗忘
  ForgetMissionDispatchState(mission_id);    // 路由锁记账/等待记录/派发起点遗忘
  if (navigation_backend_ == "action" && pending_nav_mission_id_ == mission_id) {
    if (nav_client_ && pending_nav_goal_handle_) {
      nav_client_->async_cancel_goal(pending_nav_goal_handle_);
    }
    pending_nav_mission_id_.clear();
    pending_nav_goal_handle_.reset();
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
    PublishStopPathForTrackedVehicle();
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
        StampMissionIdentity(dock_m, next_sequence_);  // B4：id/sequence 同源
        dock_m.order_id = "dock_" + mission_id;
        dock_m.type = "charge";
        dock_m.dropoff_station = recovery_dock_station_;
        dock_m.priority = mission_copy.priority;
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

amr_dispatcher_core::dispatcher::DeadlockSnapshot DispatcherNode::BuildDeadlockSnapshot() {
  amr_dispatcher_core::dispatcher::DeadlockSnapshot snap;
  // 等待记录只对仍在排队的任务有效：被批量取消/离场的等待者不进 waiting_for
  //（防 WFG 残留幽灵等待者；派发成功/终态路径已即时遗忘，此处兜底清扫）
  if (queue_) {
    for (auto wait = pending_wait_.begin(); wait != pending_wait_.end();) {
      if (queue_->Contains(wait->first)) {
        ++wait;
      } else {
        wait = pending_wait_.erase(wait);
      }
    }
  }
  for (const auto& [id, m] : active_missions_) {
    snap.active_missions.push_back(m);
    snap.last_progress[id] = mission_last_progress_[id];
  }
  // WFG 数据上环：held_by 与预留资源集合来自预约表快照（resource -> owner），
  // waiting_for 来自 Peek 预留失败的排队任务，长持锁取证时刻来自任务预留记账
  //（人工 ReserveResourceCb 不记，避免长持锁误报）
  if (reservation_table_) {
    for (const auto& [resource_id, owner_id] : reservation_table_->Snapshot()) {
      snap.allocation_graph.held_by[resource_id] = owner_id;
      snap.reserved_resource_ids.insert(resource_id);
    }
  }
  snap.allocation_graph.waiting_for = pending_wait_;
  snap.reservation_acquired_at = mission_reserved_at_;
  return snap;
}

void DispatcherNode::DeadlockTick() {
  if (!deadlock_detector_ || !recovery_policy_) {
    return;
  }

  const auto snap = BuildDeadlockSnapshot();

  const auto findings = deadlock_detector_->Detect(snap);
  deadlock_flag_ = !findings.empty();

  // 拓扑状态投影的死锁输入：存各 finding 的环路径，无死锁时清空
  last_deadlock_findings_.clear();
  for (const auto& finding : findings) {
    if (!finding.cycle_path.empty()) {
      last_deadlock_findings_.push_back(finding.cycle_path);
    }
  }

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
        // 死锁恢复取消的级联：清在途 NavigateSequence goal + 终结 ExecuteMission goal
        if (navigation_backend_ == "action" && pending_nav_mission_id_ == act.mission_id) {
          if (nav_client_ && pending_nav_goal_handle_) {
            nav_client_->async_cancel_goal(pending_nav_goal_handle_);
          }
          pending_nav_mission_id_.clear();
          pending_nav_goal_handle_.reset();
        }
        FinishExecuteGoal(act.mission_id, ExecuteFinish::kAbort, act.reason, 1);
        mission_dispatch_time_.erase(act.mission_id);  // plan 模式无 handle 时的派发时刻遗忘
        ForgetMissionDispatchState(act.mission_id);    // 路由锁记账/等待记录/派发起点遗忘
        // 死锁恢复取消到正在跟踪 /plan 的任务时，必须释放跟踪槽位并停车，
        // 否则 tracking_mission_id_ 悬挂导致后续任务永远无法下发。
        if (tracking_mission_id_ == act.mission_id) {
          tracking_mission_id_.clear();
          PublishStopPathForTrackedVehicle();
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
        // B8：写 map 前先确认任务仍在册（active 或排队中）——
        // 死锁恢复决策基于快照延迟生成，目标可能已终态/被操作员移除，
        // 盲区写 map 会把幽灵 mission_id 驻留在 attempts/last_progress 里。
        const bool mission_known =
            active_missions_.count(act.mission_id) > 0 ||
            (queue_ && queue_->Contains(act.mission_id));
        if (mission_known) {
          mission_attempts_[act.mission_id]++;
          mission_last_progress_[act.mission_id] = std::chrono::steady_clock::now();
        } else {
          RCLCPP_INFO(get_logger(),
                      "Deadlock kRetry skipped for unknown mission %s (no longer active/queued)",
                      act.mission_id.c_str());
        }
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

// ---- Task 5：VDA 5050 逐段解锁与断链守卫（Spec §5.2/§5.3）----

// ---- P0.6 / A4：per-vehicle 话题与 TF 隔离 ----

void DispatcherNode::SetupVehicleTopics(const std::string& vehicles_param) {
  vehicle_ids_.clear();
  vehicle_plan_pubs_.clear();
  vehicle_odom_subs_.clear();
  vehicle_tracking_error_subs_.clear();
  vehicle_poses_.clear();

  for (const auto& vehicle_id : SplitVehicleIds(vehicles_param)) {
    if (!IsValidVehicleId(vehicle_id)) {
      RCLCPP_WARN(get_logger(),
                  "P0.6: invalid vehicle id '%s' (must match [A-Za-z_][A-Za-z0-9_]*) — skipped",
                  vehicle_id.c_str());
      continue;
    }
    vehicle_ids_.push_back(vehicle_id);
    // /plan/{vid} 发布端：每车一份，避免多车互相覆盖路径。
    vehicle_plan_pubs_[vehicle_id] =
        create_publisher<nav_msgs::msg::Path>(ResolveVehicleTopic("", "/plan", vehicle_id), 1);
    // /odom/{vid} 订阅：只更新 vehicle_poses_，不动 legacy 的 current_pose_/has_odom_。
    vehicle_odom_subs_[vehicle_id] = create_subscription<nav_msgs::msg::Odometry>(
        ResolveVehicleTopic("", "/odom", vehicle_id), 10,
        [this, vehicle_id](const nav_msgs::msg::Odometry::SharedPtr msg) {
          OdomCbForVehicle(vehicle_id, msg);
        });
    // /tracking/error/{vid} 订阅：到达信号按车隔离（A5 收口点）。
    vehicle_tracking_error_subs_[vehicle_id] =
        create_subscription<amr_dispatcher_interfaces::msg::TrackingError>(
            ResolveVehicleTopic("", "/tracking/error", vehicle_id), 10,
            [this, vehicle_id](const amr_dispatcher_interfaces::msg::TrackingError::SharedPtr msg) {
              TrackingErrorCbForVehicle(vehicle_id, msg);
            });
  }
  if (vehicle_ids_.empty()) {
    return;  // 单车兼容模式：零行为变化
  }
  std::string joined;
  for (const auto& id : vehicle_ids_) {
    if (!joined.empty()) joined += ", ";
    joined += id;
  }
  RCLCPP_INFO(get_logger(),
              "P0.6: per-vehicle topics enabled for {%s} (/plan/{vid}, /odom/{vid}, "
              "/tracking/error/{vid})",
              joined.c_str());
}

rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
DispatcherNode::PlanPublisherForVehicle(const std::string& vehicle_id) {
  if (!vehicle_id.empty()) {
    const auto it = vehicle_plan_pubs_.find(vehicle_id);
    if (it != vehicle_plan_pubs_.end() && it->second) {
      return it->second;
    }
  }
  return plan_pub_;  // legacy /plan（单车兼容红线）
}

void DispatcherNode::PublishPlanForVehicle(const std::string& vehicle_id,
                                           const nav_msgs::msg::Path& path) {
  const auto pub = PlanPublisherForVehicle(vehicle_id);
  if (pub && pub->is_activated()) {
    pub->publish(path);
  }
}

void DispatcherNode::OdomCbForVehicle(const std::string& vehicle_id,
                                      const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto& slot = vehicle_poses_[vehicle_id];
  slot.pose.x = msg->pose.pose.position.x;
  slot.pose.y = msg->pose.pose.position.y;
  slot.pose.yaw = amr_dispatcher_core::path_tracking::YawFromQuaternion(
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  slot.has_odom = true;
}

void DispatcherNode::TrackingErrorCbForVehicle(
    const std::string& vehicle_id, const amr_dispatcher_interfaces::msg::TrackingError::SharedPtr msg) {
  // A5 收口点：把 arrived 信号路由到"该车正在执行的任务"并完成它。
  // A4 阶段仅记录事件，避免在多车槽位拆分落地前产生错误的完成语义。
  if (!msg->arrived) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "P0.6: vehicle %s reported arrived (mission completion routing is Task A5)",
              vehicle_id.c_str());
}

std::string DispatcherNode::ResolveDispatchVehicleId(
    const amr_dispatcher_core::dispatcher::Mission& mission) {
  // A4 阶段只认"任务显式指定且在本调度器清单内"的车辆；
  // 自动选车属 A5/A6（core/fleet/vehicle_assignment.hpp）。
  if (mission.assigned_vehicle_id.empty()) {
    return "";
  }
  for (const auto& id : vehicle_ids_) {
    if (id == mission.assigned_vehicle_id) {
      return id;
    }
  }
  RCLCPP_WARN(get_logger(),
              "Mission %s requests vehicle '%s' which is not in vehicle_ids — falling back "
              "to the legacy single-vehicle plan channel",
              mission.id.c_str(), mission.assigned_vehicle_id.c_str());
  return "";
}

void DispatcherNode::PublishStopPathForTrackedVehicle() {
  nav_msgs::msg::Path empty;
  empty.header.stamp = now();
  empty.header.frame_id = "map";
  PublishPlanForVehicle(tracking_vehicle_id_, empty);
  tracking_vehicle_id_.clear();
}

void DispatcherNode::SetupVda5050Links(const std::string& vehicles_param) {
  // 终审 M4：reconfigure 必须先无条件清旧链路态——否则空化 vda5050_vehicle_ids
  // 会残留陈旧 link 继续喂 EnforceVda5050LinkGuard 的挂起守卫（离线车永远
  // 封锁全队派发，且无人工解除途径）。
  vda5050_links_.clear();
  vda5050_vehicle_ids_.clear();
  vda5050_order_pubs_.clear();
  vda5050_next_order_update_id_.clear();
  vda5050_mission_vehicle_.clear();
  vda5050_quarantined_.clear();

  vda5050_state_stale_sec_ = get_parameter("vda5050_state_stale_sec").as_double();
  // 订阅策略：ROS 2 无通配订阅，网关按 MQTT 发现的小车懒建 /fleet/{vid}/* 话题。
  // 采用参数显式清单（逗号分隔 id）——确定性、参数为空时零行为变化，
  // 且订阅先于网关懒建话题也能收到后续消息（ROS 2 订阅者不依赖话题已存在）。
  std::vector<std::string> vehicle_ids;
  std::size_t start = 0;
  while (start <= vehicles_param.size()) {
    const auto end = vehicles_param.find(',', start);
    std::string token = vehicles_param.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
    const auto first = token.find_first_not_of(" \t");
    const auto last = token.find_last_not_of(" \t");
    if (first != std::string::npos) {
      token = token.substr(first, last - first + 1);
      if (std::find(vehicle_ids.begin(), vehicle_ids.end(), token) == vehicle_ids.end()) {
        vehicle_ids.push_back(token);
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  if (vehicle_ids.empty()) {
    return;  // 未配置受管小车：逐段解锁/断链守卫整体关闭（现网行为不变）
  }
  for (const auto& vehicle_id : vehicle_ids) {
    auto& link = vda5050_links_[vehicle_id];
    link.state_sub = create_subscription<std_msgs::msg::String>(
        "/fleet/" + vehicle_id + "/state", rclcpp::QoS(10),
        [this, vehicle_id](const std_msgs::msg::String::SharedPtr msg) {
          Vda5050StateCb(vehicle_id, msg);
        });
    // 与网关 online 发布端 QoS 对齐（KeepLast(1) + transient_local），
    // 保证 late-join 的调度端也能收到最近一次链路状态
    link.online_sub = create_subscription<std_msgs::msg::Bool>(
        "/fleet/" + vehicle_id + "/online",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        [this, vehicle_id](const std_msgs::msg::Bool::SharedPtr msg) {
          Vda5050OnlineCb(vehicle_id, msg);
        });
    // Task 6：订单发布端按清单一次建齐（on_activate 激活）。网关的
    // send_order 订阅虽是发现车辆后懒建，ROS 2 发布端无需对端先存在；
    // QoS(10) reliable 与网关订阅侧 create_subscription(..., 10, ...) 对齐。
    vda5050_order_pubs_[vehicle_id] =
        create_publisher<std_msgs::msg::String>("/fleet/" + vehicle_id + "/send_order", 10);
    vda5050_next_order_update_id_[vehicle_id] = 0;
  }
  vda5050_vehicle_ids_ = vehicle_ids;  // 保序清单：round-robin 绑定与遍历用
  std::string joined;
  for (const auto& id : vehicle_ids) {
    if (!joined.empty()) joined += ", ";
    joined += id;
  }
  RCLCPP_INFO(get_logger(),
              "Task 5: subscribed /fleet/{%s}/{state,online} for dynamic unlock "
              "(stale threshold %.1fs)",
              joined.c_str(), vda5050_state_stale_sec_);
}

void DispatcherNode::PublishVda5050Order(const std::string& mission_id) {
  if (vda5050_vehicle_ids_.empty()) {
    return;  // 未配置受管 VDA 5050 车辆：无订单发布（现网行为不变）
  }
  if (catalog_reloader_ == nullptr || !catalog_reloader_->loaded()) {
    RCLCPP_WARN(get_logger(),
                "Mission %s: station catalog not loaded, VDA 5050 order NOT published",
                mission_id.c_str());
    return;
  }
  const auto path_it = mission_route_paths_.find(mission_id);
  if (path_it == mission_route_paths_.end() || path_it->second.empty()) {
    RCLCPP_WARN(get_logger(),
                "Mission %s: no archived travel path, VDA 5050 order NOT published",
                mission_id.c_str());
    return;
  }
  if (!reservation_table_) {
    RCLCPP_WARN(get_logger(),
                "Mission %s: reservation table unavailable, VDA 5050 order NOT published",
                mission_id.c_str());
    return;
  }
  // 本单名下实际持有的锁（builder 契约：严禁传全局快照——否则他人锁会被
  // 误判为已放行，向小车下发未获路权的 released=true 边，违反 Spec §5.1）
  std::unordered_set<std::string> held_locks;
  for (const auto& [resource_id, owner] : reservation_table_->Snapshot()) {
    if (owner == mission_id) {
      held_locks.insert(resource_id);
    }
  }
  // 车辆绑定：保序清单上的 round-robin（简单策略，可换 first-free 而不影响契约；
  // 单车配置下结果恒定）。游标先取后进，跨单轮转。
  const std::size_t idx = vda5050_rr_cursor_ % vda5050_vehicle_ids_.size();
  vda5050_rr_cursor_ = (vda5050_rr_cursor_ + 1) % vda5050_vehicle_ids_.size();
  const std::string vehicle_id = vda5050_vehicle_ids_[idx];
  const std::uint32_t update_id = vda5050_next_order_update_id_[vehicle_id]++;
  const auto& catalog = catalog_reloader_->catalog();

  const auto order = amr_dispatcher_core::fleet::BuildVda5050OrderFromPath(
      vehicle_id, mission_id, update_id, path_it->second, catalog, held_locks);
  if (!order.has_value()) {
    RCLCPP_WARN(get_logger(),
                "Mission %s -> vehicle %s: BuildVda5050OrderFromPath failed "
                "(path %zu node(s)), VDA 5050 order NOT published",
                mission_id.c_str(), vehicle_id.c_str(), path_it->second.size());
    return;
  }
  const auto pub_it = vda5050_order_pubs_.find(vehicle_id);
  if (pub_it == vda5050_order_pubs_.end() || !pub_it->second ||
      !pub_it->second->is_activated()) {
    RCLCPP_WARN(get_logger(),
                "Mission %s -> vehicle %s: send_order publisher missing/inactive, "
                "VDA 5050 order NOT published",
                mission_id.c_str(), vehicle_id.c_str());
    return;
  }
  std_msgs::msg::String out;
  out.data = amr_dispatcher_core::fleet::SerializeVda5050Order(*order);
  pub_it->second->publish(out);
  // 登记 mission -> 车辆绑定：人工强清离线链路时据此反查"该车名下"的锁归属
  vda5050_mission_vehicle_[mission_id] = vehicle_id;
  RCLCPP_INFO(get_logger(),
              "VDA 5050 order published: mission %s -> /fleet/%s/send_order "
              "(orderId=%s, orderUpdateId=%u, %zu nodes, %zu edges, %zu bytes)",
              mission_id.c_str(), vehicle_id.c_str(), order->order_id.c_str(),
              update_id, order->nodes.size(), order->edges.size(), out.data.size());
}

void DispatcherNode::Vda5050StateCb(const std::string& vehicle_id,
                                    const std_msgs::msg::String::SharedPtr msg) {
  const auto report = amr_dispatcher_core::fleet::DeserializeVda5050State(msg->data);
  if (!report.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Malformed VDA 5050 state JSON from vehicle %s: %.128s",
                         vehicle_id.c_str(), msg->data.c_str());
    return;
  }
  // M1：状态报文必须与订阅话题的车辆身份一致，防止车 B 冒用车 A 的 orderId
  // 驱动任务 A 的解锁流程。
  if (report->vehicle_id != vehicle_id) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "VDA 5050 state identity mismatch: topic vehicle %s but "
                         "serialNumber %s — dropped",
                         vehicle_id.c_str(), report->vehicle_id.c_str());
    return;
  }
  auto link_it = vda5050_links_.find(vehicle_id);
  if (link_it != vda5050_links_.end()) {
    link_it->second.seen_state = true;
    link_it->second.last_state_time = std::chrono::steady_clock::now();
    if (!link_it->second.online) {
      // 离线期间收到 state 绝不可回置 online=true：DDS 跨话题乱序可能把断链前
      // 的最后一帧 state 排在 online=false 之后送达，若据此回置，链路会"永久
      // 在线"并绕过 Spec §5.2 的调度挂起（e2e Stage 6.4 曾复现）。
      // 链路存活的唯一权威判据是网关在 MQTT 有序流中确认断链之后、收到重连
      // 后的新 state 或 connection=ONLINE 时重发的 online=true（网关侧见
      // vda5050_gateway_node HandleState/HandleConnection）。此处仅告警：
      // 锁保留、调度挂起状态均不改变；锁的解除仍由下方 Rule 1 的 lastNodeId
      // 与订单一致性校验接管（不一致则全保留，Spec §5.2 位置重核对）。
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "VDA 5050 vehicle %s: state received while offline (locks "
                           "retained, dispatch suspended) — awaiting gateway online=true "
                           "re-publication after reconnect (Spec 5.2)",
                           vehicle_id.c_str());
    }
  }
  if (report->order_id.empty() || !reservation_table_) {
    return;
  }
  // 订单 -> 在途任务：网关订单 order_id = mission_id（BuildVda5050OrderFromPath），
  // 兼容按 Mission.order_id 业务号匹配
  auto mission_it = active_missions_.find(report->order_id);
  if (mission_it == active_missions_.end()) {
    for (auto it = active_missions_.begin(); it != active_missions_.end(); ++it) {
      if (it->second.order_id == report->order_id) {
        mission_it = it;
        break;
      }
    }
  }
  if (mission_it == active_missions_.end()) {
    return;  // 非本调度器在途任务的 state（重复/迟到/他人单）：幂等忽略，不碰锁
  }
  const auto path_it = mission_route_paths_.find(mission_it->first);
  if (path_it == mission_route_paths_.end() || path_it->second.empty()) {
    return;  // 该单无行驶路径存档（无 pickup 等）：不参与动态解锁
  }
  // Rule 1（Spec §5.3）：按确认位置回推身后锁，仅释放本任务持有且已整段越过的部分
  std::unordered_map<std::string, std::string> held_locks;
  for (const auto& [resource_id, owner] : reservation_table_->Snapshot()) {
    held_locks[resource_id] = owner;
  }
  const auto decision = amr_dispatcher_core::workflow::PlanTrailingUnlock(
      path_it->second, report->last_node_id, report->last_node_sequence_id,
      mission_it->first, held_locks);
  if (!decision.confirmed) {
    RCLCPP_WARN(get_logger(), "Dynamic unlock skipped (vehicle %s, order %s): %s",
                vehicle_id.c_str(), report->order_id.c_str(), decision.message.c_str());
    return;
  }
  if (decision.lock_ids_to_release.empty()) {
    return;  // 幂等：无新增可释放（已释放/足迹/前方锁）
  }
  const auto released = reservation_table_->ReleaseResources(mission_it->first,
                                                             decision.lock_ids_to_release);
  if (released.empty()) {
    return;
  }
  std::string joined;
  for (const auto& lock_id : released) {
    if (!joined.empty()) joined += ",";
    joined += lock_id;
  }
  RCLCPP_INFO(get_logger(),
              "Dynamic unlock (Spec 5.3): vehicle %s at %s -> mission %s released %zu lock(s): %s",
              vehicle_id.c_str(), report->last_node_id.c_str(), mission_it->first.c_str(),
              released.size(), joined.c_str());
  if (event_log_) {
    amr_dispatcher_core::dispatcher::MissionEvent evt;
    evt.stamp = "now";
    evt.mission_id = mission_it->first;
    evt.event = "dynamic_unlock";
    evt.message = "vehicle " + vehicle_id + " at " + report->last_node_id + " released " + joined;
    event_log_->Append(evt);
  }
  ReArmQueueStartRequest();  // 唤醒排队等待这批锁的后车重试派发
}

void DispatcherNode::Vda5050OnlineCb(const std::string& vehicle_id,
                                     const std_msgs::msg::Bool::SharedPtr msg) {
  auto it = vda5050_links_.find(vehicle_id);
  if (it == vda5050_links_.end()) {
    return;
  }
  if (it->second.online == msg->data) {
    return;  // 状态未变（transient_local 重放/重复报文）：不重复告警
  }
  it->second.online = msg->data;
  if (!msg->data) {
    // Rule 2（Spec §5.2 断链锁定保护）：仅标记离线 + 软看门狗告警，绝不释放任何锁。
    // 锁保留直至该车重连后 state 重核对坐标，或操作员人工强制解除。
    const auto guard = amr_dispatcher_core::workflow::EvaluateVehicleLinkGuard(
        /*vehicle_online=*/false, /*state_stale=*/false);
    RCLCPP_WARN(get_logger(), "VDA 5050 vehicle %s link broken: %s", vehicle_id.c_str(),
                guard.message.c_str());
    if (event_log_) {
      amr_dispatcher_core::dispatcher::MissionEvent evt;
      evt.stamp = "now";
      evt.mission_id = "-";  // 无任务级事件：链路级看门狗告警
      evt.event = "vehicle_offline";
      evt.message = "vehicle " + vehicle_id + ": retaining all route locks pending "
                    "reconnection and position re-verification (Spec 5.2)";
      event_log_->Append(evt);
    }
  } else {
    RCLCPP_INFO(get_logger(),
                "VDA 5050 vehicle %s back online: locks retained until state re-confirms "
                "lastNodeId against the order (Spec 5.2 position re-verification)",
                vehicle_id.c_str());
  }
}

bool DispatcherNode::EnforceVda5050LinkGuard() {
  if (vda5050_links_.empty()) {
    return false;  // 未配置受管小车：关闭状态，零开销
  }
  bool blocked = false;
  const auto now = std::chrono::steady_clock::now();
  for (const auto& [vehicle_id, link] : vda5050_links_) {
    if (vda5050_quarantined_.count(vehicle_id) > 0) {
      // 隔离态车辆已被移出调度池：它的离线是**已知且已处置**的状态，
      // 不再挂起全队派发（否则人工强清后车队仍然冻结，隔离就失去意义）。
      // 恢复需显式调用 RestoreVda5050Link（并核实物理位置）。
      continue;
    }
    if (!link.online) {
      const auto guard = amr_dispatcher_core::workflow::EvaluateVehicleLinkGuard(
          /*vehicle_online=*/false, /*state_stale=*/false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Dispatch suspended: %s (vehicle %s). On recovery the vehicle's "
                           "position is re-verified via state before any lock is released.",
                           guard.message.c_str(), vehicle_id.c_str());
      blocked = true;  // 任一受管小车离线即挂起新派发（单车 /plan 执行模型）
      continue;
    }
    if (link.seen_state) {
      const double age_sec =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - link.last_state_time)
              .count() /
          1000.0;
      if (age_sec > vda5050_state_stale_sec_) {
        // 在线但 state 陈旧：仅告警——陈旧不是释放依据，也不阻塞派发（Spec §5.3）
        const auto guard = amr_dispatcher_core::workflow::EvaluateVehicleLinkGuard(
            /*vehicle_online=*/true, /*state_stale=*/true);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "VDA 5050 vehicle %s: %s",
                             vehicle_id.c_str(), guard.message.c_str());
      }
    }
  }
  return blocked;
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

  // 受管车辆链路与调度状态（结构化消息，评审 P0.4）：
  // 供 Web 控制台展示与多车可观测性；隔离态显式暴露。
  const auto link_now = std::chrono::steady_clock::now();
  for (const auto& vehicle_id : vda5050_vehicle_ids_) {
    const auto link_it = vda5050_links_.find(vehicle_id);
    if (link_it == vda5050_links_.end()) {
      continue;
    }
    const auto& link = link_it->second;
    amr_dispatcher_interfaces::msg::VehicleLinkState vlink;
    vlink.vehicle_id = vehicle_id;
    vlink.backend = amr_dispatcher_core::fleet::VehicleBackendLabel(
        amr_dispatcher_core::fleet::VehicleExecutionBackend::kVda5050);
    vlink.online = link.online;
    vlink.busy = false;
    vlink.quarantined = vda5050_quarantined_.count(vehicle_id) > 0;
    vlink.position_valid = link.seen_state;
    vlink.battery_percent = 100.0;
    vlink.state_age_ms = link.seen_state
                             ? std::chrono::duration_cast<std::chrono::milliseconds>(
                                   link_now - link.last_state_time)
                                   .count()
                             : -1;
    msg.vehicle_links.push_back(vlink);
  }

  // ExecuteMission 进度反馈：按派发起点的直线距离折算 percent_complete
  //（state_code=1 WAITING_LOCK 为计划定死的周期反馈语义；导航中的高频反馈由 NavFeedbackCb 发 state_code=2）
  for (const auto& [id, handle] : execute_goal_handles_) {
    if (!handle || !handle->is_executing()) {
      continue;
    }
    auto ait = active_missions_.find(id);
    if (ait == active_missions_.end()) {
      continue;  // 任务已终态（finish 时会查表移除 handle）
    }
    double percent = 0.0;
    auto sit = mission_dispatch_start_.find(id);
    const bool catalog_loaded = catalog_reloader_ != nullptr && catalog_reloader_->loaded();
    if (sit != mission_dispatch_start_.end() && has_odom_ && catalog_loaded &&
        !ait->second.dropoff_station.empty()) {
      const auto* to_st = amr_dispatcher_core::catalog::FindStation(
          catalog_reloader_->catalog(), ait->second.dropoff_station);
      if (to_st) {
        const double total = std::hypot(to_st->x - sit->second.x, to_st->y - sit->second.y);
        const double covered =
            std::hypot(current_pose_.x - sit->second.x, current_pose_.y - sit->second.y);
        if (total > 1e-6) {
          percent = std::min(100.0, 100.0 * covered / total);
        }
      }
    }
    auto fb = std::make_shared<ExecuteMission::Feedback>();
    fb->state_code = 1;  // WAITING_LOCK（计划定死的周期反馈状态）
    fb->current_node = ait->second.dropoff_station;
    fb->percent_complete = static_cast<float>(percent);
    fb->estimated_remaining_sec = 0.0;
    handle->publish_feedback(fb);
  }

  state_pub_->publish(msg);

  // /dispatcher/topology_state：safety_gate 与 visualizer 的死锁/锁边输入
  //（快照全量透传，active 仅任务持有的 route_edge，deadlocked_nodes 解析成站点 id）
  if (topology_pub_ && topology_pub_->is_activated()) {
    std::unordered_set<std::string> active_ids;
    for (const auto& [id, _] : active_missions_) {
      active_ids.insert(id);
    }
    const std::vector<std::pair<std::string, std::string>> snapshot =
        reservation_table_ ? reservation_table_->Snapshot()
                           : std::vector<std::pair<std::string, std::string>>{};
    const auto topology = amr_dispatcher_core::dispatcher::BuildTopologyProjection(
        snapshot, active_ids, last_deadlock_findings_);
    amr_dispatcher_interfaces::msg::TopologyState topology_msg;
    topology_msg.stamp = now();
    topology_msg.active_edges = topology.active_edges;
    topology_msg.locked_edges = topology.locked_edges;
    topology_msg.edge_owners = topology.edge_owners;
    topology_msg.deadlocked_nodes = topology.deadlocked_nodes;
    topology_msg.deadlock_detected = topology.deadlock_detected;
    topology_pub_->publish(topology_msg);
  }
}

}  // namespace amr_dispatcher_ros

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_ros::DispatcherNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
