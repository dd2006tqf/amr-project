#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::fleet {

// 车队协议适配层的统一数据模型与控制接口。
// 调度核心只依赖本文件中的抽象，不感知具体厂商协议（VDA 5050 / 私有协议等）。

/// 单个车辆动作，挂载在节点或边上（VDA 5050 action）。
struct VehicleAction {
  std::string action_type;  // e.g. "lift", "drop", "charge", "beep"
  std::string action_id;
  std::unordered_map<std::string, std::string> parameters;
};

/// 车辆轨迹折线点（边内的几何路径点）。
struct VehicleTrajectoryPoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/// 订单中的一条边（含路权放行标记与行驶折线）。
struct VehicleEdge {
  std::string edge_id;
  std::string start_node_id;
  std::string end_node_id;
  bool released = true;
  double max_speed = 1.0;
  std::vector<VehicleTrajectoryPoint> polyline;
  std::vector<VehicleAction> actions;
};

/// 订单中的一个节点（含序号、路权放行标记与挂载动作）。
struct VehicleNode {
  std::string node_id;
  uint32_t sequence_id = 0;
  bool released = true;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::vector<VehicleAction> actions;
};

/// 一次完整的派车订单。
struct VehicleOrder {
  std::string order_id;
  uint32_t order_update_id = 0;
  std::string vehicle_id;
  std::vector<VehicleNode> nodes;
  std::vector<VehicleEdge> edges;
};

/// 车辆上报的归一化状态。
struct VehicleStateReport {
  std::string vehicle_id;
  std::string order_id;
  uint32_t last_node_sequence_id = 0;
  std::string last_node_id;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  // 定位是否已初始化（VDA 5050 agvPosition.positionInitialized）。
  // false -> 车辆位置不可信 -> 选车时该车**不合格**（不盲派）。
  bool agv_position_initialized = false;
  double battery_percent = 100.0;
  bool is_charging = false;
  bool estop_active = false;
  bool online = true;
  std::string operating_mode = "AUTOMATIC";
  std::vector<std::string> active_errors;
  std::chrono::steady_clock::time_point last_report_time = std::chrono::steady_clock::now();
};

/// 统一车队控制接口：调度核心通过它操作任意厂商车辆。
class IVehicleAdapter {
 public:
  virtual ~IVehicleAdapter() = default;

  virtual bool DispatchOrder(const std::string& vehicle_id, const VehicleOrder& order) = 0;
  virtual bool CancelOrder(const std::string& vehicle_id, const std::string& order_id) = 0;
  virtual bool PauseVehicle(const std::string& vehicle_id) = 0;
  virtual bool ResumeVehicle(const std::string& vehicle_id) = 0;
  virtual std::optional<VehicleStateReport> GetVehicleState(const std::string& vehicle_id) const = 0;
};

/// 内存测试桩：单测与脱机联调用的零 I/O 实现。
/// - states_: 注入/维护每台车的最新状态上报
/// - orders_: 每台车最近一次下发的订单（key 为 vehicle_id）
class MockVehicleAdapter : public IVehicleAdapter {
 public:
  bool DispatchOrder(const std::string& vehicle_id, const VehicleOrder& order) override;
  bool CancelOrder(const std::string& vehicle_id, const std::string& order_id) override;
  bool PauseVehicle(const std::string& vehicle_id) override;
  bool ResumeVehicle(const std::string& vehicle_id) override;
  std::optional<VehicleStateReport> GetVehicleState(const std::string& vehicle_id) const override;

  /// 单测注入车辆状态数据。
  void SetVehicleState(const VehicleStateReport& report);

  /// 查询最近一次下发到该车的订单（未下发过则返回 nullopt）。
  std::optional<VehicleOrder> GetDispatchedOrder(const std::string& vehicle_id) const;

 private:
  std::unordered_map<std::string, VehicleStateReport> states_;
  std::unordered_map<std::string, VehicleOrder> orders_;
};

}  // namespace amr_dispatcher_core::fleet
