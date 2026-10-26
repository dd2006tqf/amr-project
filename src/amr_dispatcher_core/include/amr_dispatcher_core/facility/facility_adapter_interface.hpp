// 物理设施适配器抽象接口（Physical Facility Adapter）。
//
// 背景：调度系统在多层厂区或跨防火分区运行时，需要与现场物理设施联动
// （如自动门/风淋门、电梯/提升机、充电机等）。
// 过去系统中仅有内存级的状态和资源字典判定（SelectFacilityActionResource），
// 缺乏向下对接现场设施控制层（如 PLC、OPC-UA、Modbus 或专用电梯调度器）的适配接口。
//
// 本模块定义核心抽象接口 IFacilityAdapter 与通用数据结构：
// - FacilityType：设施类型（门、电梯、充电桩、未知）
// - FacilityStatus：设施状态（就绪、忙碌、故障、离线）
// - FacilityCommandResult：设施操作结果
// 并提供纯内存 / 测试用的 MockFacilityAdapter，保证：
// 1. 纯 C++、无 ROS / OPC-UA 强制运行时依赖，宿主单测 100% 可测。
// 2. 具备异步/状态机友好的查询与指令执行语义，为后续 OPC-UA 插件预留无缝扩展点。
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace amr_dispatcher_core::facility {

enum class FacilityType {
  kUnknown,
  kDoor,
  kElevator,
  kCharger,
};

enum class FacilityState {
  kOffline,
  kReady,
  kBusy,
  kFault,
};

struct FacilityCommandResult {
  bool success = false;
  std::string facility_id;
  std::string action;
  std::string message;
  int error_code = 0;
};

struct FacilityStateReport {
  std::string facility_id;
  FacilityType type = FacilityType::kUnknown;
  FacilityState state = FacilityState::kOffline;
  std::string current_floor;
  std::string door_status;  // "open", "closed", "opening", "closing"
  bool locked = false;
  std::string locked_by;
};

class IFacilityAdapter {
 public:
  virtual ~IFacilityAdapter() = default;

  // 基础元信息
  virtual std::string GetFacilityId() const = 0;
  virtual FacilityType GetFacilityType() const = 0;

  // 查询设施当前实时状态
  virtual FacilityStateReport QueryState() = 0;

  // 设施动作控制
  // 门操作：open, close, unlock
  // 电梯操作：call, send_to_floor, open_door, close_door
  virtual FacilityCommandResult ExecuteAction(const std::string& action,
                                              const std::string& params = "") = 0;

  // 预约锁与占用释放
  virtual bool AcquireLock(const std::string& holder_id) = 0;
  virtual bool ReleaseLock(const std::string& holder_id) = 0;
};

// 内存 Mock 适配器，用于单元测试与本地仿真
class MockFacilityAdapter : public IFacilityAdapter {
 public:
  MockFacilityAdapter(std::string facility_id, FacilityType type);

  std::string GetFacilityId() const override { return facility_id_; }
  FacilityType GetFacilityType() const override { return type_; }

  FacilityStateReport QueryState() override;
  FacilityCommandResult ExecuteAction(const std::string& action,
                                      const std::string& params = "") override;

  bool AcquireLock(const std::string& holder_id) override;
  bool ReleaseLock(const std::string& holder_id) override;

  // 仿真辅助注入接口
  void SetState(FacilityState state) { state_ = state; }
  void SetCurrentFloor(const std::string& floor) { current_floor_ = floor; }
  void SetDoorStatus(const std::string& status) { door_status_ = status; }

 private:
  std::string facility_id_;
  FacilityType type_;
  FacilityState state_{FacilityState::kReady};
  std::string current_floor_{"1"};
  std::string door_status_{"closed"};
  bool locked_{false};
  std::string locked_by_;
};

}  // namespace amr_dispatcher_core::facility
