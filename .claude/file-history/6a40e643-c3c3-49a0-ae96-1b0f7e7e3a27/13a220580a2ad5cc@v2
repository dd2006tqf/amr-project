// 工业物理设施联锁控制工作流（纯函数）。
//
// 对应 Phase 8 (P2)：
// 当车辆在执行任务经过路网时，若途经或到达与设施（自动门、电梯）关联的站点，
// 由本模块给出严谨的联锁决策：
// 1. PlanFacilityArrival：车辆接近/到达设施站点时，判定是否需要申请设施锁，并发出 "open" / "call" 指令；
// 2. PlanFacilityDeparture：车辆离开设施站点时，判定是否需要发出 "close" 指令并释放设施锁；
// 3. PlanFacilityDegradation：当等待超时（设施仍未开启/到位）时，判定安全降级动作（驻车等待或报警挂起）。
#pragma once

#include <string>
#include <vector>

#include "amr_dispatcher_core/facility/facility_adapter_interface.hpp"
#include "amr_dispatcher_core/facility/facility_catalog.hpp"
#include "amr_dispatcher_core/facility/facility_reservation.hpp"

namespace amr_dispatcher_core::facility {

struct FacilityApproachRequest {
  std::string mission_id;
  std::string vehicle_id;
  std::string current_station_id;      // 车辆当前/即将进入的站点
  FacilityStateReport facility_state;  // 设施实时状态汇报
  const FacilityResource* resource = nullptr;
  const ResourceReservation* current_reservation = nullptr;
  int64_t wait_duration_ms = 0;
  int64_t timeout_threshold_ms = 10000; // 默认 10 秒超时
};

struct FacilityApproachDecision {
  enum class Action {
    kProceed,               // 设施已就绪且已获锁，允许安全通行
    kAcquireAndOpen,        // 需申请锁并发送开启指令
    kWaitInPlace,           // 指令已发，设施正在动作中，在道口停车等待
    kTimeoutDegrade,        // 设施动作超时未就绪，安全挂起并告警
  };

  Action action = Action::kProceed;
  std::string command;      // "open", "call", etc.
  std::string command_param;
  std::string message;
};

struct FacilityDepartureRequest {
  std::string mission_id;
  std::string vehicle_id;
  std::string departed_station_id;     // 车辆刚刚驶离的站点
  const FacilityResource* resource = nullptr;
  const ResourceReservation* current_reservation = nullptr;
};

struct FacilityDepartureDecision {
  bool should_close = false;
  bool should_release_lock = false;
  std::string command = "close";
  std::string message;
};

FacilityApproachDecision PlanFacilityApproach(const FacilityApproachRequest& request);
FacilityDepartureDecision PlanFacilityDeparture(const FacilityDepartureRequest& request);

}  // namespace amr_dispatcher_core::facility
