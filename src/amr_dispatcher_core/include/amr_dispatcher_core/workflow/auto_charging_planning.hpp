// 充电任务自动编排（纯函数）。
//
// 现状：core 里已有 `PlanOpportunityCharging`（按**电压**判定），但它只被单测
// 调用，调度器从不使用；调度器能看到的电量是**百分比**（VehicleRuntimeState /
// VDA 5050 batteryCharge），而且"该不该给这台车插一条充电单"还要看：车是否
// 空闲、在线、未隔离、是否已经有一张在跑/在排的充电单、充电站是否存在。
//
// 这里把这套判定独立成纯函数，供每个 tick 对每台车调用一次：
//   * 关闭（默认）-> 什么都不做
//   * 电量高于机会阈值 -> 不插单
//   * 该车已有充电单（排队或在途）-> 不重复插单（否则每 tick 插一条）
//   * 车忙 / 离线 / 隔离 -> 不插单（充电也要占执行槽位，车忙时插单只会堆积）
//   * 充电站未知 -> 不插单（不猜站点）
//   * 否则 -> 插一条 charge 单，临界电量用更高优先级
//
// 铁律：本函数**只产决策**，不碰队列、不碰日志。插单与去重记账由 ROS 侧落地。
#pragma once

#include <string>

namespace amr_dispatcher_core::workflow {

struct AutoChargingRequest {
  bool enabled = false;
  std::string vehicle_id;

  // 车辆运行时状态
  bool online = true;
  bool busy = false;
  bool quarantined = false;
  double battery_percent = 100.0;
  bool battery_known = true;

  // 该车当前是否已有充电单（排队或在途）—— 去重键是"车"，不是"站点"
  bool has_pending_charge_mission = false;

  // 充电目标站（由调用方解析后传入）
  std::string charger_station_id;
  bool charger_station_known = false;

  // 阈值与优先级
  double opportunity_threshold_percent = 30.0;
  double critical_threshold_percent = 15.0;
  int opportunity_priority = 0;
  int critical_priority = 5;
};

struct AutoChargingDecision {
  enum class Kind {
    kDisabled,        // 功能关闭
    kBatteryUnknown,  // 没有电量信息：不插单（不猜）
    kBatteryOk,       // 电量高于机会阈值
    kAlreadyQueued,   // 该车已有充电单
    kVehicleUnavailable,  // 忙 / 离线 / 隔离
    kChargerUnknown,  // 充电站未解析
    kQueueCharge,     // 插单
  };
  Kind kind = Kind::kDisabled;
  bool critical = false;
  int priority = 0;
  std::string charger_station_id;
  std::string reason;
};

AutoChargingDecision PlanAutoCharging(const AutoChargingRequest& request);

}  // namespace amr_dispatcher_core::workflow
