// 派发门禁的聚合判定（纯函数）。
//
// ScheduleGatesPass 里有四个"能不能派发"的判据，原实现是一串顺序提前 return：
//
//   1. 队列为空            -> 本 tick 无事可做
//   2. 在途任务已达上限    -> 容量门禁
//   3. 没有空闲执行槽位    -> 每台受管车同一时刻只跑一条
//   4. 地图版本不匹配      -> 环境地图与部署目录不一致，暂停出队
//
// 这四条都属于"看几个标量就能判定"的决策，却因为埋在 ROS 回调里而无法单测
// （尤其是判据的**优先顺序**：容量已超时不应该再去报地图哈希不匹配）。
// 下沉后由宿主机 ctest 覆盖，ROS 侧只负责采集标量并按结果打日志/返回。
//
// 注意：链路守卫（EnforceVda5050LinkGuard）与行为树启动请求门控不在这里——
// 它们各有独立的纯函数（workflow/fleet_link_guard.hpp 与 amr_dispatcher_bt），
// 本函数只覆盖"无状态的门禁判据"。
#pragma once

#include <cstddef>
#include <string>

namespace amr_dispatcher_core::workflow {

struct DispatchGateRequest {
  bool queue_empty = true;
  std::size_t active_mission_count = 0;
  std::size_t max_active_missions = 0;
  bool has_free_vehicle_slot = false;
  // 地图版本门禁：expected 为空 = 未配置（不校验）；catalog_loaded=false = 目录未加载（不校验）
  std::string expected_map_hash;
  bool catalog_loaded = false;
  std::string catalog_map_hash;  // 空串 = 目录里没有哈希（视为不匹配）
};

struct DispatchGateDecision {
  enum class Kind {
    kPass,               // 放行，可以继续选车
    kQueueEmpty,         // 队列为空
    kCapacityExceeded,   // 在途任务已达 max_active_missions
    kNoFreeVehicleSlot,  // 无空闲执行槽位
    kMapVersionMismatch, // 地图哈希缺失或与期望不一致
  };
  Kind kind = Kind::kQueueEmpty;
  // map_version_valid 只在**确实走到**地图门禁时才更新：前序门禁提前返回时
  // 该字段为 false，调用方据此保留上一次的值（与切分前逐字等价）。
  bool map_gate_evaluated = false;
  bool map_version_valid = true;
  // 一句话原因（写进日志用；不含格式化参数，调用方自行拼 hash）
  std::string reason;
};

inline DispatchGateDecision EvaluateDispatchGates(const DispatchGateRequest& request) {
  DispatchGateDecision decision;
  if (request.queue_empty) {
    decision.kind = DispatchGateDecision::Kind::kQueueEmpty;
    decision.reason = "queue empty";
    return decision;
  }
  if (request.active_mission_count >= request.max_active_missions) {
    decision.kind = DispatchGateDecision::Kind::kCapacityExceeded;
    decision.reason = "active mission capacity reached";
    return decision;
  }
  if (!request.has_free_vehicle_slot) {
    decision.kind = DispatchGateDecision::Kind::kNoFreeVehicleSlot;
    decision.reason = "no free vehicle slot";
    return decision;
  }
  // 地图版本门禁：仅在"配置了期望哈希且目录已加载"时生效
  decision.map_gate_evaluated = true;
  if (!request.expected_map_hash.empty() && request.catalog_loaded) {
    if (request.catalog_map_hash.empty() ||
        request.catalog_map_hash != request.expected_map_hash) {
      decision.kind = DispatchGateDecision::Kind::kMapVersionMismatch;
      decision.map_version_valid = false;
      decision.reason = "map hash mismatch";
      return decision;
    }
  }
  decision.kind = DispatchGateDecision::Kind::kPass;
  decision.map_version_valid = true;
  return decision;
}

}  // namespace amr_dispatcher_core::workflow
