// VDA 5050 订单的车辆绑定（纯函数）。
//
// 规则（三条，按优先级）：
//   1. 选车决策给出的承接车辆在受管清单内 -> 用它，游标不动
//   2. 承接车辆为空或不在清单内（配置改动后旧决策残留）-> 回退到
//      受管清单上的 round-robin
//   3. 清单为空 -> 没有可绑定的车（调用方据此不发布订单）
//
// 为什么单独抽出来：回退分支会**推进游标**，而游标是跨调用状态。
// 这个"什么时候推进、什么时候不推进"的差异直接决定多车分流是否均匀，
// 但原实现埋在 PublishVda5050Order 里，只能靠起 MQTT + 虚拟车间接验证。
// 抽成纯函数后（游标由调用方传入指针，函数本身不持有状态），可以单测
// 环绕、跳号与"显式绑定不推进游标"这几个属性。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace amr_dispatcher_core::fleet {

struct Vda5050VehicleBindingRequest {
  // 选车决策给出的承接车辆；空 = 没有显式指派
  std::string assigned_vehicle_id;
  // 保序受管车辆清单（round-robin 顺序必须确定性，不能用 unordered_map 遍历序）
  const std::vector<std::string>* managed_vehicle_ids = nullptr;
  // 进/出参数：round-robin 游标。nullptr = 调用方不关心推进
  std::size_t* round_robin_cursor = nullptr;
};

struct Vda5050VehicleBinding {
  // 绑定到的车辆；空 = 无可绑定车辆（不发布订单）
  std::string vehicle_id;
  // true = 采用显式指派；false = round-robin 回退（审计用）
  bool from_assignment = false;
};

inline bool IsManagedVda5050Vehicle(const std::vector<std::string>& managed_vehicle_ids,
                                    const std::string& vehicle_id) {
  if (vehicle_id.empty()) {
    return false;
  }
  for (const auto& candidate : managed_vehicle_ids) {
    if (candidate == vehicle_id) {
      return true;
    }
  }
  return false;
}

inline Vda5050VehicleBinding BindVda5050Vehicle(
    const Vda5050VehicleBindingRequest& request) {
  Vda5050VehicleBinding binding;
  if (request.managed_vehicle_ids == nullptr || request.managed_vehicle_ids->empty()) {
    return binding;  // 未配置受管车辆：无订单发布
  }
  const auto& managed = *request.managed_vehicle_ids;
  if (IsManagedVda5050Vehicle(managed, request.assigned_vehicle_id)) {
    binding.vehicle_id = request.assigned_vehicle_id;
    binding.from_assignment = true;
    return binding;  // 显式绑定不推进游标：否则回退序列会被悄悄跳号
  }
  // 游标可能大于当前清单长度（reconfigure 后清单缩短）：取模回绕
  const std::size_t cursor = request.round_robin_cursor == nullptr
                                 ? 0
                                 : *request.round_robin_cursor;
  const std::size_t idx = cursor % managed.size();
  binding.vehicle_id = managed[idx];
  binding.from_assignment = false;
  if (request.round_robin_cursor != nullptr) {
    *request.round_robin_cursor = (idx + 1) % managed.size();
  }
  return binding;
}

}  // namespace amr_dispatcher_core::fleet
