// 车队链路守卫的聚合决策（纯函数）。
//
// 现状问题：EnforceVda5050LinkGuard 把三条策略（隔离车跳过 / 任一在线车离线即
// 挂起全队 / stale 只告警）散在一个循环里，策略本身既没有被命名的类型，
// 也无法单测——想验证"隔离车不应阻断全队"必须起一个 ROS 图。
//
// 这里按"每台车一条判定 + 一条聚合规则"拆开：
//   * 隔离车         -> 跳过（不阻断、不告警：它的离线是**已知且已处置**的）
//   * 未隔离且离线   -> 阻断全队新派发 + 告警（Spec §5.2 铁律：锁一律保留）
//   * 在线但 state 陈旧 -> 只告警，不阻断（陈旧不是释放依据，也不是派发依据）
// 铁律不变：本决策**永不**产出释放动作（retain_all_locks 恒真）。
#pragma once

#include <string>
#include <vector>

namespace amr_dispatcher_core::workflow {

// 单台受管车的链路输入（ROS 侧从订阅状态拷贝出来）。
struct VehicleLinkInput {
  std::string vehicle_id;
  bool online = false;
  bool quarantined = false;
  bool seen_state = false;        // 是否收到过该车的 state 报文
  double state_age_sec = 0.0;     // 距最近一次 state 的秒数（seen_state=false 时无意义）
};

struct FleetLinkGuardRequest {
  std::vector<VehicleLinkInput> vehicles;
  double state_stale_sec = 5.0;
};

// 单台车的判定结果（用于日志逐车输出）。
struct VehicleLinkVerdict {
  enum class Kind {
    kSkip,           // 未受管/已隔离：不参与判定
    kSuspendDispatch,// 离线：挂起全队新派发
    kStaleWarning,   // 在线但 state 陈旧：仅告警
    kHealthy,        // 在线且新鲜：放行
  };
  Kind kind = Kind::kHealthy;
  std::string vehicle_id;
  std::string message;
};

struct FleetLinkGuardDecision {
  // 铁律：任何链路状态下守卫都不产生释放动作。
  bool retain_all_locks = true;
  bool block_new_dispatch = false;
  // 逐车判定（按输入顺序输出，便于日志与断言稳定）
  std::vector<VehicleLinkVerdict> verdicts;
};

FleetLinkGuardDecision EvaluateFleetLinkGuard(const FleetLinkGuardRequest& request);

}  // namespace amr_dispatcher_core::workflow
