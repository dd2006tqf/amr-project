#pragma once

#include <chrono>
#include <functional>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::workflow {

// 时空窗交通协调（Task B）：把"静态排他锁"升级为"带时间窗的预约"。
//
// 安全模型（评审要求）：
//   1. **逐资源区间**：一条路径涉及多个资源（边=通道、节点=路口），
//      每个资源单独计算进入/离开时刻。路口窗口 = 进入路口到驶出路口的完整区间。
//   2. **全有或全无**：整条路径的预约必须**全部成功**才提交；任一资源失败
//      则全部回滚，不留半套预约。
//   3. **窗口到期 ≠ 资源已释放**：物理资源仍需车辆确认驶离（lastNodeId 推进，
//      见 Spec §5.3 逐段解锁）才可让后车进入。延期靠 ExtendTimeSpace，
//      绝不靠"到点自动失效"。
//   4. **安全余量**：占用时长含加减速/转弯附加时间与车辆尺寸安全余量。
//
// 本模块只做**决策**（纯函数、零 ROS）；预约表的落地由 ResourceTable 承担。

// 单个资源的时间占用区间。
struct ResourceTimeWindow {
  std::string resource_id;   // route_edge:A__B 或 route_node:A
  std::chrono::steady_clock::time_point enter;
  std::chrono::steady_clock::time_point exit;
  // 该资源是"通道"（边）还是"路口"（节点）——路口的冲突判定更严格
  // （对向/交叉车辆的物理冲突发生在路口内部）。
  bool is_intersection = false;
};

// 路径占用区间集合（一个 mission 的全部资源）。
using ResourceTimeWindowSet = std::vector<ResourceTimeWindow>;

// 车辆运动参数（用于把几何长度换算成时间占用）。
struct MotionTimingConfig {
  // 巡航速度（m/s）。
  double cruise_speed_mps = 1.0;
  // 起步/刹停附加时间（秒）——每次进入资源都要付。
  double accel_overhead_sec = 1.0;
  // 转弯附加时间（秒）——按路径拐点数计。
  double turn_overhead_sec = 0.5;
  // 车辆尺寸安全余量（米）——沿路径方向额外占用长度。
  double safety_margin_m = 0.2;
  // 最小安全间隔（秒）：后车进入同一资源前必须与前车拉开该间隔。
  double min_headway_sec = 2.0;
};

// 单个候选任务的输入。
struct TimeSpaceCandidate {
  std::string mission_id;
  // 该单的行驶路径（节点序列，行驶序）。
  std::vector<std::string> path_node_ids;
  // 路径中每段的折线长度（米），与 path_node_ids 的相邻对一一对应：
  // lengths[i] 对应 path_node_ids[i] -> path_node_ids[i+1]。
  std::vector<double> segment_lengths_m;
  // 该单的最早可开始时刻。
  std::chrono::steady_clock::time_point earliest_start;
  // 资源 id 构造器（由调用方注入，核心不直接依赖 dispatcher 层）：
  // 边 id = f(from, to)，节点 id = g(node)。
  std::function<std::string(const std::string&, const std::string&)> edge_id_builder;
  std::function<std::string(const std::string&)> node_id_builder;
  // 期望时长（秒）：优先用它替代按长度估算（业务给的更准）。
  int expected_seconds = 0;
};

// 冲突检测结果（对一次预约尝试）。
struct TimeSpaceConflict {
  bool has_conflict = false;
  std::string resource_id;
  std::string conflicting_mission_id;
  // 冲突原因：headway（安全间隔不足）/ overlap（区间重叠）
  std::string reason;
  // 建议后移的时长（秒）。
  double suggested_delay_sec = 0.0;
};

// 一单的预约决策结果。
struct TimeSpacePlan {
  bool feasible = false;
  std::string reject_reason;
  // 每条资源最终占用的区间（可能整体后移）。
  ResourceTimeWindowSet windows;
  // 整体后移的时长（秒）。
  double total_delay_sec = 0.0;
  // 冲突诊断（供可观测与调度重排）。
  std::vector<TimeSpaceConflict> conflicts;
};

// 已占用区间表：resource_id -> 区间列表（含 mission 与安全间隔）。
struct TimeSpaceOccupancy {
  struct Entry {
    std::string mission_id;
    std::chrono::steady_clock::time_point enter;
    std::chrono::steady_clock::time_point exit;
    bool is_intersection = false;
  };
  std::unordered_map<std::string, std::vector<Entry>> by_resource;
};

// 纯函数：为一个候选任务规划时间窗。
//   1. 按 MotionTimingConfig 把路径换算成逐资源区间（含加减速/转弯/安全余量）。
//   2. 与 occupancy 比对：区间重叠 或 后车进入早于前车离开 + min_headway -> 冲突。
//   3. 冲突时整体后移到最早可行时刻（最多重试 occupancy.size()+1 次），
//      仍冲突 -> feasible=false。
//   4. 成功时返回各资源的最终窗口（调用方负责写回 occupancy）。
TimeSpacePlan PlanTimeSpaceReservation(const TimeSpaceCandidate& candidate,
                                       const TimeSpaceOccupancy& occupancy,
                                       const MotionTimingConfig& config);

// 把一个已确认的时间窗集合并入占用表（调用方在整条路径预约成功后调用）。
void CommitTimeSpaceOccupancy(TimeSpaceOccupancy* occupancy, const std::string& mission_id,
                               const ResourceTimeWindowSet& windows);

// 逐资源释放某任务的占用区间（车辆**确认驶离**该资源后调用）。
//   * 只释放该任务在该资源上的区间（owner 校验），他人区间不动；
//   * 该资源上该任务没有区间 -> 幂等 no-op（返回 false 但不改表）。
// 评审 B.2 的关键：**窗口到期 ≠ 资源已释放**——到期只意味着"计划上应该走了"，
// 物理释放必须由确认驶离驱动，否则后车会在前车还在时进入。
bool ReleaseTimeSpaceResources(TimeSpaceOccupancy* occupancy, const std::string& mission_id,
                               const std::vector<std::string>& resource_ids,
                               std::vector<std::string>* released_resource_ids);

// 某任务在某资源上是否仍持有占用区间。
bool HoldsTimeSpaceResource(const TimeSpaceOccupancy& occupancy, const std::string& mission_id,
                             const std::string& resource_id);

// 车辆执行滞后时延长其占用窗口（评审 B.2：窗口到期不能单独作为释放依据）。
// 延长失败（与后车窗口冲突）时返回 false，由调度层让后车等待。
bool ExtendTimeSpaceOccupancy(TimeSpaceOccupancy* occupancy, const std::string& mission_id,
                               const std::chrono::steady_clock::time_point& new_exit,
                               std::string* message);

}  // namespace amr_dispatcher_core::workflow
