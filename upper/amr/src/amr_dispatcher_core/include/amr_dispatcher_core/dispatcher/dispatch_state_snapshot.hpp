// 调度状态快照：把"进程重启即失"的调度状态（排队任务、暂停订单、在途任务、
// 隔离车辆、序号游标）落成一份 YAML 文本快照。
//
// 设计约束：
//   * 零新依赖 —— yaml-cpp 已是 core 的既有依赖；
//   * 纯数据 + 纯 IO，不读系统时钟、不碰 ROS；
//   * 落盘必须原子（tmp -> flush 校验 -> 回读校验 -> rename），
//     读到的要么是旧快照、要么是新快照，绝不是半截文件；
//   * map_hash 与拓扑版本对齐：拿旧拓扑的队列去派发 = 派到不存在的站点，
//     因此不匹配时**拒绝恢复**而不是"尽力而为"。
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "amr_dispatcher_core/dispatcher/mission_state.hpp"

namespace amr_dispatcher_core::dispatcher {

inline constexpr std::uint32_t kDispatchStateFormatVersion = 1;

// 重启后"仍在执行"的任务：只保留重建所需的最小信息。
// 恢复时这些任务**重新入队**（新进程没有任何车辆在执行它们，不能假装还在跑）。
struct MissionSnapshotRecord {
  Mission mission;
  std::string assigned_vehicle_id;
};

struct DispatchStateSnapshot {
  std::uint32_t format_version = kDispatchStateFormatVersion;
  std::string map_hash;                      // 拓扑版本；不匹配则拒绝恢复
  std::uint64_t next_sequence = 1;           // 序号游标，恢复后不得回退
  std::vector<Mission> queued_missions;      // 按出队顺序（已含 priority/sequence）
  std::vector<std::string> paused_order_ids;
  std::vector<Mission> active_missions;      // 重启时仍在执行的任务
  std::vector<std::string> quarantined_vehicle_ids;
};

// 序列化为 YAML 文本。
std::string SerializeDispatchState(const DispatchStateSnapshot& snapshot);

// 反序列化；格式不合法返回 nullopt 并写入 message（含具体原因）。
std::optional<DispatchStateSnapshot> DeserializeDispatchState(const std::string& text,
                                                             std::string* message);

// 原子落盘。任一步失败都会删除 tmp 并保留旧文件，返回 false + message。
bool WriteDispatchStateAtomic(const std::filesystem::path& path,
                              const DispatchStateSnapshot& snapshot, std::string* message);

// 读取并校验。文件不存在返回 nullopt + message = "not_found"（调用方据此区分
// "首次启动"与"文件损坏"）。
std::optional<DispatchStateSnapshot> LoadDispatchState(const std::filesystem::path& path,
                                                       std::string* message);

}  // namespace amr_dispatcher_core::dispatcher
