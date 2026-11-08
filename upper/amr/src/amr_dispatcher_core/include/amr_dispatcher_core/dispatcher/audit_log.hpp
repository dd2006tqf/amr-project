// 运营操作审计记录（JSONL 追加写）。
//
// 动机：强清 / 恢复 / 封路 / 急停这些**人工干预**目前只写 RCLCPP_WARN，
// 进程重启即消失，事后无法回答"谁在什么时候以什么理由把哪台车隔离了"。
// 这里把审计记录落成一行一条 JSON（JSONL），便于 grep / jq / 直接喂给日志系统。
//
// 约定：
//   * core 不读系统时钟 —— 时间戳由调用方注入（与 traffic_reservation 等模块一致）；
//   * 追加写不做"先读后写"，避免并发下丢记录；
//   * 写失败返回 false + message，调用方自行决定是否降级为仅日志。
#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace amr_dispatcher_core::dispatcher {

struct AuditRecord {
  std::string stamp_iso8601;   // 由调用方注入（core 不读时钟）
  std::string operator_id;
  std::string action;          // force_clear / restore / block_route / estop / ...
  std::string vehicle_id;
  std::string mission_id;
  std::string reason;
  std::vector<std::string> released_resources;
};

// 单行 JSON（不含换行）。字段全部转义，保证"一行一条"不被内容破坏。
std::string SerializeAuditRecordJsonl(const AuditRecord& record);

// 追加一行到 path（不存在则创建，父目录自动创建）。
// 失败（权限/磁盘满）返回 false 并写 message；不抛异常。
bool AppendAuditRecord(const std::filesystem::path& path, const AuditRecord& record,
                       std::string* message);

}  // namespace amr_dispatcher_core::dispatcher
