// 订单批处理 / 拼单（Order Batching & Bundling）纯函数。
//
// 背景：在上游业务（WMS/MES）下发多个订单时，往往存在具有相同起止站点
// 或具有相近站点序的任务。如果逐单单独派发，会导致多车重复空跑相同路线；
// 如果能将可合并的订单 bundle 成为多 leg 的 station sequence 或合并起送站点，
// 可以大幅提升车队运行吞吐并减少路线锁争用。
//
// 本模块提供：
// 1. OrderBatchBundleCandidate：表示可合并订单候选条目。
// 2. PlanOrderBatchBundling：纯函数，对输入订单列表进行批处理聚合，将同一路径/同站订单
//    归集并转化为具有有序 legs 的 StationSequence 任务候选。
// 3. 铁律：本模块纯 C++、无 ROS 依赖，100% 宿主单测可覆盖。
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::workflow {

struct OrderBatchItem {
  std::string order_id;
  std::string pickup_station;
  std::string dropoff_station;
  int priority = 0;
  int64_t deadline_unix_ms = 0;
};

struct BundledOrderLeg {
  std::string pickup_station;
  std::string dropoff_station;
  std::vector<std::string> bundled_order_ids;
};

struct BundledOrderGroup {
  std::string group_id;
  int priority = 0;
  int64_t earliest_deadline_unix_ms = 0;
  std::vector<std::string> order_ids;
  std::vector<BundledOrderLeg> legs;
  // 沿途站点序列（如 receiving -> storage_a -> packing）
  std::vector<std::string> station_sequence;
};

struct OrderBatchBundleRequest {
  std::string batch_id;
  std::vector<OrderBatchItem> items;
  // 是否允许跨站串联拼单（如 A->B 和 B->C 串联成 A->B->C）
  bool enable_route_chaining = true;
};

struct OrderBatchBundleResult {
  bool success = true;
  std::string message;
  std::vector<BundledOrderGroup> groups;
};

OrderBatchBundleResult PlanOrderBatchBundling(const OrderBatchBundleRequest& request);

}  // namespace amr_dispatcher_core::workflow
