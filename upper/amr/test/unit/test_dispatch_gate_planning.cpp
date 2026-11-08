#include "amr_dispatcher_core/workflow/dispatch_gate_planning.hpp"

#include <gtest/gtest.h>

using amr_dispatcher_core::workflow::DispatchGateDecision;
using amr_dispatcher_core::workflow::DispatchGateRequest;
using amr_dispatcher_core::workflow::EvaluateDispatchGates;

namespace {

// 默认构造一份"全部放行"的请求，各用例只改一处。
DispatchGateRequest MakePassingRequest() {
  DispatchGateRequest request;
  request.queue_empty = false;
  request.active_mission_count = 0;
  request.max_active_missions = 3;
  request.has_free_vehicle_slot = true;
  request.expected_map_hash = "fnv1a64_abc";
  request.catalog_loaded = true;
  request.catalog_map_hash = "fnv1a64_abc";
  return request;
}

}  // namespace

TEST(DispatchGatePlanningTest, AllGatesOpenMeansPass) {
  const auto decision = EvaluateDispatchGates(MakePassingRequest());
  EXPECT_EQ(decision.kind, DispatchGateDecision::Kind::kPass);
  EXPECT_TRUE(decision.map_gate_evaluated);
  EXPECT_TRUE(decision.map_version_valid);
  EXPECT_TRUE(decision.reason.empty());
}

TEST(DispatchGatePlanningTest, EmptyQueueBlocksBeforeAnythingElse) {
  auto request = MakePassingRequest();
  request.queue_empty = true;
  request.active_mission_count = 99;  // 容量也超了，但队列空优先
  const auto decision = EvaluateDispatchGates(request);
  EXPECT_EQ(decision.kind, DispatchGateDecision::Kind::kQueueEmpty);
  EXPECT_FALSE(decision.map_gate_evaluated);
}

// 优先顺序：容量满时不该去算地图哈希（切分前就是先 return）。
TEST(DispatchGatePlanningTest, CapacityExceededBeatsMapMismatch) {
  auto request = MakePassingRequest();
  request.active_mission_count = 3;
  request.catalog_map_hash = "fnv1a64_stale";
  const auto decision = EvaluateDispatchGates(request);
  EXPECT_EQ(decision.kind, DispatchGateDecision::Kind::kCapacityExceeded);
  EXPECT_FALSE(decision.map_gate_evaluated);
  EXPECT_TRUE(decision.map_version_valid);  // 未评估 -> 保持默认 true，不覆盖调用方状态
}

TEST(DispatchGatePlanningTest, NoFreeSlotBlocks) {
  auto request = MakePassingRequest();
  request.has_free_vehicle_slot = false;
  EXPECT_EQ(EvaluateDispatchGates(request).kind,
            DispatchGateDecision::Kind::kNoFreeVehicleSlot);
}

TEST(DispatchGatePlanningTest, CapacityBoundaryIsInclusive) {
  auto request = MakePassingRequest();
  request.max_active_missions = 2;
  request.active_mission_count = 1;
  EXPECT_EQ(EvaluateDispatchGates(request).kind, DispatchGateDecision::Kind::kPass);
  request.active_mission_count = 2;  // >= 上限即拦
  EXPECT_EQ(EvaluateDispatchGates(request).kind,
            DispatchGateDecision::Kind::kCapacityExceeded);
}

TEST(DispatchGatePlanningTest, MapHashMismatchPausesDispatch) {
  auto request = MakePassingRequest();
  request.catalog_map_hash = "fnv1a64_other";
  const auto decision = EvaluateDispatchGates(request);
  EXPECT_EQ(decision.kind, DispatchGateDecision::Kind::kMapVersionMismatch);
  EXPECT_TRUE(decision.map_gate_evaluated);
  EXPECT_FALSE(decision.map_version_valid);
}

// 目录里没有哈希（例如地图导入失败）同样算不匹配：绝不盲派。
TEST(DispatchGatePlanningTest, MissingCatalogHashIsMismatch) {
  auto request = MakePassingRequest();
  request.catalog_map_hash.clear();
  const auto decision = EvaluateDispatchGates(request);
  EXPECT_EQ(decision.kind, DispatchGateDecision::Kind::kMapVersionMismatch);
  EXPECT_FALSE(decision.map_version_valid);
}

TEST(DispatchGatePlanningTest, EmptyExpectedHashDisablesMapGate) {
  auto request = MakePassingRequest();
  request.expected_map_hash.clear();
  request.catalog_map_hash = "anything";
  const auto decision = EvaluateDispatchGates(request);
  EXPECT_EQ(decision.kind, DispatchGateDecision::Kind::kPass);
  EXPECT_TRUE(decision.map_gate_evaluated);  // 走到了门禁，但未配置即通过
  EXPECT_TRUE(decision.map_version_valid);
}

TEST(DispatchGatePlanningTest, CatalogNotLoadedSkipsMapGate) {
  auto request = MakePassingRequest();
  request.catalog_loaded = false;
  request.catalog_map_hash.clear();
  const auto decision = EvaluateDispatchGates(request);
  EXPECT_EQ(decision.kind, DispatchGateDecision::Kind::kPass);
  EXPECT_TRUE(decision.map_version_valid);
}

// 默认构造 = "什么都还没有"：不得放行（防止调用方漏填字段时误派发）。
TEST(DispatchGatePlanningTest, DefaultRequestDoesNotPass) {
  const DispatchGateRequest request;
  EXPECT_NE(EvaluateDispatchGates(request).kind, DispatchGateDecision::Kind::kPass);
}

TEST(DispatchGatePlanningTest, ZeroCapacityBlocksEverything) {
  auto request = MakePassingRequest();
  request.max_active_missions = 0;
  EXPECT_EQ(EvaluateDispatchGates(request).kind,
            DispatchGateDecision::Kind::kCapacityExceeded);
}
