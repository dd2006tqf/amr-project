// 拓扑变更的在途影响分析验收。
//
// 目标：把"封一条边会不会弄坏正在跑的车"这个判断变成可断言的事实。
// 判定的优先级本身就是安全语义（顺序写错 = 车在半路失去路权）：
//   已在终点 < 剩余航段仍可通行 < 正压在边上（拒绝） < 需要绕行 < 无替代（拒绝）
#include "amr_dispatcher_core/workflow/topology_change_impact.hpp"

#include <gtest/gtest.h>

#include <algorithm>

#include "amr_dispatcher_core/catalog/topology_overlay.hpp"

namespace amr_dispatcher_core::workflow {
namespace {

using catalog::CatalogStationEdge;
using catalog::OverlayEdge;
using catalog::StationCatalog;
using catalog::Station;
using catalog::TopologyOverlay;

// a -- b -- c   （直链）
//  \         /
//   +- n1 --+    （绕行通道）
StationCatalog MakeCatalogWithDetour() {
  StationCatalog catalog;
  catalog.map_hash = "h";
  catalog.stations.push_back(Station{"a", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"b", "map", 1.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"c", "map", 2.0, 0.0, 0.0});
  catalog.road_nodes.push_back(catalog::RoadNode{"n1", "map", 1.0, 2.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"a", "b", 1.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"b", "c", 1.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"a", "n1", 2.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"n1", "c", 2.0, true, true});
  return catalog;
}

// 纯直链：没有绕行可能，用来构造"无替代路线"
StationCatalog MakePlainChain() {
  StationCatalog catalog;
  catalog.map_hash = "h";
  catalog.stations.push_back(Station{"a", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"b", "map", 1.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"c", "map", 2.0, 0.0, 0.0});
  catalog.edges.push_back(CatalogStationEdge{"a", "b", 1.0, true, true});
  catalog.edges.push_back(CatalogStationEdge{"b", "c", 1.0, true, true});
  return catalog;
}

OverlayEdge MakeOverlayEdge(const std::string& from, const std::string& to) {
  OverlayEdge edge;
  edge.from = from;
  edge.to = to;
  edge.reason = "construction";
  edge.operator_id = "op";
  return edge;
}

InFlightMissionView MakeMission(const std::string& id, const std::string& vehicle,
                                const std::string& from_node, const std::string& to_node,
                                std::vector<std::string> path) {
  InFlightMissionView view;
  view.mission_id = id;
  view.vehicle_id = vehicle;
  view.current_safe_node = from_node;
  view.destination_node = to_node;
  view.full_original_path = std::move(path);
  return view;
}

bool Contains(const std::vector<std::string>& v, const std::string& value) {
  return std::find(v.begin(), v.end(), value) != v.end();
}

const VehicleImpact& FindImpact(const TopologyChangeImpact& impact, const std::string& mission_id) {
  static VehicleImpact kMissing;
  for (const auto& vehicle : impact.vehicles) {
    if (vehicle.mission_id == mission_id) {
      return vehicle;
    }
  }
  return kMissing;
}

// ---------------------------------------------------------------- 无影响

TEST(TopologyChangeImpactTest, NoInFlightVehiclesIsSafe) {
  const auto catalog = MakeCatalogWithDetour();
  TopologyChangeImpactRequest request;
  request.new_catalog = &catalog;

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_TRUE(impact.safe_to_apply);
  EXPECT_EQ(impact.error_code, "ok");
  EXPECT_EQ(impact.message, "no in-flight missions");
}

// 最常见的情况：变更的边根本不在车的剩余航段上。
// 此时**不能**产生任何锁置换 —— 无故释放再申请会制造一次无意义的资源抖动。
TEST(TopologyChangeImpactTest, UnaffectedVehicleIsNoOp) {
  auto base = MakeCatalogWithDetour();
  auto overlay = TopologyOverlay{};
  overlay.base_map_hash = base.map_hash;
  overlay.blocked_edges.push_back(MakeOverlayEdge("a", "n1"));  // 不在 a->b->c 上
  const auto new_catalog = catalog::ApplyTopologyOverlay(base, overlay);

  TopologyChangeImpactRequest request;
  request.new_catalog = &new_catalog;
  auto mission = MakeMission("m1", "v1", "a", "c", {"a", "b", "c"});
  mission.current_held_locks = {"route_node:a", "route_edge:a__b", "route_node:b"};
  request.missions.push_back(mission);

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_TRUE(impact.safe_to_apply);
  const auto& result = FindImpact(impact, "m1");
  EXPECT_EQ(result.kind, VehicleImpact::Kind::kUnaffected);
  EXPECT_TRUE(result.new_required_locks.empty());
  EXPECT_TRUE(result.obsolete_locks.empty());
  EXPECT_EQ(impact.reroute_count, 0u);
}

TEST(TopologyChangeImpactTest, NoMissionsMeansNoLocksChurn) {
  const auto catalog = MakeCatalogWithDetour();
  TopologyChangeImpactRequest request;
  request.new_catalog = &catalog;
  request.missions.clear();
  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_TRUE(impact.vehicles.empty());
  EXPECT_TRUE(impact.safe_to_apply);
}

// 已到终点的车不参与任何判定
TEST(TopologyChangeImpactTest, AlreadyAtGoalIsIgnored) {
  const auto catalog = MakeCatalogWithDetour();
  TopologyChangeImpactRequest request;
  request.new_catalog = &catalog;
  request.missions.push_back(MakeMission("m1", "v1", "c", "c", {"a", "b", "c"}));

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_TRUE(impact.safe_to_apply);
  EXPECT_EQ(FindImpact(impact, "m1").kind, VehicleImpact::Kind::kAlreadyAtGoal);
}

// ---------------------------------------------------------------- 绕行

// 封掉 b-c：车在 a、剩余航段 a->b->c 的第二段断了，但存在 a->n1->c 的替代路线。
TEST(TopologyChangeImpactTest, ReroutableVehicleProducesTwoPhaseLocks) {
  auto base = MakeCatalogWithDetour();
  auto overlay = TopologyOverlay{};
  overlay.base_map_hash = base.map_hash;
  overlay.blocked_edges.push_back(MakeOverlayEdge("b", "c"));
  const auto new_catalog = catalog::ApplyTopologyOverlay(base, overlay);

  TopologyChangeImpactRequest request;
  request.new_catalog = &new_catalog;
  auto mission = MakeMission("m1", "v1", "a", "c", {"a", "b", "c"});
  mission.current_held_locks = {"route_node:a", "route_edge:a__b", "route_node:b",
                                "route_edge:b__c", "route_node:c"};
  request.missions.push_back(mission);

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_TRUE(impact.safe_to_apply) << impact.message;
  EXPECT_EQ(impact.error_code, "ok");
  EXPECT_EQ(impact.reroute_count, 1u);

  const auto& result = FindImpact(impact, "m1");
  EXPECT_EQ(result.kind, VehicleImpact::Kind::kRerouteReady);
  // 新路线必须经过 n1（绕开被封的 b-c）
  EXPECT_TRUE(Contains(result.full_updated_path, "n1")) << "new route should use the detour";
  EXPECT_FALSE(result.new_required_locks.empty());
  EXPECT_FALSE(result.obsolete_locks.empty());

  // 两阶段置换的语义：新锁 = 需要新申请的；旧锁 = 以后不再需要、可以释放的
  EXPECT_TRUE(Contains(result.new_required_locks, "route_node:n1"));
  EXPECT_TRUE(Contains(result.obsolete_locks, "route_edge:b__c"));
}

// 【安全铁律】车当前停靠节点的锁绝不能被划进 obsolete_locks ——
// 否则在"先申请新锁、再释放旧锁"的两阶段之间，车会短暂失去物理立足点的保护。
TEST(TopologyChangeImpactTest, PreservesCurrentSafeNodeLock) {
  auto base = MakeCatalogWithDetour();
  auto overlay = TopologyOverlay{};
  overlay.base_map_hash = base.map_hash;
  overlay.blocked_edges.push_back(MakeOverlayEdge("b", "c"));
  const auto new_catalog = catalog::ApplyTopologyOverlay(base, overlay);

  TopologyChangeImpactRequest request;
  request.new_catalog = &new_catalog;
  auto mission = MakeMission("m1", "v1", "a", "c", {"a", "b", "c"});
  mission.current_held_locks = {"route_node:a", "route_edge:a__b", "route_node:b"};
  request.missions.push_back(mission);

  const auto impact = AnalyzeTopologyChangeImpact(request);
  const auto& result = FindImpact(impact, "m1");
  ASSERT_EQ(result.kind, VehicleImpact::Kind::kRerouteReady);
  EXPECT_FALSE(Contains(result.obsolete_locks, "route_node:a"))
      << "the vehicle's current safe node lock must never be released";
  // 也不该把"自己已经持有的锁"重复申请
  EXPECT_FALSE(Contains(result.new_required_locks, "route_node:a"));
  EXPECT_TRUE(Contains(result.new_required_locks, "route_node:n1"));
}

// 单向变更同样可能逼出绕行（且方向必须被正确理解）。
TEST(TopologyChangeImpactTest, OneWayReversalForcesReroute) {
  auto base = MakeCatalogWithDetour();
  auto overlay = TopologyOverlay{};
  overlay.base_map_hash = base.map_hash;
  // b-c 只许 c -> b，于是 a->b->c 的第二段不可走，必须走 a->n1->c
  overlay.one_way_edges.push_back(MakeOverlayEdge("c", "b"));
  const auto new_catalog = catalog::ApplyTopologyOverlay(base, overlay);

  TopologyChangeImpactRequest request;
  request.new_catalog = &new_catalog;
  auto mission = MakeMission("m1", "v1", "a", "c", {"a", "b", "c"});
  mission.current_held_locks = {"route_node:a"};
  request.missions.push_back(mission);

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_TRUE(impact.safe_to_apply) << impact.message;
  const auto& result = FindImpact(impact, "m1");
  EXPECT_EQ(result.kind, VehicleImpact::Kind::kRerouteReady);
  EXPECT_TRUE(Contains(result.full_updated_path, "n1"));
}

// ---------------------------------------------------------------- 拒绝

// 【安全核心】车可能正压在要被封的边上：`lastNodeId` 只报告"最后确认到达的节点"，
// 它已经在去下一段的路上。让它半路失去路权在物理上不安全 —— 必须拒绝。
TEST(TopologyChangeImpactTest, VehicleOnBlockedEdgeIsRejected) {
  auto base = MakeCatalogWithDetour();
  auto overlay = TopologyOverlay{};
  overlay.base_map_hash = base.map_hash;
  overlay.blocked_edges.push_back(MakeOverlayEdge("a", "b"));  // 正是离开当前安全节点的那条
  const auto new_catalog = catalog::ApplyTopologyOverlay(base, overlay);

  TopologyChangeImpactRequest request;
  request.new_catalog = &new_catalog;
  auto mission = MakeMission("m1", "v1", "a", "c", {"a", "b", "c"});
  mission.current_held_locks = {"route_node:a", "route_edge:a__b"};
  request.missions.push_back(mission);

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_FALSE(impact.safe_to_apply);
  EXPECT_EQ(impact.error_code, "edge_occupied_in_flight");
  EXPECT_EQ(impact.occupied_count, 1u);
  EXPECT_EQ(FindImpact(impact, "m1").kind, VehicleImpact::Kind::kOnBlockedEdge);
}

// 操作员明确确认"车会安全驶离"时才允许提交；
// 此时语义是"只对新规划生效"，在途车占用的边锁保留到它自己释放。
TEST(TopologyChangeImpactTest, AllowEdgeUnderVehicleOverrides) {
  auto base = MakeCatalogWithDetour();
  auto overlay = TopologyOverlay{};
  overlay.base_map_hash = base.map_hash;
  overlay.blocked_edges.push_back(MakeOverlayEdge("a", "b"));
  const auto new_catalog = catalog::ApplyTopologyOverlay(base, overlay);

  TopologyChangeImpactRequest request;
  request.new_catalog = &new_catalog;
  request.allow_edge_under_vehicle = true;
  auto mission = MakeMission("m1", "v1", "a", "c", {"a", "b", "c"});
  mission.current_held_locks = {"route_node:a", "route_edge:a__b"};
  request.missions.push_back(mission);

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_TRUE(impact.safe_to_apply) << impact.message;
  const auto& result = FindImpact(impact, "m1");
  EXPECT_EQ(result.kind, VehicleImpact::Kind::kUnaffected);
  // 覆盖放行时不产生锁置换：该边在预约层的锁必须留给在途车
  EXPECT_TRUE(result.new_required_locks.empty());
  EXPECT_TRUE(result.obsolete_locks.empty());
  EXPECT_EQ(impact.occupied_count, 0u);
}

// 拓扑上根本没有替代路线：提交会让这台车永远到不了终点。
TEST(TopologyChangeImpactTest, NoAlternativeIsRejected) {
  auto base = MakePlainChain();
  auto overlay = TopologyOverlay{};
  overlay.base_map_hash = base.map_hash;
  overlay.blocked_edges.push_back(MakeOverlayEdge("b", "c"));
  const auto new_catalog = catalog::ApplyTopologyOverlay(base, overlay);

  TopologyChangeImpactRequest request;
  request.new_catalog = &new_catalog;
  auto mission = MakeMission("m1", "v1", "a", "c", {"a", "b", "c"});
  mission.current_held_locks = {"route_node:a"};
  request.missions.push_back(mission);

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_FALSE(impact.safe_to_apply);
  EXPECT_EQ(impact.error_code, "would_strand_vehicles");
  EXPECT_EQ(impact.stranded_count, 1u);
  EXPECT_EQ(FindImpact(impact, "m1").kind, VehicleImpact::Kind::kNoAlternative);
}

// 混合车队：只要有一台车不可行，整次变更就拒绝（不做部分应用）。
// 部分应用正是"有些车回得了家、有些车回不了"的成因。
//
// 构造要点：被封的边必须**不邻接**在途车的当前安全节点，否则会先命中
// kOnBlockedEdge（那是更早、更严格的一档），测不到"无替代路线"这条路径。
TEST(TopologyChangeImpactTest, MixedFleetAnyStrandBlocksAll) {
  StationCatalog base;
  base.map_hash = "h";
  base.stations.push_back(Station{"a", "map", 0.0, 0.0, 0.0});
  base.stations.push_back(Station{"b", "map", 1.0, 0.0, 0.0});
  base.stations.push_back(Station{"c", "map", 2.0, 0.0, 0.0});
  base.stations.push_back(Station{"d", "map", 3.0, 0.0, 0.0});
  base.road_nodes.push_back(catalog::RoadNode{"n1", "map", 1.0, 2.0, 0.0});
  base.edges.push_back(CatalogStationEdge{"a", "b", 1.0, true, true});
  base.edges.push_back(CatalogStationEdge{"b", "c", 1.0, true, true});
  base.edges.push_back(CatalogStationEdge{"c", "d", 1.0, true, true});
  base.edges.push_back(CatalogStationEdge{"a", "n1", 2.0, true, true});
  base.edges.push_back(CatalogStationEdge{"n1", "c", 2.0, true, true});

  auto overlay = TopologyOverlay{};
  overlay.base_map_hash = base.map_hash;
  overlay.blocked_edges.push_back(MakeOverlayEdge("c", "d"));  // 只通向 d 的那一段
  const auto new_catalog = catalog::ApplyTopologyOverlay(base, overlay);

  TopologyChangeImpactRequest request;
  request.new_catalog = &new_catalog;
  // 这台不受影响：目的地是 c，剩余航段 a->b->c 全通
  request.missions.push_back(MakeMission("m_ok", "v1", "a", "c", {"a", "b", "c"}));
  // 这台无解：目的地 d，唯一通路 c->d 被封，且 c 不是它的当前节点（绕行尝试后失败）
  request.missions.push_back(MakeMission("m_stranded", "v2", "a", "d", {"a", "b", "c", "d"}));

  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_FALSE(impact.safe_to_apply);
  EXPECT_EQ(impact.error_code, "would_strand_vehicles");
  EXPECT_EQ(impact.stranded_count, 1u);
  EXPECT_EQ(FindImpact(impact, "m_ok").kind, VehicleImpact::Kind::kUnaffected);
  EXPECT_EQ(FindImpact(impact, "m_stranded").kind, VehicleImpact::Kind::kNoAlternative);
}

// ---------------------------------------------------------------- 兜底

TEST(TopologyChangeImpactTest, NullCatalogIsInternalError) {
  TopologyChangeImpactRequest request;
  request.new_catalog = nullptr;
  const auto impact = AnalyzeTopologyChangeImpact(request);
  EXPECT_FALSE(impact.safe_to_apply);
  EXPECT_EQ(impact.error_code, "internal_error");
}

// 起点不在归档路径里（数据不一致）：保守判为不可通行，绝不假设"没事"。
TEST(TopologyChangeImpactTest, UnknownStartNodeIsNotTraversable) {
  const auto catalog = MakeCatalogWithDetour();
  std::vector<std::string> blocked;
  EXPECT_FALSE(RemainingPathTraversable(catalog, {"a", "b", "c"}, "nowhere", &blocked));
  ASSERT_EQ(blocked.size(), 1u);
  EXPECT_EQ(blocked.front(), "unknown_start:nowhere");
}

TEST(TopologyChangeImpactTest, RemainingPathTraversableRespectsDisabledEdges) {
  auto catalog = MakeCatalogWithDetour();
  EXPECT_TRUE(RemainingPathTraversable(catalog, {"a", "b", "c"}, "a", nullptr));

  catalog.edges[1].enabled = false;  // b -> c
  std::vector<std::string> blocked;
  EXPECT_FALSE(RemainingPathTraversable(catalog, {"a", "b", "c"}, "a", &blocked));
  ASSERT_EQ(blocked.size(), 1u);
  EXPECT_EQ(blocked.front(), "route_edge:b__c");
}

// 已驶过的航段不再影响判定：从 b 出发时 a->b 被封无所谓。
TEST(TopologyChangeImpactTest, AlreadyPassedSegmentDoesNotMatter) {
  auto catalog = MakeCatalogWithDetour();
  catalog.edges[0].enabled = false;  // a -> b 封了
  std::vector<std::string> blocked;
  EXPECT_TRUE(RemainingPathTraversable(catalog, {"a", "b", "c"}, "b", &blocked));
  EXPECT_TRUE(blocked.empty());
}

}  // namespace
}  // namespace amr_dispatcher_core::workflow
