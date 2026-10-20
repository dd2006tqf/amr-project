#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/dispatcher/traffic_reservation.hpp"
#include "amr_dispatcher_core/fleet/vehicle_adapter_interface.hpp"

namespace amr_dispatcher_core::fleet {

// Vda5050OrderBuilder 默认配置。
struct Vda5050OrderBuilderConfig {
  double default_max_speed_mps = 1.0;
  std::string default_map_id = "map";
};

// 将调度选出的拓扑路径节点序列转换为标准 VehicleOrder（Spec §4.2 / §5）。
//
// 参数:
//   - vehicle_id: 目标小车 ID
//   - mission_id: 业务任务 ID（作为 order_id，保证整单可寻址）
//   - order_update_id: 订单更新版本号
//   - path_node_ids: Dijkstra 产出的节点 ID 序列（例如 ["station_A", "rn_1", "station_B"]）
//   - catalog: 当前生效的路网拓扑目录（stations + road_nodes + 带 polyline 的 edges）
//   - reserved_locks: 当前由 ResourceTable 成功上锁的资源集合（RouteNodeId / RouteEdgeId）
//   - terminal_actions: 挂载在终点站点的动作（取货/卸货/充电等）
//   - config: 默认速度与地图 ID 配置
//
// 放行规则:
//   - 节点锁 RouteNodeId 存在 -> node.released = true（Base 区间），否则 false（Horizon 区间）
//   - 边锁 RouteEdgeId 存在且两端节点均已放行 -> edge.released = true，否则 false
//   - 协议一致性：一旦出现 released = false，其后所有节点与边一律拉平为 false
//     （VDA 5050 规定 Horizon 区间之后不得再出现 Base 区间）
//
// 返回: 路径为空或任一节点在目录中缺失时返回 std::nullopt。
std::optional<VehicleOrder> BuildVda5050OrderFromPath(
    const std::string& vehicle_id,
    const std::string& mission_id,
    uint32_t order_update_id,
    const std::vector<std::string>& path_node_ids,
    const catalog::StationCatalog& catalog,
    const std::unordered_set<std::string>& reserved_locks,
    const std::vector<VehicleAction>& terminal_actions = {},
    const Vda5050OrderBuilderConfig& config = {});

}  // namespace amr_dispatcher_core::fleet
