// 拓扑连通性分析（纯函数）。
//
// 为什么必须有它：封一条边看起来是局部操作，但如果那条边是两片区域之间唯一的
// 通路，封完之后某些站点就变成"孤岛"——调度器会把任务派给一台永远到不了的车。
// 这类错误在提交时看不出来，要到出车失败才暴露，代价是一趟空跑 + 现场排查。
//
// 判定规则（比"图连通"更严格的实用口径）：
//   * 所有**站点**必须互相可达（强连通，尊重 bidirectional 与 enabled）；
//   * 路网节点允许是死胡同（骨架提取常留短支路），只作为 warning 报出。
// 用 Tarjan SCC 求强连通分量：唯一"含全部站点"的 SCC 才叫健康。
#pragma once

#include <string>
#include <vector>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"

namespace amr_dispatcher_core::catalog {

struct ReachabilityReport {
  bool all_stations_reachable = false;
  // 每个强连通分量的成员（排序后，便于稳定输出与 diff）。只包含含站点的分量。
  std::vector<std::vector<std::string>> components;
  // 不在"主分量"里的站点：提交必须因此被拒绝
  std::vector<std::string> unreachable_stations;
  // 只能单向到达/离开的路网节点（死胡同）：仅告警
  std::vector<std::string> dead_end_road_nodes;
  std::string message;
};

ReachabilityReport AnalyzeTopologyConnectivity(const StationCatalog& catalog);

}  // namespace amr_dispatcher_core::catalog
