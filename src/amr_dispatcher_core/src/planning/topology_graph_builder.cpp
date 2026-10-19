#include "amr_dispatcher_core/planning/topology_graph_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::planning {
namespace {

// 8 邻域方向（顺时针，自正北起）：与 skeleton_extractor 保持一致。
constexpr int kDRow[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
constexpr int kDCol[8] = {0, 1, 1, 1, 0, -1, -1, -1};

// 像素坐标（整数栅格）。
struct Pixel {
  int col = 0;
  int row = 0;

  bool operator==(const Pixel& other) const {
    return col == other.col && row == other.row;
  }
};

struct PixelHash {
  size_t operator()(const Pixel& p) const {
    return (static_cast<size_t>(static_cast<uint32_t>(p.col)) << 32) ^
           static_cast<size_t>(static_cast<uint32_t>(p.row));
  }
};

// 8 邻域环上的连通组数（P2..P9,P2 环状序列中 0→1 跳变次数）：
//   0 = 孤立，1 = 端点，2 = 中段，>= 3 = 分叉。
// 与 skeleton_extractor.cpp 中 CrossingNumber 完全相同的判据。
int CrossingNumber(const BinaryGrid& grid, int col, int row) {
  int neighbors[8];
  int B = 0;
  for (int i = 0; i < 8; ++i) {
    neighbors[i] = grid.GetValue(col + kDCol[i], row + kDRow[i]) ? 1 : 0;
    B += neighbors[i];
  }
  if (B == 0) {
    return 0;
  }
  if (B == 8) {
    return 1;
  }
  int groups = 0;
  for (int i = 0; i < 8; ++i) {
    if (!neighbors[i] && neighbors[(i + 1) % 8]) {
      ++groups;
    }
  }
  return groups;
}

// 折线累加长度：相邻折线点间欧氏距离之和。
double PolylineLength(const std::vector<std::pair<double, double>>& polyline) {
  double total = 0.0;
  for (size_t i = 1; i < polyline.size(); ++i) {
    const double dx = polyline[i].first - polyline[i - 1].first;
    const double dy = polyline[i].second - polyline[i - 1].second;
    total += std::hypot(dx, dy);
  }
  return total;
}

// 点 P 到线段 AB 的最近点（垂足或端点）。
// 返回 (proj_x, proj_y, dist, t)，t 在 [0,1] 表示投影参数位置。
std::pair<std::pair<double, double>, double> ProjectPointToSegment(
    double px, double py, double ax, double ay, double bx, double by,
    double* t_out) {
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = px - ax;
  const double apy = py - ay;
  const double ab_len_sq = abx * abx + aby * aby;
  double t = 0.0;
  if (ab_len_sq > 0.0) {
    t = (apx * abx + apy * aby) / ab_len_sq;
    t = std::max(0.0, std::min(1.0, t));
  }
  const double qx = ax + t * abx;
  const double qy = ay + t * aby;
  const double dist = std::hypot(px - qx, py - qy);
  if (t_out != nullptr) {
    *t_out = t;
  }
  return {{qx, qy}, dist};
}

// ---- 骨架图追踪 -------------------------------------------------------------

// 内部图边（像素域）：from/to 为节点像素，polyline_px 为从 from 到 to 的
// 像素序列（含两端节点）。在生成 CatalogStationEdge 时再统一转世界坐标。
struct PixelEdge {
  Pixel from;
  Pixel to;
  std::vector<Pixel> polyline_px;
};

// 提取所有骨架特征点（crossing != 2 的像素）。返回特征点集合。
std::vector<Pixel> ExtractFeaturePixels(const BinaryGrid& skeleton) {
  std::vector<Pixel> features;
  for (uint32_t row = 0; row < skeleton.height; ++row) {
    for (uint32_t col = 0; col < skeleton.width; ++col) {
      const int c = static_cast<int>(col);
      const int r = static_cast<int>(row);
      if (!skeleton.GetValue(c, r)) {
        continue;
      }
      const int crossings = CrossingNumber(skeleton, c, r);
      if (crossings != 2) {
        features.push_back({c, r});
      }
    }
  }
  return features;
}

// 单步推进：从 prev -> cur，返回所有可能的下一像素（除 prev 之外的骨架
// 邻居）。返回的下一像素若仍是中段（crossing==2）需要继续推进。
std::vector<Pixel> NextPixels(const BinaryGrid& skeleton, const Pixel& prev,
                              const Pixel& cur) {
  std::vector<Pixel> next;
  for (int i = 0; i < 8; ++i) {
    const Pixel nb{cur.col + kDCol[i], cur.row + kDRow[i]};
    if (nb == prev) {
      continue;
    }
    if (skeleton.GetValue(nb.col, nb.row)) {
      next.push_back(nb);
    }
  }
  return next;
}

// 在多候选下一像素中选择主续像素：
//   1. 优先选不在当前链中的像素（避免回环自相交）；
//   2. 其中选与来向 (cur - prev) 点积最大者（直线延续优先于斜向逃逸）。
// 这处理 1 像素宽骨架在 8 邻域下的"环上相邻"情况：L 形拐角内侧像素
// 会有 3 个骨架邻居（prev、内侧对角、正前方），ring 连通组仍为 2，
// 必须靠方向选择走出拐角而不是误判为分叉。
Pixel PickNextPixel(const Pixel& prev, const Pixel& cur,
                    const std::vector<Pixel>& nexts,
                    const std::unordered_set<Pixel, PixelHash>& chain_set) {
  const int dir_col = cur.col - prev.col;
  const int dir_row = cur.row - prev.row;
  Pixel best{-1, -1};
  double best_cos = -std::numeric_limits<double>::infinity();
  bool best_fresh = false;
  for (const Pixel& nb : nexts) {
    const bool fresh = chain_set.count(nb) == 0;
    const int step_col = nb.col - cur.col;
    const int step_row = nb.row - cur.row;
    const double step_len =
        (step_col == 0 || step_row == 0) ? 1.0 : std::sqrt(2.0);
    const double cos_angle =
        (dir_col * step_col + dir_row * step_row) / step_len;
    // 未访问优先；并列时按方向余弦。
    if (best.col == -1 || (fresh && !best_fresh) ||
        (fresh == best_fresh && cos_angle > best_cos)) {
      best = nb;
      best_cos = cos_angle;
      best_fresh = fresh;
    }
  }
  return best;
}

// 从起点 start 沿邻居 first_step 方向追踪，直到抵达下一个特征点。
// 返回包含 start、所有中间像素、终止特征点的完整链。
// 若 first_step 本身即特征点，则链长为 2。
// 若链成环回到起点（无其他特征点），返回空——由外层做闭合环处理。
std::vector<Pixel> TraceFromToFeature(
    const BinaryGrid& skeleton, const Pixel& start, const Pixel& first_step,
    const std::unordered_set<Pixel, PixelHash>& feature_set) {
  std::vector<Pixel> chain;
  chain.push_back(start);
  chain.push_back(first_step);
  std::unordered_set<Pixel, PixelHash> chain_set;
  chain_set.insert(start);
  chain_set.insert(first_step);

  Pixel prev = start;
  Pixel cur = first_step;
  const size_t max_steps =
      static_cast<size_t>(skeleton.width) * skeleton.height + 1;
  while (chain.size() <= max_steps) {
    // 当前点若是特征点（且不是起点本身），停止。
    if (!(cur == start) && feature_set.count(cur) > 0) {
      return chain;
    }
    // 中段：在非 prev 邻居中挑选主续方向（避免 8 邻域内侧角点误判分叉）。
    const std::vector<Pixel> nexts = NextPixels(skeleton, prev, cur);
    if (nexts.empty()) {
      return chain;  // 死胡同
    }
    const Pixel next = PickNextPixel(prev, cur, nexts, chain_set);
    if (next.col == -1) {
      return chain;
    }
    prev = cur;
    cur = next;
    chain.push_back(cur);
    chain_set.insert(cur);
    // 闭合回到起点：返回当前链（标记为环）
    if (cur == start) {
      return chain;
    }
  }
  return chain;
}

// ---- 投影与边拆分 -----------------------------------------------------------

// 在 edge.polyline（世界坐标）上找点 (px,py) 的最近投影点。
// 返回 (proj_x, proj_y, distance, segment_index, segment_t)
struct ProjectionResult {
  double px = 0.0;
  double py = 0.0;
  double distance = std::numeric_limits<double>::infinity();
  size_t segment_index = 0;  // 所在折线段索引（[i, i+1]）
  double segment_t = 0.0;    // 段内参数 [0,1]
};

ProjectionResult ProjectToPolyline(
    double px, double py,
    const std::vector<std::pair<double, double>>& polyline) {
  ProjectionResult best;
  if (polyline.size() < 2) {
    return best;
  }
  for (size_t i = 0; i + 1 < polyline.size(); ++i) {
    double t = 0.0;
    const auto& a = polyline[i];
    const auto& b = polyline[i + 1];
    const auto proj = ProjectPointToSegment(px, py, a.first, a.second, b.first,
                                            b.second, &t);
    if (proj.second < best.distance) {
      best.px = proj.first.first;
      best.py = proj.first.second;
      best.distance = proj.second;
      best.segment_index = i;
      best.segment_t = t;
    }
  }
  return best;
}

}  // namespace

TopologyBuildResult BuildTopologyCatalog(
    const BinaryGrid& skeleton, const MapGrid& map_info,
    const std::vector<AnchorStation>& anchors,
    const TopologyBuilderConfig& config) {
  TopologyBuildResult result;
  catalog::StationCatalog& catalog = result.catalog;
  catalog.map_hash = config.map_hash;
  catalog.graph_version = config.graph_version;

  if (skeleton.width == 0 || skeleton.height == 0) {
    result.error_message = "skeleton is empty";
    return result;
  }

  // ---- 第一步：提取特征点并分配 RoadNode id ----
  const std::vector<Pixel> feature_pixels = ExtractFeaturePixels(skeleton);
  std::unordered_set<Pixel, PixelHash> feature_set(feature_pixels.begin(),
                                                   feature_pixels.end());

  // 像素 -> RoadNode id 映射
  std::unordered_map<Pixel, std::string, PixelHash> pixel_to_node_id;
  std::unordered_map<Pixel, size_t, PixelHash> pixel_to_node_index;
  int next_node_idx = 0;
  for (const Pixel& p : feature_pixels) {
    const WorldCoord w =
        PixelToWorld(PixelCoord{static_cast<double>(p.col),
                                static_cast<double>(p.row)},
                     map_info);
    catalog::RoadNode node;
    node.id = "rn_" + std::to_string(p.col) + "_" + std::to_string(p.row);
    node.x = w.x;
    node.y = w.y;
    catalog.road_nodes.push_back(node);
    pixel_to_node_id[p] = node.id;
    pixel_to_node_index[p] = catalog.road_nodes.size() - 1;
    (void)next_node_idx;
    ++next_node_idx;
  }

  // ---- 第二步：从每个特征点的每个邻居方向追踪，提取像素边 ----
  // visited_directed_edge：(from_node, first_step_pixel) 标记该方向是否已追踪。
  // 同一条无向边 (A,m1,...,B) 会被两端各追踪一次，用 (from_node, first_step)
  // 去重——注意反向追踪时 first_step 是该边在 B 端的最后中间像素，与正向
  // 的 first_step 不同，因此需要额外检查边是否已存在。
  std::vector<PixelEdge> pixel_edges;
  std::unordered_set<Pixel, PixelHash> visited_middle;

  for (const Pixel& start : feature_pixels) {
    // 收集起点的所有骨架邻居
    std::vector<Pixel> neighbors;
    for (int i = 0; i < 8; ++i) {
      const Pixel nb{start.col + kDCol[i], start.row + kDRow[i]};
      if (skeleton.GetValue(nb.col, nb.row)) {
        neighbors.push_back(nb);
      }
    }
    for (const Pixel& first_step : neighbors) {
      // 若 first_step 已被访问且不是特征点，说明该方向上的边已被处理
      // （来自对端或同端的另一个起点方向）。
      if (feature_set.count(first_step) == 0 &&
          visited_middle.count(first_step) > 0) {
        continue;
      }
      // 检查边是否已被反向追踪（first_step 的对端像素对应）。
      // 简单做法：直接追踪，事后通过 (from_id,to_id,polyline) 三元组去重。
      std::vector<Pixel> chain =
          TraceFromToFeature(skeleton, start, first_step, feature_set);
      if (chain.size() < 2) {
        continue;
      }
      // 标记中间像素为已访问（特征点除外）。
      for (size_t i = 1; i + 1 < chain.size(); ++i) {
        visited_middle.insert(chain[i]);
      }
      PixelEdge pe;
      pe.from = chain.front();
      pe.to = chain.back();
      pe.polyline_px = chain;
      pixel_edges.push_back(std::move(pe));
    }
  }

  // 处理孤立闭合环：未被任何边覆盖的骨架像素（无任何特征点的纯环）。
  // 遍历所有骨架像素，若某像素既不在 visited_middle 也不是 feature，则
  // 说明它属于某个未被追踪的闭合环。在环上任取一点作为 RoadNode，并把
  // 整条环像素序列作为一条"自环"边（from=to=该节点）——但 stations.yaml
  // schema 不允许 from==to，改为在环上任取两个像素作为 RoadNode，环切成
  // 两段边。
  {
    std::unordered_set<Pixel, PixelHash> covered;
    for (const PixelEdge& e : pixel_edges) {
      for (const Pixel& p : e.polyline_px) {
        covered.insert(p);
      }
    }
    // 收集所有未被覆盖的骨架像素
    std::vector<Pixel> orphan;
    for (uint32_t row = 0; row < skeleton.height; ++row) {
      for (uint32_t col = 0; col < skeleton.width; ++col) {
        const Pixel p{static_cast<int>(col), static_cast<int>(row)};
        if (skeleton.GetValue(p.col, p.row) && covered.count(p) == 0) {
          orphan.push_back(p);
        }
      }
    }
    // BFS 分组各连通环
    std::unordered_set<Pixel, PixelHash> visited_orphan;
    for (const Pixel& seed : orphan) {
      if (visited_orphan.count(seed) > 0) {
        continue;
      }
      // BFS 收集本环所有像素
      std::vector<Pixel> ring;
      std::queue<Pixel> bfs;
      bfs.push(seed);
      visited_orphan.insert(seed);
      while (!bfs.empty()) {
        const Pixel cur = bfs.front();
        bfs.pop();
        ring.push_back(cur);
        for (int i = 0; i < 8; ++i) {
          const Pixel nb{cur.col + kDCol[i], cur.row + kDRow[i]};
          if (!skeleton.GetValue(nb.col, nb.row) ||
              visited_orphan.count(nb) > 0 ||
              feature_set.count(nb) > 0) {
            continue;
          }
          visited_orphan.insert(nb);
          bfs.push(nb);
        }
      }
      if (ring.size() < 3) {
        continue;  // 过短环不应出现，防御性跳过
      }
      // 在环上任取两个像素作为 RoadNode，环切成两段。
      // 简化：取 ring[0] 与 ring[ring.size()/2]。
      const Pixel n_a = ring.front();
      const Pixel n_b = ring[ring.size() / 2];
      auto add_node = [&](const Pixel& p) {
        if (pixel_to_node_id.count(p) > 0) {
          return;
        }
        const WorldCoord w =
            PixelToWorld(PixelCoord{static_cast<double>(p.col),
                                    static_cast<double>(p.row)},
                         map_info);
        catalog::RoadNode node;
        node.id = "rn_" + std::to_string(p.col) + "_" + std::to_string(p.row);
        node.x = w.x;
        node.y = w.y;
        catalog.road_nodes.push_back(node);
        pixel_to_node_id[p] = node.id;
        feature_set.insert(p);
      };
      add_node(n_a);
      add_node(n_b);
      // 在环上从 n_a 沿两个方向 BFS/追踪到 n_b。简化：直接追踪有序路径。
      // 构造环像素的相邻列表，然后从 n_a 走两条方向到 n_b。
      auto trace_on_ring = [&](const Pixel& from, const Pixel& first,
                               const Pixel& target) {
        std::vector<Pixel> path{from, first};
        Pixel prev = from;
        Pixel cur = first;
        const size_t limit = ring.size() + 8;
        while (!(cur == target) && path.size() <= limit) {
          // 选非 prev 的环邻居
          Pixel next{-1, -1};
          int found = 0;
          for (int i = 0; i < 8; ++i) {
            const Pixel nb{cur.col + kDCol[i], cur.row + kDRow[i]};
            if (nb == prev) continue;
            if (!skeleton.GetValue(nb.col, nb.row)) continue;
            if (std::find(ring.begin(), ring.end(), nb) == ring.end()) continue;
            next = nb;
            ++found;
          }
          if (found == 0) break;
          prev = cur;
          cur = next;
          path.push_back(cur);
        }
        return path;
      };
      // n_a 的两个环邻居
      std::vector<Pixel> a_nbrs;
      for (int i = 0; i < 8; ++i) {
        const Pixel nb{n_a.col + kDCol[i], n_a.row + kDRow[i]};
        if (skeleton.GetValue(nb.col, nb.row) &&
            std::find(ring.begin(), ring.end(), nb) != ring.end() &&
            !(nb == n_b)) {
          a_nbrs.push_back(nb);
        }
      }
      for (const Pixel& first : a_nbrs) {
        const std::vector<Pixel> path = trace_on_ring(n_a, first, n_b);
        if (path.size() >= 2 && path.back() == n_b) {
          PixelEdge pe;
          pe.from = n_a;
          pe.to = n_b;
          pe.polyline_px = path;
          pixel_edges.push_back(std::move(pe));
        }
      }
    }
  }

  // 边去重：(from_id, to_id, polyline_size) 三元组 + 检查双向重复。
  {
    std::vector<PixelEdge> unique_edges;
    for (const auto& e : pixel_edges) {
      bool dup = false;
      for (const auto& u : unique_edges) {
        if (e.from == u.from && e.to == u.to &&
            e.polyline_px.size() == u.polyline_px.size()) {
          dup = true;
          break;
        }
        // 反向同边（A->B 与 B->A 折线等长且首尾对应）
        if (e.from == u.to && e.to == u.from &&
            e.polyline_px.size() == u.polyline_px.size()) {
          // 验证首像素匹配（同一条几何链）
          if (e.polyline_px.front() == u.polyline_px.back() &&
              e.polyline_px.back() == u.polyline_px.front()) {
            dup = true;
            break;
          }
        }
      }
      if (!dup) {
        unique_edges.push_back(e);
      }
    }
    pixel_edges = std::move(unique_edges);
  }

  // ---- 第三步：转换为 CatalogStationEdge（世界坐标 polyline + 累加距离）----
  for (const PixelEdge& pe : pixel_edges) {
    const auto it_from = pixel_to_node_id.find(pe.from);
    const auto it_to = pixel_to_node_id.find(pe.to);
    if (it_from == pixel_to_node_id.end() || it_to == pixel_to_node_id.end()) {
      continue;  // 防御：未注册的端点跳过
    }
    catalog::CatalogStationEdge edge;
    edge.from = it_from->second;
    edge.to = it_to->second;
    edge.bidirectional = true;
    edge.enabled = true;
    edge.polyline.reserve(pe.polyline_px.size());
    for (const Pixel& p : pe.polyline_px) {
      const WorldCoord w =
          PixelToWorld(PixelCoord{static_cast<double>(p.col),
                                  static_cast<double>(p.row)},
                       map_info);
      edge.polyline.emplace_back(w.x, w.y);
    }
    edge.distance_m = PolylineLength(edge.polyline);
    catalog.edges.push_back(std::move(edge));
  }

  // ---- 第四步：站点吸附与边拆分 ----
  for (const AnchorStation& anchor : anchors) {
    if (catalog.edges.empty()) {
      result.error_message =
          "no edges available to snap station " + anchor.id;
      return result;
    }
    // 找到最近边及其投影点
    size_t best_edge_idx = std::numeric_limits<size_t>::max();
    ProjectionResult best_proj;
    for (size_t i = 0; i < catalog.edges.size(); ++i) {
      const auto& edge = catalog.edges[i];
      const ProjectionResult proj =
          ProjectToPolyline(anchor.x, anchor.y, edge.polyline);
      if (proj.distance < best_proj.distance) {
        best_proj = proj;
        best_edge_idx = i;
      }
    }
    if (best_edge_idx == std::numeric_limits<size_t>::max()) {
      result.error_message = "no valid edge for station " + anchor.id;
      return result;
    }
    if (best_proj.distance > config.max_snap_dist_m) {
      std::ostringstream oss;
      oss << "Station " << anchor.id
          << " is too far from skeleton: " << best_proj.distance << "m";
      result.error_message = oss.str();
      return result;
    }

    // 将站点作为图节点（id = anchor.id，世界坐标采用投影点使节点落在
    // 骨架上——这样 SkeletonPlanner 沿折线循迹时能精确到站点）。
    catalog::Station st;
    st.id = anchor.id;
    st.x = best_proj.px;
    st.y = best_proj.py;
    st.yaw = anchor.yaw;
    catalog.stations.push_back(st);

    // 拆分最近边：U -> V 变 U -> anchor -> V
    catalog::CatalogStationEdge original =
        catalog.edges[best_edge_idx];
    // 构造两段 polyline
    std::vector<std::pair<double, double>> poly_a;  // U -> proj
    std::vector<std::pair<double, double>> poly_b;  // proj -> V
    const size_t seg = best_proj.segment_index;
    const std::pair<double, double> proj_pt{best_proj.px, best_proj.py};
    // poly_a: polyline[0..seg] + proj_pt
    for (size_t i = 0; i <= seg; ++i) {
      poly_a.push_back(original.polyline[i]);
    }
    // 若 proj_pt 与末端点不同则补上（t<1 时需要）
    if (best_proj.segment_t < 1.0 - 1e-9 ||
        poly_a.back() != proj_pt) {
      if (std::hypot(poly_a.back().first - proj_pt.first,
                     poly_a.back().second - proj_pt.second) > 1e-9) {
        poly_a.push_back(proj_pt);
      }
    }
    // poly_b: proj_pt + polyline[seg+1..end]
    poly_b.push_back(proj_pt);
    for (size_t i = seg + 1; i < original.polyline.size(); ++i) {
      poly_b.push_back(original.polyline[i]);
    }
    // 清理 poly_b 起点：若 proj_pt 恰等于 polyline[seg+1] 则跳过重复
    if (poly_b.size() >= 2 &&
        std::hypot(poly_b[0].first - poly_b[1].first,
                   poly_b[0].second - poly_b[1].second) < 1e-9) {
      poly_b.erase(poly_b.begin());
    }

    // 移除原边，加入两段新边
    catalog.edges.erase(catalog.edges.begin() +
                        static_cast<long>(best_edge_idx));
    if (poly_a.size() >= 2) {
      catalog::CatalogStationEdge e1;
      e1.from = original.from;
      e1.to = anchor.id;
      e1.bidirectional = original.bidirectional;
      e1.enabled = original.enabled;
      e1.polyline = std::move(poly_a);
      e1.distance_m = PolylineLength(e1.polyline);
      catalog.edges.push_back(std::move(e1));
    } else {
      // 投影点恰是端点 U：站点直接替代 U，但 schema 要求 from!=to。
      // 防御：跳过该段（意味着 anchor 与 U 重合，anchor 站点应替代 U
      // 的节点身份——但我们已将 anchor 作为新站点，需要至少保留 anchor
      // 的接入）。
      // 此时若 poly_b 也不存在，则 anchor 成为孤立站点，连通性校验会失败。
      // 为容错，把 U 节点的所有其他边重定向到 anchor。
      for (auto& e : catalog.edges) {
        if (e.from == original.from) e.from = anchor.id;
        if (e.to == original.from) e.to = anchor.id;
      }
    }
    if (poly_b.size() >= 2) {
      catalog::CatalogStationEdge e2;
      e2.from = anchor.id;
      e2.to = original.to;
      e2.bidirectional = original.bidirectional;
      e2.enabled = original.enabled;
      e2.polyline = std::move(poly_b);
      e2.distance_m = PolylineLength(e2.polyline);
      catalog.edges.push_back(std::move(e2));
    } else {
      // 投影点恰是端点 V：同上重定向。
      for (auto& e : catalog.edges) {
        if (e.from == original.to) e.from = anchor.id;
        if (e.to == original.to) e.to = anchor.id;
      }
    }
  }

  // ---- 第五步：连通性校验 ----
  if (catalog.stations.size() >= 2) {
    const std::string& base = catalog.stations[0].id;
    for (size_t i = 1; i < catalog.stations.size(); ++i) {
      const std::string& target = catalog.stations[i].id;
      const auto path = catalog::FindRoutePath(catalog, base, target);
      if (!path.has_value()) {
        result.error_message =
            "Map network is disconnected: station " + target +
            " cannot reach " + base;
        return result;
      }
    }
  }

  // ---- 第六步：schema 校验 ----
  std::string validation_error;
  if (!catalog::ValidateStationCatalog(catalog, &validation_error)) {
    result.error_message =
        "generated catalog failed validation: " + validation_error;
    return result;
  }

  result.success = true;
  return result;
}

}  // namespace amr_dispatcher_core::planning
