#include "amr_dispatcher_core/planning/skeleton_extractor.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::planning {
namespace {

// 8 邻域方向（顺时针，自正北起）：N, NE, E, SE, S, SW, W, NW。
// Zhang-Suen 的 P2..P9 与剪枝的度数统计共用这一顺序。
constexpr int kDRow[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
constexpr int kDCol[8] = {0, 1, 1, 1, 0, -1, -1, -1};

// 受保护判定的欧式距离阈值（像素）：站点停靠位通常经正交投影落在骨架
// 附近，1.5 像素（默认 0.075m）容差足以吸收亚像素舍入。
constexpr double kProtectedDistancePx = 1.5;

// 骨架像素坐标（整数栅格）。
struct Pixel {
  int col = 0;
  int row = 0;
};

// 8 邻域内相邻骨架像素数，即剪枝算法中的"度数"。
int Degree(const BinaryGrid& grid, int col, int row) {
  int count = 0;
  for (int i = 0; i < 8; ++i) {
    if (grid.GetValue(col + kDCol[i], row + kDRow[i])) {
      ++count;
    }
  }
  return count;
}

// 8 邻域环上的连通组数（P2..P9,P2 环状序列中 0→1 跳变次数）：
//   0 = 无邻居，1 = 端点 / 单组，2 = 线段点，>= 3 = 分叉点。
// 与裸度数相比，组数不会把"紧邻正交交叉点的直线像素"（其度数也 >= 3）
// 误判为分叉，是像素骨架上更稳健的分叉判据。
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
    return 1;  // 完全被包围：单一连通组（防御分支，1 像素宽骨架不会出现）
  }
  int groups = 0;
  for (int i = 0; i < 8; ++i) {
    if (!neighbors[i] && neighbors[(i + 1) % 8]) {
      ++groups;
    }
  }
  return groups;
}

// 剪除分支后，停止点 S 的剩余邻居在 4 邻接（网格意义）下是否构成单一
// 连通组。成立时说明 S 仅因被剪掉的分支而呈现为"分叉"——本体是主线上的
// 冗余凸起像素（1 像素宽骨架的正交 T 接合处常见），若不一并剪除会在主干
// 上留下 1 像素毛刺伪影。真正的 Y / T / 十字分叉点剩余邻居彼此不相邻，
// 返回 false 得以保留。
bool NeighborsSingleFourConnected(const BinaryGrid& grid, int col, int row) {
  std::vector<Pixel> neighbors;
  for (int i = 0; i < 8; ++i) {
    if (grid.GetValue(col + kDCol[i], row + kDRow[i])) {
      neighbors.push_back({col + kDCol[i], row + kDRow[i]});
    }
  }
  if (neighbors.size() <= 1) {
    return true;
  }
  // 在邻居像素集合上做 4 邻接连通性 BFS。
  std::vector<uint8_t> visited(neighbors.size(), 0);
  std::vector<size_t> stack{0};
  visited[0] = 1;
  size_t reached = 1;
  while (!stack.empty()) {
    const size_t index = stack.back();
    stack.pop_back();
    for (size_t j = 0; j < neighbors.size(); ++j) {
      if (visited[j]) {
        continue;
      }
      const int dc = neighbors[index].col - neighbors[j].col;
      const int dr = neighbors[index].row - neighbors[j].row;
      if ((dc == 0 && (dr == 1 || dr == -1)) ||
          (dr == 0 && (dc == 1 || dc == -1))) {
        visited[j] = 1;
        stack.push_back(j);
        ++reached;
      }
    }
  }
  return reached == neighbors.size();
}

// 分支是否受保护：任一像素与某保护点欧式距离 < 1.5 像素即豁免。
bool IsProtectedBranch(const std::vector<Pixel>& branch,
                       const std::vector<PixelCoord>& protected_points) {
  for (const Pixel& pixel : branch) {
    for (const PixelCoord& point : protected_points) {
      const double d_col = static_cast<double>(pixel.col) - point.col;
      const double d_row = static_cast<double>(pixel.row) - point.row;
      if (d_col * d_col + d_row * d_row <
          kProtectedDistancePx * kProtectedDistancePx) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

BinaryGrid ZhangSuenThinning(const BinaryGrid& grid) {
  BinaryGrid result = grid;
  if (result.width == 0 || result.height == 0) {
    return result;
  }

  bool changed = true;
  while (changed) {
    changed = false;
    for (int step = 0; step < 2; ++step) {
      // 先收集本轮待删除像素，再统一置 0 —— 保证子步内所有判定基于同一
      // 快照（Zhang-Suen 的标准同步语义）。
      std::vector<Pixel> to_remove;
      for (uint32_t row = 0; row < result.height; ++row) {
        for (uint32_t col = 0; col < result.width; ++col) {
          const int c = static_cast<int>(col);
          const int r = static_cast<int>(row);
          if (!result.GetValue(c, r)) {
            continue;
          }
          // P2..P9（自正北顺时针），B 为邻域内骨架像素总数。
          int neighbors[8];
          int B = 0;
          for (int i = 0; i < 8; ++i) {
            neighbors[i] = result.GetValue(c + kDCol[i], r + kDRow[i]) ? 1 : 0;
            B += neighbors[i];
          }
          if (B < 2 || B > 6) {
            continue;  // 孤立点与骨架内部点均不删除
          }
          // A：P2..P9,P2 环状序列中 0→1 跳变次数，==1 保证不切断连通性。
          int A = 0;
          for (int i = 0; i < 8; ++i) {
            if (!neighbors[i] && neighbors[(i + 1) % 8]) {
              ++A;
            }
          }
          if (A != 1) {
            continue;
          }
          bool condition = false;
          if (step == 0) {
            // 子步 1：P2*P4*P6 == 0 且 P4*P6*P8 == 0
            condition = (neighbors[0] * neighbors[2] * neighbors[4] == 0) &&
                        (neighbors[2] * neighbors[4] * neighbors[6] == 0);
          } else {
            // 子步 2：P2*P4*P8 == 0 且 P2*P6*P8 == 0
            condition = (neighbors[0] * neighbors[2] * neighbors[6] == 0) &&
                        (neighbors[0] * neighbors[4] * neighbors[6] == 0);
          }
          if (condition) {
            to_remove.push_back({c, r});
          }
        }
      }
      if (!to_remove.empty()) {
        changed = true;
        for (const Pixel& pixel : to_remove) {
          result.SetValue(pixel.col, pixel.row, 0);
        }
      }
    }
  }
  return result;
}

BinaryGrid PruneSkeletonProtected(const BinaryGrid& skeleton,
                                  const std::vector<PixelCoord>& protected_points,
                                  const PruneConfig& config) {
  BinaryGrid result = skeleton;
  if (result.width == 0 || result.height == 0) {
    return result;
  }
  const double resolution = config.resolution > 0.0 ? config.resolution : 0.05;

  bool changed = true;
  while (changed) {
    changed = false;
    // 每轮重新扫描端点：上一轮剪除会改变度数，端点集合随之变化。
    for (uint32_t row = 0; row < result.height; ++row) {
      for (uint32_t col = 0; col < result.width; ++col) {
        const int start_col = static_cast<int>(col);
        const int start_row = static_cast<int>(row);
        // 端点在处理时即时复核：同一轮里它可能已被先前的剪除删除。
        if (!result.GetValue(start_col, start_row) ||
            Degree(result, start_col, start_row) != 1) {
          continue;
        }

        // 从端点向内追踪分支：visited 记录分支像素（端点起，不含停止点）。
        std::vector<Pixel> visited;
        visited.push_back({start_col, start_row});
        int prev_col = start_col;
        int prev_row = start_row;
        // 端点唯一的骨架邻居即为追踪方向。
        int cur_col = -1;
        int cur_row = -1;
        for (int i = 0; i < 8; ++i) {
          const int ncol = start_col + kDCol[i];
          const int nrow = start_row + kDRow[i];
          if (result.GetValue(ncol, nrow)) {
            cur_col = ncol;
            cur_row = nrow;
            break;
          }
        }

        bool reached_junction = false;
        bool reached_other_end = false;
        const size_t max_steps =
            static_cast<size_t>(result.width) * result.height + 1;
        while (visited.size() <= max_steps) {
          // 分叉判据用连通组数而非裸度数：紧邻正交交叉点的直线像素（如 T
          // 接合处主干上的 (15,3)）8 邻域裸度数也 >= 3，若按裸度数判定
          // 会在剪除后于主干上残留 1 像素凸起伪影。
          const int crossings = CrossingNumber(result, cur_col, cur_row);
          if (crossings >= 3) {
            reached_junction = true;
            break;
          }
          if (crossings <= 1) {
            // 追踪到另一个端点：整条线段无分叉锚点（例如孤立短线段），
            // 不属于"死胡同毛刺"，跳过不剪，避免误删整个连通分量。
            reached_other_end = true;
            break;
          }
          // 连通组数 == 2：分支中段，继续沿非来向邻居推进。
          visited.push_back({cur_col, cur_row});
          for (int i = 0; i < 8; ++i) {
            const int ncol = cur_col + kDCol[i];
            const int nrow = cur_row + kDRow[i];
            if ((ncol != prev_col || nrow != prev_row) &&
                result.GetValue(ncol, nrow)) {
              prev_col = cur_col;
              prev_row = cur_row;
              cur_col = ncol;
              cur_row = nrow;
              break;
            }
          }
        }
        if (!reached_junction || reached_other_end) {
          continue;
        }

        const double branch_length_m =
            static_cast<double>(visited.size()) * resolution;
        if (!(branch_length_m < config.min_branch_length_m)) {
          continue;  // 长分支保留
        }
        // 受保护分支（承载站点）强制豁免，不可剪除。
        if (IsProtectedBranch(visited, protected_points)) {
          continue;
        }
        // 剪除短毛刺：分支像素全部置 0，分叉点保留。
        for (const Pixel& pixel : visited) {
          result.SetValue(pixel.col, pixel.row, 0);
        }
        // 冗余停止点清理：T 接合处，停止点 S 常因被剪分支才呈现为"分叉"
        // （其邻居经 4 邻接连通后实为主干线的一段）。分支剪除后 S 的剩余
        // 邻居若构成单一连通组，S 即为主干上的冗余凸起，一并剪除以免留
        // 下 1 像素伪影；真正的 Y / T / 十字分叉点剩余邻居彼此不相邻，
        // 得以保留。受保护点附近的停止点同样豁免。
        if (!IsProtectedBranch({{cur_col, cur_row}}, protected_points) &&
            NeighborsSingleFourConnected(result, cur_col, cur_row)) {
          result.SetValue(cur_col, cur_row, 0);
        }
        changed = true;
      }
    }
  }
  return result;
}

}  // namespace amr_dispatcher_core::planning
