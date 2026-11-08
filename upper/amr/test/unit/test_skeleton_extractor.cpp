#include "amr_dispatcher_core/planning/skeleton_extractor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::planning {
namespace {

constexpr double kResolution = 0.05;  // 米/像素

BinaryGrid MakeGrid(uint32_t width, uint32_t height) {
  BinaryGrid grid;
  grid.width = width;
  grid.height = height;
  grid.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
  return grid;
}

// 将 [col0, col1] × [row0, row1] 矩形区域置为可通行 (1)。
void FillRect(BinaryGrid& grid, int col0, int row0, int col1, int row1) {
  for (int row = row0; row <= row1; ++row) {
    for (int col = col0; col <= col1; ++col) {
      grid.SetValue(col, row, 1);
    }
  }
}

int CountSkeleton(const BinaryGrid& grid) {
  int count = 0;
  for (uint32_t row = 0; row < grid.height; ++row) {
    for (uint32_t col = 0; col < grid.width; ++col) {
      if (grid.GetValue(static_cast<int>(col), static_cast<int>(row))) {
        ++count;
      }
    }
  }
  return count;
}

// 8 连通连通分量个数（用于校验骨架连续不断裂）。
int CountComponents(const BinaryGrid& grid) {
  std::vector<uint8_t> visited(grid.data.size(), 0);
  int components = 0;
  for (uint32_t start_row = 0; start_row < grid.height; ++start_row) {
    for (uint32_t start_col = 0; start_col < grid.width; ++start_col) {
      const size_t start_index =
          static_cast<size_t>(start_row) * grid.width + start_col;
      if (!grid.GetValue(static_cast<int>(start_col),
                         static_cast<int>(start_row)) ||
          visited[start_index]) {
        continue;
      }
      ++components;
      std::queue<std::pair<int, int>> bfs;
      bfs.emplace(static_cast<int>(start_col), static_cast<int>(start_row));
      visited[start_index] = 1;
      while (!bfs.empty()) {
        const auto [col, row] = bfs.front();
        bfs.pop();
        for (int drow = -1; drow <= 1; ++drow) {
          for (int dcol = -1; dcol <= 1; ++dcol) {
            if (drow == 0 && dcol == 0) {
              continue;
            }
            const int ncol = col + dcol;
            const int nrow = row + drow;
            if (!grid.IsValidPixel(ncol, nrow) ||
                !grid.GetValue(ncol, nrow)) {
              continue;
            }
            const size_t index =
                static_cast<size_t>(nrow) * grid.width +
                static_cast<size_t>(ncol);
            if (visited[index]) {
              continue;
            }
            visited[index] = 1;
            bfs.emplace(ncol, nrow);
          }
        }
      }
    }
  }
  return components;
}

// 是否存在 2×2 全为骨架像素的实心块（1 像素宽骨架不应出现）。
bool Has2x2Block(const BinaryGrid& grid) {
  if (grid.width < 2 || grid.height < 2) {
    return false;
  }
  for (uint32_t row = 0; row + 1 < grid.height; ++row) {
    for (uint32_t col = 0; col + 1 < grid.width; ++col) {
      const int c = static_cast<int>(col);
      const int r = static_cast<int>(row);
      if (grid.GetValue(c, r) && grid.GetValue(c + 1, r) &&
          grid.GetValue(c, r + 1) && grid.GetValue(c + 1, r + 1)) {
        return true;
      }
    }
  }
  return false;
}

// 细化只允许移除像素：输出为输入的子集。
bool IsSubset(const BinaryGrid& candidate, const BinaryGrid& source) {
  if (candidate.width != source.width ||
      candidate.height != source.height) {
    return false;
  }
  for (uint32_t row = 0; row < source.height; ++row) {
    for (uint32_t col = 0; col < source.width; ++col) {
      const int c = static_cast<int>(col);
      const int r = static_cast<int>(row);
      if (candidate.GetValue(c, r) && !source.GetValue(c, r)) {
        return false;
      }
    }
  }
  return true;
}

// 8 邻域内相邻骨架像素数（即剪枝算法中的"度数"）。
int Degree(const BinaryGrid& grid, int col, int row) {
  int count = 0;
  for (int drow = -1; drow <= 1; ++drow) {
    for (int dcol = -1; dcol <= 1; ++dcol) {
      if (drow == 0 && dcol == 0) {
        continue;
      }
      if (grid.GetValue(col + dcol, row + drow)) {
        ++count;
      }
    }
  }
  return count;
}

// ---------------------------------------------------------------------------
// 用例 1: 细化提取直走廊 —— 5 像素宽长走廊细化为 1 像素宽居中骨架线
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, StraightCorridor_ThinsToCenteredOnePixelLine) {
  BinaryGrid corridor = MakeGrid(20, 5);
  FillRect(corridor, 0, 0, 19, 4);  // 5 像素宽、20 像素长的直走廊

  const BinaryGrid skeleton = ZhangSuenThinning(corridor);

  ASSERT_EQ(skeleton.width, 20u);
  ASSERT_EQ(skeleton.height, 5u);
  EXPECT_TRUE(IsSubset(skeleton, corridor));

  // 骨架是一条连通的单像素线，不存在 2×2 实心块。
  EXPECT_EQ(CountComponents(skeleton), 1);
  EXPECT_FALSE(Has2x2Block(skeleton));

  // 骨架居中：全部像素位于中心行 row=2 附近（上下各留 1 像素余量）。
  int min_row = 99;
  int max_row = -1;
  int min_col = 99;
  int max_col = -1;
  int count = 0;
  for (uint32_t row = 0; row < skeleton.height; ++row) {
    for (uint32_t col = 0; col < skeleton.width; ++col) {
      if (!skeleton.GetValue(static_cast<int>(col),
                             static_cast<int>(row))) {
        continue;
      }
      ++count;
      min_row = std::min(min_row, static_cast<int>(row));
      max_row = std::max(max_row, static_cast<int>(row));
      min_col = std::min(min_col, static_cast<int>(col));
      max_col = std::max(max_col, static_cast<int>(col));
    }
  }
  EXPECT_GE(min_row, 1);
  EXPECT_LE(max_row, 3);
  EXPECT_EQ(max_row - min_row, 0) << "骨架应为 1 像素宽（单行）";

  // 长度保持：横向跨度与像素数一致（连续无断点）。
  // 注：经典 Zhang-Suen 会修剪自由端的端帽像素（20 像素长走廊收敛为
  // 15 像素骨架线），这里按 >=70% 原长度校验。
  EXPECT_EQ(count, max_col - min_col + 1);
  EXPECT_GE(count, 14);
  EXPECT_LE(count, 20);
  for (int col = min_col; col <= max_col; ++col) {
    int column_sum = 0;
    for (uint32_t row = 0; row < skeleton.height; ++row) {
      column_sum += skeleton.GetValue(col, static_cast<int>(row)) ? 1 : 0;
    }
    EXPECT_EQ(column_sum, 1) << "col=" << col;
  }
}

// ---------------------------------------------------------------------------
// 用例 2: L 弯走廊骨架提取 —— 直角弯道细化为连续的 1 像素中心折线
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, LCornerCorridor_ContinuousRightAngleCenterline) {
  BinaryGrid l_shape = MakeGrid(20, 20);
  FillRect(l_shape, 0, 0, 19, 4);  // 水平臂：5 像素宽 × 20 像素长
  FillRect(l_shape, 0, 0, 4, 19);  // 垂直臂：5 像素宽 × 20 像素长

  const BinaryGrid skeleton = ZhangSuenThinning(l_shape);

  EXPECT_TRUE(IsSubset(skeleton, l_shape));
  EXPECT_EQ(CountComponents(skeleton), 1) << "骨架必须连通";
  EXPECT_FALSE(Has2x2Block(skeleton)) << "骨架必须为 1 像素宽";

  // 水平中心线（row=2）延伸至右端附近。
  bool horizontal_far = false;
  for (int col = 16; col <= 19; ++col) {
    if (skeleton.GetValue(col, 2)) {
      horizontal_far = true;
    }
  }
  EXPECT_TRUE(horizontal_far) << "水平臂中心线应延伸到右端";

  // 垂直中心线（col=2）延伸至底端附近。
  bool vertical_far = false;
  for (int row = 16; row <= 19; ++row) {
    if (skeleton.GetValue(2, row)) {
      vertical_far = true;
    }
  }
  EXPECT_TRUE(vertical_far) << "垂直臂中心线应延伸到底端";

  // 直角拐点区域存在骨架像素，两臂在拐角处连为一体。
  bool corner_connected = false;
  for (int row = 1; row <= 3; ++row) {
    for (int col = 1; col <= 3; ++col) {
      if (skeleton.GetValue(col, row)) {
        corner_connected = true;
      }
    }
  }
  EXPECT_TRUE(corner_connected) << "直角拐点处应有骨架像素";

  const int count = CountSkeleton(skeleton);
  EXPECT_GE(count, 25);
  EXPECT_LE(count, 45);
}

// ---------------------------------------------------------------------------
// 用例 3: 十字路口骨架提取 —— 保留中心交叉点与 4 个对称分支
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, CrossJunction_CenterJunctionWithFourBranches) {
  BinaryGrid cross = MakeGrid(15, 15);
  FillRect(cross, 0, 5, 14, 9);  // 水平带：row 5..9
  FillRect(cross, 5, 0, 9, 14);  // 垂直带：col 5..9

  const BinaryGrid skeleton = ZhangSuenThinning(cross);

  EXPECT_TRUE(IsSubset(skeleton, cross));
  EXPECT_EQ(CountComponents(skeleton), 1);
  EXPECT_FALSE(Has2x2Block(skeleton));

  // 中心交叉点保留：(7,7) 是两臂中心线的交点。
  EXPECT_EQ(skeleton.GetValue(7, 7), 1);

  // 分叉点（度数 >= 3）聚簇在中心交叉区域，端点（度数 == 1）恰好 4 个。
  // 注：8 邻域下正交十字交叉处，中心点及其上下左右紧邻像素都会呈现
  // 度数 >= 3，因此分叉像素是一小簇而非单点——全部落在中心 3×3 内。
  int endpoints = 0;
  int junctions = 0;
  for (uint32_t row = 0; row < skeleton.height; ++row) {
    for (uint32_t col = 0; col < skeleton.width; ++col) {
      if (!skeleton.GetValue(static_cast<int>(col),
                             static_cast<int>(row))) {
        continue;
      }
      const int degree =
          Degree(skeleton, static_cast<int>(col), static_cast<int>(row));
      if (degree == 1) {
        ++endpoints;
      } else if (degree >= 3) {
        ++junctions;
        EXPECT_GE(col, 6u) << "分叉像素应聚簇于中心";
        EXPECT_LE(col, 8u) << "分叉像素应聚簇于中心";
        EXPECT_GE(row, 6u) << "分叉像素应聚簇于中心";
        EXPECT_LE(row, 8u) << "分叉像素应聚簇于中心";
      }
    }
  }
  EXPECT_GE(junctions, 1) << "中心应存在分叉点";
  EXPECT_EQ(endpoints, 4) << "4 个对称分支各有一个端点";

  // 4 个分支均自中心 (7,7) 向四个方向延伸。
  bool north = false;
  bool south = false;
  bool east = false;
  bool west = false;
  for (int row = 0; row <= 14; ++row) {
    if (skeleton.GetValue(7, row) && row <= 2) {
      north = true;
    }
    if (skeleton.GetValue(7, row) && row >= 9) {
      south = true;
    }
  }
  for (int col = 0; col <= 14; ++col) {
    if (skeleton.GetValue(col, 7) && col <= 2) {
      west = true;
    }
    if (skeleton.GetValue(col, 7) && col >= 10) {
      east = true;
    }
  }
  EXPECT_TRUE(north);
  EXPECT_TRUE(south);
  EXPECT_TRUE(east);
  EXPECT_TRUE(west);
}

// ---------------------------------------------------------------------------
// 公共测试场景：主干走廊 + 短毛刺死胡同
//   主干：row 0..4 × col 0..29（细化后中心线 row=2，col 2..26）
//   毛刺：单像素列 col=15, row=5..7 —— 细化后分支为 (15,7)→(15,3)
//         共 5 像素（0.25m < 0.5m 阈值）
// ---------------------------------------------------------------------------

BinaryGrid MakeCorridorWithBristle() {
  BinaryGrid grid = MakeGrid(30, 16);
  FillRect(grid, 0, 0, 29, 4);
  FillRect(grid, 15, 5, 15, 7);  // 短毛刺死胡同（细化后 5 像素分支）
  return grid;
}

// ---------------------------------------------------------------------------
// 用例 4: 短毛刺无保护剪除 —— 无保护点时毛刺被清除，主干道完好
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, ShortBristleUnprotected_PrunedMainLineIntact) {
  const BinaryGrid corridor = MakeCorridorWithBristle();
  const BinaryGrid skeleton = ZhangSuenThinning(corridor);

  // 细化后毛刺分支确实存在：(15,3)..(15,7) 共 5 个像素挂在主干上。
  ASSERT_EQ(skeleton.GetValue(15, 7), 1);
  ASSERT_EQ(skeleton.GetValue(15, 6), 1);
  ASSERT_EQ(skeleton.GetValue(15, 5), 1);
  ASSERT_EQ(skeleton.GetValue(15, 4), 1);
  ASSERT_EQ(skeleton.GetValue(15, 3), 1);
  ASSERT_EQ(skeleton.GetValue(15, 2), 1) << "分叉点位于主干中心线";
  const int before = CountSkeleton(skeleton);

  PruneConfig config;  // min_branch_length_m = 0.5, resolution = 0.05
  const BinaryGrid pruned =
      PruneSkeletonProtected(skeleton, {}, config);

  // 短毛刺（0.25m < 0.5m，无保护）被完整剪除。
  EXPECT_EQ(pruned.GetValue(15, 7), 0);
  EXPECT_EQ(pruned.GetValue(15, 6), 0);
  EXPECT_EQ(pruned.GetValue(15, 5), 0);
  EXPECT_EQ(pruned.GetValue(15, 4), 0);
  EXPECT_EQ(pruned.GetValue(15, 3), 0);
  EXPECT_EQ(CountSkeleton(pruned), before - 5) << "只应剪除毛刺的 5 个像素";

  // 主干道完好：分叉点并入主干，仍为单一连通骨架。
  EXPECT_EQ(pruned.GetValue(15, 2), 1);
  EXPECT_EQ(CountComponents(pruned), 1);
  int main_line_count = 0;
  for (int col = 0; col < 30; ++col) {
    main_line_count += pruned.GetValue(col, 2) ? 1 : 0;
  }
  EXPECT_GE(main_line_count, 25) << "主干中心线应基本完好";
}

// ---------------------------------------------------------------------------
// 用例 5: 保护点豁免剪除 —— 站点位于短毛刺末端时，分支强制完整保留
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, ShortBristleWithProtectedPoint_ExemptFromPruning) {
  const BinaryGrid corridor = MakeCorridorWithBristle();
  const BinaryGrid skeleton = ZhangSuenThinning(corridor);
  ASSERT_EQ(skeleton.GetValue(15, 7), 1);
  ASSERT_EQ(skeleton.GetValue(15, 6), 1);
  ASSERT_EQ(skeleton.GetValue(15, 5), 1);
  ASSERT_EQ(skeleton.GetValue(15, 4), 1);
  ASSERT_EQ(skeleton.GetValue(15, 3), 1);
  const int before = CountSkeleton(skeleton);

  // 站点停靠位位于毛刺末端（模拟小死胡同内的停靠点）。
  const std::vector<PixelCoord> protected_points{{15.0, 7.0}};
  PruneConfig config;
  const BinaryGrid pruned =
      PruneSkeletonProtected(skeleton, protected_points, config);

  // 受保护的短分支强制豁免，完整保留。
  EXPECT_EQ(pruned.GetValue(15, 7), 1);
  EXPECT_EQ(pruned.GetValue(15, 6), 1);
  EXPECT_EQ(pruned.GetValue(15, 5), 1);
  EXPECT_EQ(pruned.GetValue(15, 4), 1);
  EXPECT_EQ(pruned.GetValue(15, 3), 1);
  EXPECT_EQ(CountSkeleton(pruned), before) << "受保护分支不应被剪除任何像素";
}

// ---------------------------------------------------------------------------
// 用例 6: 长分支不被误剪 —— 1.0m (> 0.5m) 的死胡同即使无保护点也保留
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, LongDeadEndBranch_NotPruned) {
  BinaryGrid grid = MakeGrid(30, 30);
  FillRect(grid, 0, 0, 29, 4);      // 主干走廊
  FillRect(grid, 15, 5, 15, 22);    // 长毛刺：细化后分支约 20 像素 = 1.0m

  const BinaryGrid skeleton = ZhangSuenThinning(grid);
  ASSERT_EQ(skeleton.GetValue(15, 22), 1) << "长毛刺末端应保留在骨架上";
  ASSERT_EQ(skeleton.GetValue(15, 10), 1) << "长毛刺中段应保留在骨架上";
  const int before = CountSkeleton(skeleton);

  PruneConfig config;
  const BinaryGrid pruned = PruneSkeletonProtected(skeleton, {}, config);

  // 1.0m > 0.5m：长分支不被剪除。
  EXPECT_EQ(CountSkeleton(pruned), before);
  EXPECT_EQ(pruned.GetValue(15, 22), 1);
  EXPECT_EQ(pruned.GetValue(15, 10), 1);
  EXPECT_EQ(pruned.GetValue(15, 5), 1);
  EXPECT_EQ(CountComponents(pruned), 1);
}

// ---------------------------------------------------------------------------
// 用例 7 (C1 回归): 正交 2 像素毛刺紧邻主线 —— 追踪不得斜向逃逸到主线
//   主线：row 6, col 0..29（1 像素宽）
//   毛刺：(20,5)、(20,4) 垂直悬挂；基座像素 (20,5) 与主线像素 (19,6)、
//         (21,6) 8 邻接。旧实现按固定环序先考察 SE=(21,6) 再考察 S=
//         (20,6)，追踪从毛刺横向逃逸到主线并沿主线走到端点，毛刺漏剪。
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, OrthogonalTwoPixelBristle_PrunedNoLateralEscape) {
  BinaryGrid skeleton = MakeGrid(30, 12);
  for (int col = 0; col < 30; ++col) {
    skeleton.SetValue(col, 6, 1);  // 主干：row 6, col 0..29
  }
  skeleton.SetValue(20, 5, 1);
  skeleton.SetValue(20, 4, 1);  // 2 像素垂直毛刺

  PruneConfig config;
  const BinaryGrid pruned = PruneSkeletonProtected(skeleton, {}, config);

  // 毛刺被剪除。
  EXPECT_EQ(pruned.GetValue(20, 4), 0);
  EXPECT_EQ(pruned.GetValue(20, 5), 0);
  // 主线 30 像素全部完好（含原分叉点 (20,6)）。
  for (int col = 0; col < 30; ++col) {
    EXPECT_EQ(pruned.GetValue(col, 6), 1) << "col=" << col;
  }
  EXPECT_EQ(CountSkeleton(pruned), 30);
  EXPECT_EQ(CountComponents(pruned), 1);
}

// ---------------------------------------------------------------------------
// 用例 8 (C1 回归): 方环 + 两个对称水平短毛刺 —— 毛刺剪除且环无缺口
//   环：cols 2..6 × rows 2..6 的 1 像素外框（16 像素）
//   毛刺：左侧 (0,4)、(1,4)；右侧 (7,4)、(8,4)
//   旧实现从毛刺横向逃逸上环后沿环追踪至另一分叉点，visited 误吞环像素
//   并将其整段剪除（环出现缺口）。
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, RingWithTwoSymmetricSpurs_BothPrunedRingIntact) {
  BinaryGrid skeleton = MakeGrid(10, 10);
  for (int col = 2; col <= 6; ++col) {
    skeleton.SetValue(col, 2, 1);  // 上边
    skeleton.SetValue(col, 6, 1);  // 下边
  }
  for (int row = 3; row <= 5; ++row) {
    skeleton.SetValue(2, row, 1);  // 左边
    skeleton.SetValue(6, row, 1);  // 右边
  }
  skeleton.SetValue(0, 4, 1);
  skeleton.SetValue(1, 4, 1);  // 左毛刺
  skeleton.SetValue(7, 4, 1);
  skeleton.SetValue(8, 4, 1);  // 右毛刺

  PruneConfig config;
  const BinaryGrid pruned = PruneSkeletonProtected(skeleton, {}, config);

  // 两个毛刺均被剪除。
  EXPECT_EQ(pruned.GetValue(0, 4), 0);
  EXPECT_EQ(pruned.GetValue(1, 4), 0);
  EXPECT_EQ(pruned.GetValue(7, 4), 0);
  EXPECT_EQ(pruned.GetValue(8, 4), 0);
  // 环完整无缺口：16 个环像素全部保留。
  int ring_count = 0;
  for (int col = 2; col <= 6; ++col) {
    ring_count += pruned.GetValue(col, 2) ? 1 : 0;
    ring_count += pruned.GetValue(col, 6) ? 1 : 0;
  }
  for (int row = 3; row <= 5; ++row) {
    ring_count += pruned.GetValue(2, row) ? 1 : 0;
    ring_count += pruned.GetValue(6, row) ? 1 : 0;
  }
  EXPECT_EQ(ring_count, 16) << "环上不允许出现缺口";
  EXPECT_EQ(CountSkeleton(pruned), 16);
  EXPECT_EQ(CountComponents(pruned), 1);
}

// ---------------------------------------------------------------------------
// 用例 9 (C1 回归): 阶梯对角毛刺 —— 命中分叉候选立即停止追踪
//   主线：row 6, col 0..29
//   毛刺：阶梯 (20,5)、(19,4)、(18,3)。基座像素 (20,5) 的非来向候选中，
//         S=(20,6) 是分叉点而 SE=(21,6) 是直线方向上的主线像素——纯方向
//         选择会沿 SE 逃逸上主线，必须优先命中分叉候选。
// ---------------------------------------------------------------------------

TEST(SkeletonExtractorTest, DiagonalStaircaseBristle_PrunedMainLineIntact) {
  BinaryGrid skeleton = MakeGrid(30, 12);
  for (int col = 0; col < 30; ++col) {
    skeleton.SetValue(col, 6, 1);  // 主干：row 6, col 0..29
  }
  skeleton.SetValue(20, 5, 1);
  skeleton.SetValue(19, 4, 1);
  skeleton.SetValue(18, 3, 1);  // 3 像素阶梯对角毛刺

  PruneConfig config;
  const BinaryGrid pruned = PruneSkeletonProtected(skeleton, {}, config);

  // 阶梯毛刺被完整剪除。
  EXPECT_EQ(pruned.GetValue(18, 3), 0);
  EXPECT_EQ(pruned.GetValue(19, 4), 0);
  EXPECT_EQ(pruned.GetValue(20, 5), 0);
  // 主线完好。
  for (int col = 0; col < 30; ++col) {
    EXPECT_EQ(pruned.GetValue(col, 6), 1) << "col=" << col;
  }
  EXPECT_EQ(CountSkeleton(pruned), 30);
  EXPECT_EQ(CountComponents(pruned), 1);
}

}  // namespace
}  // namespace amr_dispatcher_core::planning
