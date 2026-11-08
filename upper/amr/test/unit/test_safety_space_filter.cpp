#include "amr_dispatcher_core/planning/safety_space_filter.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace amr_dispatcher_core::planning {
namespace {

MapGrid MakeGrid(uint32_t width, uint32_t height, double resolution) {
  MapGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height),
                   kCellFree);
  return grid;
}

// ---------------------------------------------------------------------------
// 用例 1: 全空旷地图 —— 无障碍时全部保留为可通行 (1)
// ---------------------------------------------------------------------------

TEST(SafetySpaceFilterTest, EmptyMap_AllCellsRemainPassable) {
  const MapGrid grid = MakeGrid(20, 20, 0.1);
  const SafetyFilterConfig config;  // R = 0.25 + 0.05 = 0.3m

  const BinaryGrid result = FilterSafetySpace(grid, config);

  ASSERT_EQ(result.width, grid.width);
  ASSERT_EQ(result.height, grid.height);
  ASSERT_EQ(result.data.size(), grid.data.size());
  for (uint32_t row = 0; row < grid.height; ++row) {
    for (uint32_t col = 0; col < grid.width; ++col) {
      EXPECT_EQ(result.GetValue(static_cast<int>(col), static_cast<int>(row)),
                1)
          << "col=" << col << " row=" << row;
    }
  }
}

// ---------------------------------------------------------------------------
// 用例 2: 单个障碍点膨胀 —— 欧氏距离 <= R 的圆内全为 0，圆外仍为 1
// res = 0.1m, R = 0.3m, r_pix = 3
// ---------------------------------------------------------------------------

TEST(SafetySpaceFilterTest, SingleObstacle_InflatesCircularRegion) {
  MapGrid grid = MakeGrid(21, 21, 0.1);
  const int center = 10;
  grid.SetValue(center, center, kCellOccupied);

  const SafetyFilterConfig config;  // R = 0.3m

  const BinaryGrid result = FilterSafetySpace(grid, config);

  // 圆内（含边界）全部不可通行，圆外保持可通行。
  for (int dy = -3; dy <= 3; ++dy) {
    for (int dx = -3; dx <= 3; ++dx) {
      const double dist_m = std::hypot(dx * grid.resolution, dy * grid.resolution);
      if (dist_m <= 0.3 + 1e-9) {
        EXPECT_EQ(result.GetValue(center + dx, center + dy), 0)
            << "dx=" << dx << " dy=" << dy << " dist=" << dist_m;
      } else {
        EXPECT_EQ(result.GetValue(center + dx, center + dy), 1)
            << "dx=" << dx << " dy=" << dy << " dist=" << dist_m;
      }
    }
  }

  // 显式抽查几个关键点。
  EXPECT_EQ(result.GetValue(center, center), 0);      // 障碍自身
  EXPECT_EQ(result.GetValue(center + 3, center), 0);  // 距离 0.3m（边界，含）
  EXPECT_EQ(result.GetValue(center + 2, center + 2),
            0);  // 距离 0.2*sqrt(2)≈0.283m（含）
  EXPECT_EQ(result.GetValue(center + 3, center + 1),
            1);  // 距离 0.1*sqrt(10)≈0.316m（不含）
  EXPECT_EQ(result.GetValue(center + 4, center), 1);  // 距离 0.4m（不含）
}

// ---------------------------------------------------------------------------
// 用例 3: 狭窄走廊被阻断 —— 0.4m 宽走廊 < 车体需求 0.6m，中心线全部腐蚀
// ---------------------------------------------------------------------------

TEST(SafetySpaceFilterTest, NarrowCorridor_BlockedCompletely) {
  // 列 0 与列 5 为墙，列 1-4 为 0.4m 宽自由走廊（res=0.1m）。
  MapGrid grid = MakeGrid(20, 10, 0.1);
  for (uint32_t row = 0; row < grid.height; ++row) {
    grid.SetValue(0, static_cast<int>(row), kCellOccupied);
    grid.SetValue(5, static_cast<int>(row), kCellOccupied);
  }

  const SafetyFilterConfig config;  // R = 0.3m，需要至少 0.6m 净宽

  const BinaryGrid result = FilterSafetySpace(grid, config);

  for (uint32_t row = 0; row < grid.height; ++row) {
    for (int col = 1; col <= 4; ++col) {
      EXPECT_EQ(result.GetValue(col, static_cast<int>(row)), 0)
          << "col=" << col << " row=" << row;
    }
  }
}

// ---------------------------------------------------------------------------
// 用例 4: 宽走廊保持连通 —— 1.0m 宽走廊 > 车体需求 0.6m，中心部分保留
// ---------------------------------------------------------------------------

TEST(SafetySpaceFilterTest, WideCorridor_CenterRemainsPassable) {
  // 列 0 与列 11 为墙，列 1-10 为 1.0m 宽自由走廊（res=0.1m）。
  MapGrid grid = MakeGrid(20, 10, 0.1);
  for (uint32_t row = 0; row < grid.height; ++row) {
    grid.SetValue(0, static_cast<int>(row), kCellOccupied);
    grid.SetValue(11, static_cast<int>(row), kCellOccupied);
  }

  const SafetyFilterConfig config;  // R = 0.3m

  const BinaryGrid result = FilterSafetySpace(grid, config);

  for (uint32_t row = 0; row < grid.height; ++row) {
    // 走廊中心（列 5、6，距两侧墙均 >= 0.5m）保持可通行。
    EXPECT_EQ(result.GetValue(5, static_cast<int>(row)), 1) << "row=" << row;
    EXPECT_EQ(result.GetValue(6, static_cast<int>(row)), 1) << "row=" << row;
    // 紧邻墙侧（列 1-3，距墙 <= 0.3m）被腐蚀。
    EXPECT_EQ(result.GetValue(1, static_cast<int>(row)), 0) << "row=" << row;
    EXPECT_EQ(result.GetValue(2, static_cast<int>(row)), 0) << "row=" << row;
    EXPECT_EQ(result.GetValue(3, static_cast<int>(row)), 0) << "row=" << row;
  }
}

// ---------------------------------------------------------------------------
// 用例 5: 未知区域处理 —— treat_unknown_as_obstacle=true 时按障碍膨胀
// ---------------------------------------------------------------------------

TEST(SafetySpaceFilterTest, UnknownRegion_TreatedAsObstacleAndInflated) {
  MapGrid grid = MakeGrid(20, 20, 0.1);
  // 中央 3x3 未知区域。
  for (int row = 4; row <= 6; ++row) {
    for (int col = 4; col <= 6; ++col) {
      grid.SetValue(col, row, kCellUnknown);
    }
  }

  SafetyFilterConfig config;
  config.treat_unknown_as_obstacle = true;

  const BinaryGrid result = FilterSafetySpace(grid, config);

  // 未知块本身及其 R=0.3m 膨胀环均为 0。
  for (int row = 1; row <= 9; ++row) {
    for (int col = 1; col <= 9; ++col) {
      // 距 3x3 块（[4,6]x[4,6]）最近的块内格欧氏距离 <= 0.3m 则应为 0。
      const int near_col = std::max(4, std::min(6, col));
      const int near_row = std::max(4, std::min(6, row));
      const double dist_m =
          std::hypot((col - near_col) * grid.resolution,
                     (row - near_row) * grid.resolution);
      if (dist_m <= 0.3 + 1e-9) {
        EXPECT_EQ(result.GetValue(col, row), 0)
            << "col=" << col << " row=" << row;
      }
    }
  }
  // 远离未知块的区域仍可通行。
  EXPECT_EQ(result.GetValue(15, 15), 1);
  EXPECT_EQ(result.GetValue(0, 19), 1);
}

TEST(SafetySpaceFilterTest, UnknownRegion_PassThroughWhenConfigured) {
  MapGrid grid = MakeGrid(10, 10, 0.1);
  for (int row = 4; row <= 6; ++row) {
    for (int col = 4; col <= 6; ++col) {
      grid.SetValue(col, row, kCellUnknown);
    }
  }

  SafetyFilterConfig config;
  config.treat_unknown_as_obstacle = false;
  config.robot_radius_m = 0.0;
  config.safety_margin_m = 0.0;

  const BinaryGrid result = FilterSafetySpace(grid, config);

  // 未知格不视为障碍：无膨胀时全部保留为可通行。
  for (uint32_t row = 0; row < grid.height; ++row) {
    for (uint32_t col = 0; col < grid.width; ++col) {
      EXPECT_EQ(result.GetValue(static_cast<int>(col), static_cast<int>(row)),
                1)
          << "col=" << col << " row=" << row;
    }
  }
}

// ---------------------------------------------------------------------------
// 占用阈值边界: >= occupied_threshold 为障碍, 低于则为自由
// ---------------------------------------------------------------------------

TEST(SafetySpaceFilterTest, OccupiedThreshold_BoundaryValues) {
  MapGrid grid = MakeGrid(10, 1, 0.1);
  grid.SetValue(0, 0, 64);   // 低于阈值：自由
  grid.SetValue(1, 0, 65);   // 达到阈值：障碍
  grid.SetValue(2, 0, 100);  // 占用：障碍

  SafetyFilterConfig config;
  config.robot_radius_m = 0.0;
  config.safety_margin_m = 0.0;

  const BinaryGrid result = FilterSafetySpace(grid, config);

  EXPECT_EQ(result.GetValue(0, 0), 1);
  EXPECT_EQ(result.GetValue(1, 0), 0);
  EXPECT_EQ(result.GetValue(2, 0), 0);
}

// ---------------------------------------------------------------------------
// 零膨胀半径: 只做二值标记, 不做形态学腐蚀
// ---------------------------------------------------------------------------

TEST(SafetySpaceFilterTest, ZeroRadius_NoInflationOnlyBinarization) {
  MapGrid grid = MakeGrid(10, 10, 0.1);
  grid.SetValue(5, 5, kCellOccupied);

  SafetyFilterConfig config;
  config.robot_radius_m = 0.0;
  config.safety_margin_m = 0.0;

  const BinaryGrid result = FilterSafetySpace(grid, config);

  EXPECT_EQ(result.GetValue(5, 5), 0);
  EXPECT_EQ(result.GetValue(6, 5), 1);
  EXPECT_EQ(result.GetValue(4, 4), 1);
}

// ---------------------------------------------------------------------------
// BinaryGrid 访问器语义
// ---------------------------------------------------------------------------

TEST(SafetySpaceFilterTest, BinaryGrid_AccessorsHandleOutOfBounds) {
  BinaryGrid grid;
  grid.width = 4;
  grid.height = 3;
  grid.data.assign(12, 1);

  EXPECT_TRUE(grid.IsValidPixel(0, 0));
  EXPECT_TRUE(grid.IsValidPixel(3, 2));
  EXPECT_FALSE(grid.IsValidPixel(-1, 0));
  EXPECT_FALSE(grid.IsValidPixel(0, -1));
  EXPECT_FALSE(grid.IsValidPixel(4, 0));
  EXPECT_FALSE(grid.IsValidPixel(0, 3));

  // 越界读取返回 0（不可通行）。
  EXPECT_EQ(grid.GetValue(-1, 0), 0);
  EXPECT_EQ(grid.GetValue(4, 0), 0);
  EXPECT_EQ(grid.GetValue(0, 3), 0);

  grid.SetValue(1, 1, 0);
  EXPECT_EQ(grid.GetValue(1, 1), 0);
  // 越界写入为 no-op。
  grid.SetValue(99, 99, 0);
  EXPECT_EQ(grid.data[0], 1);
}

}  // namespace
}  // namespace amr_dispatcher_core::planning
