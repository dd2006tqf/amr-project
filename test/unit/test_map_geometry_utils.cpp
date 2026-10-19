#include "amr_dispatcher_core/planning/map_geometry_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace amr_dispatcher_core::planning {
namespace {

constexpr double kTol = 1e-9;

MapGrid MakeGrid(uint32_t width, uint32_t height, double resolution,
                 double origin_x = 0.0, double origin_y = 0.0,
                 double origin_yaw = 0.0) {
  MapGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.origin_x = origin_x;
  grid.origin_y = origin_y;
  grid.origin_yaw = origin_yaw;
  grid.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height),
                   kCellFree);
  return grid;
}

// ---------------------------------------------------------------------------
// 零原点、无旋转：标准转换与半像素中心对齐
// ---------------------------------------------------------------------------

TEST(MapGeometryUtilsTest, ZeroOriginNoRotation_PixelCenterMapsToHalfCellOffset) {
  const MapGrid grid = MakeGrid(10, 8, 0.05);

  // 像素 (0,0) 的中心 = (0.5 * res, 0.5 * res)，不是原点本身。
  const WorldCoord origin_pixel = PixelToWorld({0.0, 0.0}, grid);
  EXPECT_NEAR(origin_pixel.x, 0.5 * grid.resolution, kTol);
  EXPECT_NEAR(origin_pixel.y, 0.5 * grid.resolution, kTol);

  const WorldCoord pixel_23 = PixelToWorld({2.0, 3.0}, grid);
  EXPECT_NEAR(pixel_23.x, 2.5 * grid.resolution, kTol);
  EXPECT_NEAR(pixel_23.y, 3.5 * grid.resolution, kTol);
}

TEST(MapGeometryUtilsTest, ZeroOriginNoRotation_WorldToPixelInverts) {
  const MapGrid grid = MakeGrid(10, 8, 0.05);

  const PixelCoord pixel = WorldToPixel({0.125, 0.175}, grid);
  EXPECT_NEAR(pixel.col, 2.0, kTol);
  EXPECT_NEAR(pixel.row, 3.0, kTol);

  const PixelCoord origin_pixel = WorldToPixel({0.025, 0.025}, grid);
  EXPECT_NEAR(origin_pixel.col, 0.0, kTol);
  EXPECT_NEAR(origin_pixel.row, 0.0, kTol);
}

TEST(MapGeometryUtilsTest, ZeroOriginNoRotation_RoundTripConsistency) {
  const MapGrid grid = MakeGrid(10, 8, 0.05);
  const PixelCoord samples[] = {{0.0, 0.0}, {9.0, 7.0}, {4.0, 4.0},
                                {7.0, 2.0}, {1.25, 5.75}};

  for (const PixelCoord& sample : samples) {
    const PixelCoord round_trip =
        WorldToPixel(PixelToWorld(sample, grid), grid);
    EXPECT_NEAR(round_trip.col, sample.col, kTol);
    EXPECT_NEAR(round_trip.row, sample.row, kTol);
  }
}

// ---------------------------------------------------------------------------
// 非零原点平移
// ---------------------------------------------------------------------------

TEST(MapGeometryUtilsTest, NonZeroOrigin_TranslationShiftsWorldFrame) {
  const MapGrid grid = MakeGrid(10, 8, 0.05, /*origin_x=*/-10.0,
                                /*origin_y=*/-5.0);

  const WorldCoord origin_pixel = PixelToWorld({0.0, 0.0}, grid);
  EXPECT_NEAR(origin_pixel.x, -10.0 + 0.5 * grid.resolution, kTol);
  EXPECT_NEAR(origin_pixel.y, -5.0 + 0.5 * grid.resolution, kTol);

  const PixelCoord back = WorldToPixel(origin_pixel, grid);
  EXPECT_NEAR(back.col, 0.0, kTol);
  EXPECT_NEAR(back.row, 0.0, kTol);
}

TEST(MapGeometryUtilsTest, NonZeroOrigin_RoundTripConsistency) {
  const MapGrid grid = MakeGrid(20, 15, 0.02, /*origin_x=*/-10.0,
                                /*origin_y=*/-5.0);
  const PixelCoord samples[] = {{0.0, 0.0}, {19.0, 14.0}, {8.5, 3.25}};

  for (const PixelCoord& sample : samples) {
    const PixelCoord round_trip =
        WorldToPixel(PixelToWorld(sample, grid), grid);
    EXPECT_NEAR(round_trip.col, sample.col, kTol);
    EXPECT_NEAR(round_trip.row, sample.row, kTol);
  }
}

// ---------------------------------------------------------------------------
// 非零旋转（30° 与 15°）
// ---------------------------------------------------------------------------

TEST(MapGeometryUtilsTest, NonZeroRotation_MapsPixelThroughYaw) {
  const double yaw = M_PI / 6.0;  // 30°
  const MapGrid grid = MakeGrid(10, 8, 0.05, /*origin_x=*/-1.0,
                                /*origin_y=*/2.0, yaw);

  const double px = 2.5 * grid.resolution;
  const double py = 1.5 * grid.resolution;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const WorldCoord world = PixelToWorld({2.0, 1.0}, grid);
  EXPECT_NEAR(world.x, -1.0 + cos_yaw * px - sin_yaw * py, kTol);
  EXPECT_NEAR(world.y, 2.0 + sin_yaw * px + cos_yaw * py, kTol);
}

TEST(MapGeometryUtilsTest, NonZeroRotation_RoundTripConsistency) {
  const double yaws[] = {M_PI / 6.0, M_PI / 12.0, -M_PI / 4.0};
  const PixelCoord samples[] = {{0.0, 0.0}, {9.0, 7.0}, {4.5, 2.25}};

  for (const double yaw : yaws) {
    const MapGrid grid = MakeGrid(10, 8, 0.05, /*origin_x=*/-3.25,
                                  /*origin_y=*/7.75, yaw);
    for (const PixelCoord& sample : samples) {
      const PixelCoord round_trip =
          WorldToPixel(PixelToWorld(sample, grid), grid);
      EXPECT_NEAR(round_trip.col, sample.col, 1e-9);
      EXPECT_NEAR(round_trip.row, sample.row, 1e-9);
    }
  }
}

// ---------------------------------------------------------------------------
// 取整与越界防御
// ---------------------------------------------------------------------------

TEST(MapGeometryUtilsTest, PixelColRowRound_RoundsHalfUp) {
  EXPECT_EQ(PixelColRound(0.0), 0);
  EXPECT_EQ(PixelColRound(0.49), 0);
  EXPECT_EQ(PixelColRound(0.5), 1);
  EXPECT_EQ(PixelColRound(2.7), 3);
  EXPECT_EQ(PixelColRound(-0.5), 0);
  EXPECT_EQ(PixelColRound(-1.2), -1);
  EXPECT_EQ(PixelColRound(-2.5), -2);

  EXPECT_EQ(PixelRowRound(0.0), 0);
  EXPECT_EQ(PixelRowRound(0.5), 1);
  EXPECT_EQ(PixelRowRound(3.4999), 3);
  EXPECT_EQ(PixelRowRound(-0.75), -1);
}

TEST(MapGeometryUtilsTest, IsValidPixel_RejectsOutOfBounds) {
  const MapGrid grid = MakeGrid(10, 8, 0.05);

  EXPECT_TRUE(grid.IsValidPixel(0, 0));
  EXPECT_TRUE(grid.IsValidPixel(9, 0));
  EXPECT_TRUE(grid.IsValidPixel(0, 7));
  EXPECT_TRUE(grid.IsValidPixel(9, 7));

  EXPECT_FALSE(grid.IsValidPixel(-1, 0));
  EXPECT_FALSE(grid.IsValidPixel(0, -1));
  EXPECT_FALSE(grid.IsValidPixel(10, 0));
  EXPECT_FALSE(grid.IsValidPixel(0, 8));
  EXPECT_FALSE(grid.IsValidPixel(-5, -5));
  EXPECT_FALSE(grid.IsValidPixel(100, 100));
}

TEST(MapGeometryUtilsTest, GetValue_OutOfBoundsReturnsUnknown) {
  MapGrid grid = MakeGrid(10, 8, 0.05);
  grid.SetValue(3, 2, kCellOccupied);
  EXPECT_EQ(grid.GetValue(3, 2), kCellOccupied);

  // 越界读一律按未知区域（-1）返回，下游安全过滤会将其按障碍处理。
  EXPECT_EQ(grid.GetValue(-1, 2), kCellUnknown);
  EXPECT_EQ(grid.GetValue(10, 0), kCellUnknown);
  EXPECT_EQ(grid.GetValue(0, 8), kCellUnknown);
  EXPECT_EQ(grid.GetValue(-3, -3), kCellUnknown);

  // data 为空（未初始化）时，越界与在界读都防御性返回未知。
  MapGrid empty_grid;
  empty_grid.width = 4;
  empty_grid.height = 4;
  EXPECT_EQ(empty_grid.GetValue(1, 1), kCellUnknown);
}

TEST(MapGeometryUtilsTest, SetValue_OutOfBoundsIsNoOp) {
  MapGrid grid = MakeGrid(10, 8, 0.05);
  const size_t size_before = grid.data.size();

  grid.SetValue(-1, 0, kCellOccupied);
  grid.SetValue(10, 0, kCellOccupied);
  grid.SetValue(0, 8, kCellOccupied);

  EXPECT_EQ(grid.data.size(), size_before);
  EXPECT_EQ(grid.GetValue(0, 0), kCellFree);
}

// ---------------------------------------------------------------------------
// 距离
// ---------------------------------------------------------------------------

TEST(MapGeometryUtilsTest, DistanceMeters_Euclidean) {
  EXPECT_NEAR(DistanceMeters({0.0, 0.0}, {3.0, 4.0}), 5.0, kTol);
  EXPECT_NEAR(DistanceMeters({1.5, -2.0}, {1.5, -2.0}), 0.0, kTol);
  EXPECT_NEAR(DistanceMeters({-1.0, -1.0}, {2.0, 3.0}), 5.0, kTol);
}

}  // namespace
}  // namespace amr_dispatcher_core::planning
