#include "amr_dispatcher_core/planning/map_geometry_utils.hpp"

#include <cmath>
#include <cstddef>

namespace amr_dispatcher_core::planning {
namespace {

// 行主序索引：data[row * width + col]。
size_t CellIndex(uint32_t width, int col, int row) {
  return static_cast<size_t>(row) * static_cast<size_t>(width) +
         static_cast<size_t>(col);
}

// half-up 取整：floor(v + 0.5)。对负值对称（-0.5 -> 0，-1.5 -> -1）。
int RoundHalfUp(double v) { return static_cast<int>(std::floor(v + 0.5)); }

}  // namespace

bool MapGrid::IsValidPixel(int col, int row) const {
  return col >= 0 && row >= 0 && static_cast<uint32_t>(col) < width &&
         static_cast<uint32_t>(row) < height;
}

int8_t MapGrid::GetValue(int col, int row) const {
  if (!IsValidPixel(col, row)) {
    return kCellUnknown;
  }
  const size_t index = CellIndex(width, col, row);
  if (index >= data.size()) {
    return kCellUnknown;
  }
  return data[index];
}

void MapGrid::SetValue(int col, int row, int8_t val) {
  if (!IsValidPixel(col, row)) {
    return;
  }
  const size_t index = CellIndex(width, col, row);
  if (index >= data.size()) {
    return;
  }
  data[index] = val;
}

WorldCoord PixelToWorld(const PixelCoord& pixel, const MapGrid& grid) {
  const double px = (pixel.col + 0.5) * grid.resolution;
  const double py = (pixel.row + 0.5) * grid.resolution;
  const double cos_yaw = std::cos(grid.origin_yaw);
  const double sin_yaw = std::sin(grid.origin_yaw);
  return WorldCoord{
      grid.origin_x + cos_yaw * px - sin_yaw * py,
      grid.origin_y + sin_yaw * px + cos_yaw * py,
  };
}

PixelCoord WorldToPixel(const WorldCoord& world, const MapGrid& grid) {
  const double dx = world.x - grid.origin_x;
  const double dy = world.y - grid.origin_y;
  const double cos_yaw = std::cos(grid.origin_yaw);
  const double sin_yaw = std::sin(grid.origin_yaw);
  // 旋转 -yaw（即转置旋转矩阵）把世界位移拉回栅格轴向。
  const double px = cos_yaw * dx + sin_yaw * dy;
  const double py = -sin_yaw * dx + cos_yaw * dy;
  return PixelCoord{
      px / grid.resolution - 0.5,
      py / grid.resolution - 0.5,
  };
}

int PixelColRound(double col) { return RoundHalfUp(col); }

int PixelRowRound(double row) { return RoundHalfUp(row); }

double DistanceMeters(const WorldCoord& a, const WorldCoord& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace amr_dispatcher_core::planning
