#include "amr_dispatcher_core/planning/safety_space_filter.hpp"

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::planning {
namespace {

// 行主序索引：data[row * width + col]。
size_t CellIndex(uint32_t width, int col, int row) {
  return static_cast<size_t>(row) * static_cast<size_t>(width) +
         static_cast<size_t>(col);
}

// 欧氏距离比较容差：抵消 dx*res 等浮点乘积的舍入误差
// （例如 3*0.1 = 0.30000000000000004 > 0.3，边界格不应因此漏标）。
constexpr double kDistanceTolerance = 1e-9;

}  // namespace

bool BinaryGrid::IsValidPixel(int col, int row) const {
  return col >= 0 && row >= 0 && static_cast<uint32_t>(col) < width &&
         static_cast<uint32_t>(row) < height;
}

uint8_t BinaryGrid::GetValue(int col, int row) const {
  if (!IsValidPixel(col, row)) {
    return 0;
  }
  const size_t index = CellIndex(width, col, row);
  if (index >= data.size()) {
    return 0;
  }
  return data[index];
}

void BinaryGrid::SetValue(int col, int row, uint8_t val) {
  if (!IsValidPixel(col, row)) {
    return;
  }
  const size_t index = CellIndex(width, col, row);
  if (index >= data.size()) {
    return;
  }
  data[index] = val;
}

BinaryGrid FilterSafetySpace(const MapGrid& grid,
                             const SafetyFilterConfig& config) {
  BinaryGrid result;
  result.width = grid.width;
  result.height = grid.height;
  const size_t total = static_cast<size_t>(grid.width) *
                       static_cast<size_t>(grid.height);
  result.data.assign(total, 1);  // 默认：通行候选

  // ---- 第一步：二值标记（占用 / 未知 -> 初始障碍），同时收集障碍点 ----
  std::vector<std::pair<int, int>> obstacles;
  obstacles.reserve(total / 8);
  for (uint32_t row = 0; row < grid.height; ++row) {
    for (uint32_t col = 0; col < grid.width; ++col) {
      // GetValue 对越界或未初始化数据防御性返回 -1（未知），
      // 未知是否视为障碍由 treat_unknown_as_obstacle 决定。
      const int8_t value = grid.GetValue(static_cast<int>(col),
                                         static_cast<int>(row));
      const bool is_obstacle =
          value >= config.occupied_threshold ||
          (config.treat_unknown_as_obstacle && value < 0);
      if (is_obstacle) {
        result.data[CellIndex(result.width, static_cast<int>(col),
                              static_cast<int>(row))] = 0;
        obstacles.emplace_back(static_cast<int>(col),
                               static_cast<int>(row));
      }
    }
  }

  // ---- 第二步：圆形核膨胀（等价于可通行区欧式距离腐蚀） ----
  const double radius = config.robot_radius_m + config.safety_margin_m;
  const double resolution = grid.resolution;
  if (radius <= 0.0 || resolution <= 0.0 || obstacles.empty()) {
    return result;  // 无需膨胀
  }
  const int r_pix = static_cast<int>(std::ceil(radius / resolution));
  if (r_pix == 0) {
    return result;
  }

  // 预计算圆形核偏移：所有满足 hypot(dx*res, dy*res) <= R 的窗口偏移。
  std::vector<std::pair<int, int>> kernel_offsets;
  kernel_offsets.reserve(static_cast<size_t>(2 * r_pix + 1) *
                         static_cast<size_t>(2 * r_pix + 1));
  const double tolerance = radius + kDistanceTolerance;
  for (int dy = -r_pix; dy <= r_pix; ++dy) {
    for (int dx = -r_pix; dx <= r_pix; ++dx) {
      const double dist_m =
          std::hypot(static_cast<double>(dx) * resolution,
                     static_cast<double>(dy) * resolution);
      if (dist_m <= tolerance) {
        kernel_offsets.emplace_back(dx, dy);
      }
    }
  }

  // 对每个初始障碍，在其 R 半径窗口内把所有可达像素置 0。
  for (const auto& obstacle : obstacles) {
    const int obstacle_col = obstacle.first;
    const int obstacle_row = obstacle.second;
    for (const auto& offset : kernel_offsets) {
      const int col = obstacle_col + offset.first;
      const int row = obstacle_row + offset.second;
      if (result.IsValidPixel(col, row)) {
        result.data[CellIndex(result.width, col, row)] = 0;
      }
    }
  }
  return result;
}

}  // namespace amr_dispatcher_core::planning
