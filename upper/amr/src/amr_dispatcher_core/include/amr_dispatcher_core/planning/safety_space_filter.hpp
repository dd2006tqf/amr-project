#ifndef AMR_DISPATCHER_CORE__PLANNING__SAFETY_SPACE_FILTER_HPP_
#define AMR_DISPATCHER_CORE__PLANNING__SAFETY_SPACE_FILTER_HPP_

#include <cstdint>
#include <vector>

#include "amr_dispatcher_core/planning/map_geometry_utils.hpp"

namespace amr_dispatcher_core::planning {

// 二值通过图：1 = 安全通行（自由空间），0 = 不可通行（障碍 / 未知 / 膨胀余量）。
struct BinaryGrid {
  uint32_t width = 0;   // 列数（像素）
  uint32_t height = 0;  // 行数（像素）
  // 行主序（index = row * width + col），长度应为 width * height。
  std::vector<uint8_t> data;

  // 像素坐标是否落在 [0, width) × [0, height) 内。
  bool IsValidPixel(int col, int row) const;

  // 越界（或 data 未初始化）时防御性返回 0（不可通行），
  // 与 MapGrid 的 GetValue 越界返回未知语义对齐：出图区域一律不放行。
  uint8_t GetValue(int col, int row) const;

  // 越界写入为无操作（no-op），不扩容不改尺寸。
  void SetValue(int col, int row, uint8_t val);
};

// 机器人尺寸腐蚀与自由空间过滤参数。
struct SafetyFilterConfig {
  double robot_radius_m = 0.25;  // 机器人半身半径（米）
  double safety_margin_m = 0.05; // 额外安全余量（米）
  // 格子值 >= occupied_threshold 视为障碍（与 nav_msgs/OccupancyGrid 一致，
  // 默认 65 介于"确定空闲"与"确定占用(100)"之间）。
  int8_t occupied_threshold = 65;
  // 未知格（-1）是否按障碍处理。默认 true：未知区域不放行。
  bool treat_unknown_as_obstacle = true;
};

// 将原始 MapGrid 过滤为二值通过图 BinaryGrid。
// 第一步：occupied_threshold 之上（以及可选的未知格）标记为初始障碍，其余为通行候选。
// 第二步：以 R = robot_radius_m + safety_margin_m 为半径做圆形核膨胀——
//         任何与初始障碍欧氏距离 <= R 的格子置 0（可通行区腐蚀）。
BinaryGrid FilterSafetySpace(const MapGrid& grid, const SafetyFilterConfig& config);

}  // namespace amr_dispatcher_core::planning

#endif  // AMR_DISPATCHER_CORE__PLANNING__SAFETY_SPACE_FILTER_HPP_
