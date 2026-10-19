#ifndef AMR_DISPATCHER_CORE__PLANNING__MAP_GEOMETRY_UTILS_HPP_
#define AMR_DISPATCHER_CORE__PLANNING__MAP_GEOMETRY_UTILS_HPP_

#include <cstdint>
#include <vector>

namespace amr_dispatcher_core::planning {

// 栅格占用语义（与 nav_msgs/OccupancyGrid 一致）：
//   -1 未知 / 0 空闲 / 100 占用。下游 SafetySpaceFilter 将未知与占用统一按不可通行处理。
inline constexpr int8_t kCellUnknown = -1;
inline constexpr int8_t kCellFree = 0;
inline constexpr int8_t kCellOccupied = 100;

// 栅格地图纯数据结构（零 ROS 依赖）。
// data 为行主序（index = row * width + col），长度应为 width * height。
struct MapGrid {
  uint32_t width = 0;    // 列数（像素）
  uint32_t height = 0;   // 行数（像素）
  double resolution = 0.05;  // m/pixel
  double origin_x = 0.0;     // 栅格原点（像素 (0,0) 左下角所在世界坐标）
  double origin_y = 0.0;
  double origin_yaw = 0.0;  // 栅格相对世界系的旋转角 (radians)
  std::vector<int8_t> data; // cell occupancy: -1 unknown, 0 free, 100 occupied

  // 像素坐标是否落在 [0, width) × [0, height) 内。
  bool IsValidPixel(int col, int row) const;

  // 越界（或 data 未初始化）时防御性返回 kCellUnknown(-1)，
  // 让下游安全过滤把出图区域按障碍处理，而不是放行。
  int8_t GetValue(int col, int row) const;

  // 越界写入为无操作（no-op），不扩容不改尺寸。
  void SetValue(int col, int row, int8_t val);
};

// 像素坐标（可为亚像素，列 col 向右增大，行 row 向上/向下由图数据约定决定）
struct PixelCoord {
  double col = 0.0;
  double row = 0.0;
};

// 世界坐标（米）
struct WorldCoord {
  double x = 0.0;
  double y = 0.0;
};

// 像素 -> 世界：以栅格中心为基准取半像素偏移，再施加 origin_yaw 旋转与原点平移。
//   px = (col + 0.5) * resolution; py = (row + 0.5) * resolution
//   world_x = origin_x + cos(yaw) * px - sin(yaw) * py
//   world_y = origin_y + sin(yaw) * px + cos(yaw) * py
WorldCoord PixelToWorld(const PixelCoord& pixel, const MapGrid& grid);

// 世界 -> 像素：PixelToWorld 的严格逆变换。
//   dx = world.x - origin_x; dy = world.y - origin_y
//   px =  cos(yaw) * dx + sin(yaw) * dy
//   py = -sin(yaw) * dx + cos(yaw) * dy
//   col = px / resolution - 0.5; row = py / resolution - 0.5
PixelCoord WorldToPixel(const WorldCoord& world, const MapGrid& grid);

// 亚像素列/行取整（half-up：floor(v + 0.5)），用于像素索引落位。
int PixelColRound(double col);
int PixelRowRound(double row);

// 两点间欧氏距离（米）。
double DistanceMeters(const WorldCoord& a, const WorldCoord& b);

}  // namespace amr_dispatcher_core::planning

#endif  // AMR_DISPATCHER_CORE__PLANNING__MAP_GEOMETRY_UTILS_HPP_
