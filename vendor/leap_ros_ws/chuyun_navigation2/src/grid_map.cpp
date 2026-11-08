#include "chuyun_navigation2/grid_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "chuyun_navigation2/types.hpp"

namespace chuyun_navigation2
{
namespace
{

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string stripQuotes(std::string value)
{
  value = trim(value);
  if (value.size() >= 2 &&
    ((value.front() == '\'' && value.back() == '\'') ||
    (value.front() == '"' && value.back() == '"')))
  {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

std::string nextPgmToken(std::istream & input)
{
  std::string token;
  while (input >> token) {
    if (!token.empty() && token.front() == '#') {
      std::string ignored;
      std::getline(input, ignored);
      continue;
    }
    return token;
  }
  throw std::runtime_error("PGM 文件内容不完整");
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

struct DistanceEntry
{
  float distance;
  int index;
  bool operator<(const DistanceEntry & other) const {return distance > other.distance;}
};

}  // namespace

void GridMap::load(const std::string & yaml_path, const std::string & frame_id)
{
  if (yaml_path.empty()) throw std::runtime_error("map_yaml 参数为空");
  std::ifstream yaml(yaml_path);
  if (!yaml) throw std::runtime_error("无法打开地图 YAML: " + yaml_path);

  std::string image_name;
  double resolution = 0.05;
  double occupied_threshold = 0.65;
  double free_threshold = 0.25;
  int negate = 0;
  std::array<double, 3> origin{0.0, 0.0, 0.0};
  std::string line;
  while (std::getline(yaml, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    const std::string key = trim(line.substr(0, colon));
    std::string value = trim(line.substr(colon + 1));
    if (key == "image") image_name = stripQuotes(value);
    else if (key == "resolution") resolution = std::stod(value);
    else if (key == "occupied_thresh") occupied_threshold = std::stod(value);
    else if (key == "free_thresh") free_threshold = std::stod(value);
    else if (key == "negate") negate = std::stoi(value);
    else if (key == "origin") {
      std::replace(value.begin(), value.end(), '[', ' ');
      std::replace(value.begin(), value.end(), ']', ' ');
      std::replace(value.begin(), value.end(), ',', ' ');
      std::stringstream values(value);
      if (!(values >> origin[0] >> origin[1] >> origin[2])) {
        throw std::runtime_error("地图 YAML 的 origin 字段无效");
      }
    }
  }
  if (image_name.empty()) throw std::runtime_error("地图 YAML 缺少 image 字段");
  if (resolution <= 0.0) throw std::runtime_error("地图分辨率必须大于零");

  std::filesystem::path image_path(image_name);
  if (image_path.is_relative()) {
    image_path = std::filesystem::path(yaml_path).parent_path() / image_path;
  }
  std::ifstream image(image_path, std::ios::binary);
  if (!image) throw std::runtime_error("无法打开地图图像: " + image_path.string());

  const std::string magic = nextPgmToken(image);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("仅支持 PGM P5/P2 地图");
  }
  const int width = std::stoi(nextPgmToken(image));
  const int height = std::stoi(nextPgmToken(image));
  const int maximum_value = std::stoi(nextPgmToken(image));
  if (width <= 0 || height <= 0 || maximum_value <= 0 || maximum_value > 255) {
    throw std::runtime_error("PGM 地图尺寸或灰度范围无效");
  }

  std::vector<uint8_t> pixels(static_cast<size_t>(width) * height);
  if (magic == "P5") {
    char separator = 0;
    image.get(separator);
    if (separator == '\r' && image.peek() == '\n') image.get();
    image.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (image.gcount() != static_cast<std::streamsize>(pixels.size())) {
      throw std::runtime_error("PGM 地图像素不完整");
    }
  } else {
    for (auto & pixel : pixels) {
      pixel = static_cast<uint8_t>(std::stoi(nextPgmToken(image)));
    }
  }

  occupancy_grid_ = nav_msgs::msg::OccupancyGrid();
  occupancy_grid_.header.frame_id = frame_id;
  occupancy_grid_.info.resolution = static_cast<float>(resolution);
  occupancy_grid_.info.width = static_cast<uint32_t>(width);
  occupancy_grid_.info.height = static_cast<uint32_t>(height);
  occupancy_grid_.info.origin.position.x = origin[0];
  occupancy_grid_.info.origin.position.y = origin[1];
  occupancy_grid_.info.origin.orientation = quaternionFromYaw(origin[2]);
  occupancy_grid_.data.assign(pixels.size(), -1);
  map_origin_yaw_ = origin[2];

  for (int row = 0; row < height; ++row) {
    const int grid_y = height - 1 - row;
    for (int x = 0; x < width; ++x) {
      const uint8_t pixel = pixels[static_cast<size_t>(row) * width + x];
      double occupied = static_cast<double>(pixel) / maximum_value;
      if (!negate) occupied = 1.0 - occupied;
      int8_t value = -1;
      if (occupied > occupied_threshold) value = 100;
      else if (occupied < free_threshold) value = 0;
      occupancy_grid_.data[static_cast<size_t>(grid_y) * width + x] = value;
    }
  }
}

void GridMap::buildFields(double inflation_radius, double maximum_distance)
{
  if (occupancy_grid_.data.empty()) throw std::runtime_error("必须先加载地图");
  const int width = static_cast<int>(occupancy_grid_.info.width);
  const int height = static_cast<int>(occupancy_grid_.info.height);
  const float resolution = occupancy_grid_.info.resolution;
  maximum_distance_ = std::max(maximum_distance, static_cast<double>(resolution));
  distance_field_.assign(occupancy_grid_.data.size(), static_cast<float>(maximum_distance_));

  std::priority_queue<DistanceEntry> queue;
  for (size_t i = 0; i < occupancy_grid_.data.size(); ++i) {
    if (occupancy_grid_.data[i] >= 65) {
      distance_field_[i] = 0.0F;
      queue.push({0.0F, static_cast<int>(i)});
    }
  }

  const std::array<int, 8> dx{-1, 0, 1, -1, 1, -1, 0, 1};
  const std::array<int, 8> dy{-1, -1, -1, 0, 0, 1, 1, 1};
  const std::array<float, 8> step{
    1.41421356F, 1.0F, 1.41421356F, 1.0F,
    1.0F, 1.41421356F, 1.0F, 1.41421356F};
  while (!queue.empty()) {
    const DistanceEntry current = queue.top();
    queue.pop();
    if (current.distance > distance_field_[current.index]) continue;
    const int x = current.index % width;
    const int y = current.index / width;
    for (size_t direction = 0; direction < dx.size(); ++direction) {
      const int next_x = x + dx[direction];
      const int next_y = y + dy[direction];
      if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height) continue;
      const int next_index = next_y * width + next_x;
      const float candidate = current.distance + step[direction] * resolution;
      if (candidate >= distance_field_[next_index] || candidate > maximum_distance_) continue;
      distance_field_[next_index] = candidate;
      queue.push({candidate, next_index});
    }
  }

  inflated_grid_ = occupancy_grid_;
  const double radius = std::max(0.0, inflation_radius);
  for (size_t i = 0; i < occupancy_grid_.data.size(); ++i) {
    inflated_grid_.data[i] =
      (occupancy_grid_.data[i] < 0 || distance_field_[i] <= radius) ? 100 : 0;
  }
}

bool GridMap::inBounds(int cell_x, int cell_y) const
{
  return cell_x >= 0 && cell_y >= 0 &&
         cell_x < static_cast<int>(occupancy_grid_.info.width) &&
         cell_y < static_cast<int>(occupancy_grid_.info.height);
}

size_t GridMap::index(int cell_x, int cell_y) const
{
  return static_cast<size_t>(cell_y) * occupancy_grid_.info.width + cell_x;
}

bool GridMap::worldToCell(double world_x, double world_y, int & cell_x, int & cell_y) const
{
  const double dx = world_x - occupancy_grid_.info.origin.position.x;
  const double dy = world_y - occupancy_grid_.info.origin.position.y;
  const double c = std::cos(map_origin_yaw_);
  const double s = std::sin(map_origin_yaw_);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  cell_x = static_cast<int>(std::floor(local_x / occupancy_grid_.info.resolution));
  cell_y = static_cast<int>(std::floor(local_y / occupancy_grid_.info.resolution));
  return inBounds(cell_x, cell_y);
}

Point2D GridMap::cellToWorld(int cell_x, int cell_y) const
{
  const double local_x = (cell_x + 0.5) * occupancy_grid_.info.resolution;
  const double local_y = (cell_y + 0.5) * occupancy_grid_.info.resolution;
  const double c = std::cos(map_origin_yaw_);
  const double s = std::sin(map_origin_yaw_);
  return {
    occupancy_grid_.info.origin.position.x + c * local_x - s * local_y,
    occupancy_grid_.info.origin.position.y + s * local_x + c * local_y};
}

bool GridMap::isFree(int cell_x, int cell_y) const
{
  return inBounds(cell_x, cell_y) && !inflated_grid_.data.empty() &&
         inflated_grid_.data[index(cell_x, cell_y)] == 0;
}

bool GridMap::nearestFree(int & cell_x, int & cell_y, int maximum_radius_cells) const
{
  if (isFree(cell_x, cell_y)) return true;
  for (int radius = 1; radius <= maximum_radius_cells; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::abs(dx) != radius && std::abs(dy) != radius) continue;
        if (isFree(cell_x + dx, cell_y + dy)) {
          cell_x += dx;
          cell_y += dy;
          return true;
        }
      }
    }
  }
  return false;
}

bool GridMap::lineIsFree(int x0, int y0, int x1, int y1) const
{
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  while (true) {
    if (!isFree(x0, y0)) return false;
    if (x0 == x1 && y0 == y1) return true;
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x0 += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

bool GridMap::distanceAtWorld(double world_x, double world_y, double & distance) const
{
  int cell_x = 0;
  int cell_y = 0;
  if (!worldToCell(world_x, world_y, cell_x, cell_y)) return false;
  const size_t cell_index = index(cell_x, cell_y);
  if (occupancy_grid_.data[cell_index] < 0 || distance_field_.empty()) return false;
  distance = distance_field_[cell_index];
  return true;
}

double GridMap::distanceAtCell(int cell_x, int cell_y) const
{
  if (!inBounds(cell_x, cell_y) || distance_field_.empty()) return 0.0;
  return distance_field_[index(cell_x, cell_y)];
}

}  // namespace chuyun_navigation2
