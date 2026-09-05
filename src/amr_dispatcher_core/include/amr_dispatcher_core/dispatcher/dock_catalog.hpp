#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_core::dispatcher {

struct ChargingDock {
  std::string id;
  std::string station_id;            // 物理停靠站点
  std::string approach_station_id;   // 航向对齐或排队等待站点
  std::string resource_id;           // 绑定的物理充电机资源锁名
  bool enabled = true;
  bool occupied = false;
  std::string current_robot_id;
};

class DockCatalog {
 public:
  DockCatalog() = default;

  void AddDock(ChargingDock dock);
  void RemoveDock(const std::string& dock_id);

  const ChargingDock* FindDock(const std::string& dock_id) const;
  const ChargingDock* FindAvailableDock() const;

  bool SetDockEnabled(const std::string& dock_id, bool enabled);
  bool OccupyDock(const std::string& dock_id, const std::string& robot_id);
  bool ReleaseDock(const std::string& dock_id);

  std::vector<ChargingDock> ListDocks() const;
  std::size_t size() const { return docks_.size(); }
  void Clear();

 private:
  std::unordered_map<std::string, ChargingDock> docks_;
};

}  // namespace amr_dispatcher_core::dispatcher
