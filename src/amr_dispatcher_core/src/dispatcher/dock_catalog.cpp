#include "amr_dispatcher_core/dispatcher/dock_catalog.hpp"

namespace amr_dispatcher_core::dispatcher {

void DockCatalog::AddDock(ChargingDock dock) {
  std::string id = dock.id;
  docks_[id] = std::move(dock);
}

void DockCatalog::RemoveDock(const std::string& dock_id) {
  docks_.erase(dock_id);
}

const ChargingDock* DockCatalog::FindDock(const std::string& dock_id) const {
  auto it = docks_.find(dock_id);
  if (it == docks_.end()) return nullptr;
  return &it->second;
}

const ChargingDock* DockCatalog::FindAvailableDock() const {
  for (const auto& [_, d] : docks_) {
    if (d.enabled && !d.occupied) {
      return &d;
    }
  }
  return nullptr;
}

bool DockCatalog::SetDockEnabled(const std::string& dock_id, bool enabled) {
  auto it = docks_.find(dock_id);
  if (it == docks_.end()) return false;
  it->second.enabled = enabled;
  return true;
}

bool DockCatalog::OccupyDock(const std::string& dock_id, const std::string& robot_id) {
  auto it = docks_.find(dock_id);
  if (it == docks_.end() || !it->second.enabled || it->second.occupied) {
    return false;
  }
  it->second.occupied = true;
  it->second.current_robot_id = robot_id;
  return true;
}

bool DockCatalog::ReleaseDock(const std::string& dock_id) {
  auto it = docks_.find(dock_id);
  if (it == docks_.end()) return false;
  it->second.occupied = false;
  it->second.current_robot_id.clear();
  return true;
}

std::vector<ChargingDock> DockCatalog::ListDocks() const {
  std::vector<ChargingDock> list;
  list.reserve(docks_.size());
  for (const auto& [_, d] : docks_) {
    list.push_back(d);
  }
  return list;
}

void DockCatalog::Clear() {
  docks_.clear();
}

}  // namespace amr_dispatcher_core::dispatcher
