#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "amr_dispatcher_core/config/config_loader.hpp"

namespace amr_dispatcher_core::facility {

struct FacilityResource {
  std::string id;
  std::string type;
  std::string station_id;
  bool enabled = true;
  bool exclusive = true;
  std::string protocol = "mock";  // "mock" | "modbus_tcp" | "opc_ua"
  std::string ip = "127.0.0.1";
  uint16_t port = 502;
  uint8_t unit_id = 1;
};

struct FacilityCatalog {
  std::vector<FacilityResource> resources;
};

std::optional<FacilityCatalog> LoadFacilityCatalog(const std::filesystem::path& path);
const FacilityResource* FindFacilityResource(
    const FacilityCatalog& catalog, const std::string& resource_id);
const FacilityResource* FindFirstAvailableResource(
    const FacilityCatalog& catalog, const std::string& resource_type,
    const std::vector<std::string>& unavailable_resource_ids);
bool ValidateFacilityCatalog(const FacilityCatalog& catalog, std::string* message);

// 设施目录热重载器（与 StationCatalogReloader 模式完全一致）
class FacilityCatalogReloader {
 public:
  explicit FacilityCatalogReloader(std::filesystem::path path);

  bool Tick();

  const FacilityCatalog& catalog() const { return catalog_; }
  bool loaded() const { return loaded_; }
  std::string last_error() const;

 private:
  void TryLoad(bool count_as_reload);

  std::filesystem::path path_;
  FacilityCatalog catalog_{};
  bool loaded_ = false;
  std::size_t reload_count_ = 0;
  std::string last_error_;
  config::HotReloadWatcher watcher_;
};

}  // namespace amr_dispatcher_core::facility
