#include "amr_dispatcher_core/facility/facility_catalog.hpp"

#include <fstream>
#include <unordered_set>

namespace amr_dispatcher_core::facility {
namespace {

std::string Trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::optional<std::string> ValueAfterColon(const std::string& line, const std::string& key) {
  const std::string prefix = key + ":";
  if (line.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  return Trim(line.substr(prefix.size()));
}

bool ParseBool(const std::string& value) {
  return value == "true" || value == "True" || value == "1";
}

}  // namespace

std::optional<FacilityCatalog> LoadFacilityCatalog(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return std::nullopt;
  }

  FacilityCatalog catalog;
  FacilityResource current;
  bool in_resource = false;
  bool has_id = false;
  bool has_type = false;
  bool incomplete = false;  // 有 id 却缺 type 的记录：配置错误，不得静默丢弃

  auto flush_resource = [&]() {
    if (in_resource && has_id) {
      if (!has_type) {
        incomplete = true;  // 记录不完整 -> 整份目录判负
      } else {
        catalog.resources.push_back(current);
      }
    }
    current = FacilityResource{};
    in_resource = false;
    has_id = false;
    has_type = false;
  };

  std::string section;
  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (line == "resources:" || line == "facilities:") {
      flush_resource();
      section = line.substr(0, line.size() - 1);
      continue;
    }
    if (line.rfind("- ", 0) == 0) {
      if (section != "resources" && section != "facilities") {
        continue;
      }
      flush_resource();
      in_resource = true;
      line = Trim(line.substr(2));
    }
    if (!in_resource) {
      continue;
    }
    if (auto value = ValueAfterColon(line, "id")) {
      current.id = *value;
      has_id = !current.id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "type")) {
      current.type = *value;
      has_type = !current.type.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "station_id")) {
      current.station_id = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "enabled")) {
      current.enabled = ParseBool(*value);
      continue;
    }
    if (auto value = ValueAfterColon(line, "exclusive")) {
      current.exclusive = ParseBool(*value);
      continue;
    }
    if (auto value = ValueAfterColon(line, "protocol")) {
      current.protocol = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "ip")) {
      current.ip = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "port")) {
      try {
        current.port = static_cast<uint16_t>(std::stoi(*value));
      } catch (...) {}
      continue;
    }
    if (auto value = ValueAfterColon(line, "unit_id")) {
      try {
        current.unit_id = static_cast<uint8_t>(std::stoi(*value));
      } catch (...) {}
      continue;
    }
  }
  flush_resource();
  if (incomplete) {
    return std::nullopt;  // 存在缺 type 的资源记录 -> 配置错误，整份判负
  }

  std::string message;
  if (!ValidateFacilityCatalog(catalog, &message)) {
    return std::nullopt;
  }
  return catalog;
}

const FacilityResource* FindFacilityResource(
    const FacilityCatalog& catalog, const std::string& resource_id) {
  for (const auto& resource : catalog.resources) {
    if (resource.id == resource_id) {
      return &resource;
    }
  }
  return nullptr;
}

const FacilityResource* FindFirstAvailableResource(
    const FacilityCatalog& catalog, const std::string& resource_type,
    const std::vector<std::string>& unavailable_resource_ids) {
  for (const auto& resource : catalog.resources) {
    if (!resource.enabled || resource.type != resource_type) {
      continue;
    }
    bool unavailable = false;
    for (const auto& unavailable_id : unavailable_resource_ids) {
      if (resource.id == unavailable_id) {
        unavailable = true;
        break;
      }
    }
    if (!unavailable) {
      return &resource;
    }
  }
  return nullptr;
}

bool ValidateFacilityCatalog(const FacilityCatalog& catalog, std::string* message) {
  std::unordered_set<std::string> ids;
  for (const auto& resource : catalog.resources) {
    if (resource.id.empty()) {
      if (message != nullptr) {
        *message = "facility resource id is empty";
      }
      return false;
    }
    if (resource.type.empty()) {
      if (message != nullptr) {
        *message = "facility resource type is empty: " + resource.id;
      }
      return false;
    }
    if (!ids.insert(resource.id).second) {
      if (message != nullptr) {
        *message = "duplicate facility resource id: " + resource.id;
      }
      return false;
    }
  }
  if (message != nullptr) {
    *message = "facility catalog ok";
  }
  return true;
}

FacilityCatalogReloader::FacilityCatalogReloader(std::filesystem::path path)
    : path_(std::move(path)),
      watcher_(path_.string(), [this](const YAML::Node&) {
        // HotReloadWatcher 经 mtime 检测到变动后回调
        TryLoad(/*count_as_reload=*/true);
      }) {
  TryLoad(/*count_as_reload=*/false);
}

bool FacilityCatalogReloader::Tick() {
  const std::size_t before = reload_count_;
  watcher_.Tick();
  if (!loaded_) {
    // 启动时缺失的文件可能在运行期间生成，补尝试加载
    TryLoad(/*count_as_reload=*/false);
  }
  return reload_count_ > before;
}

std::string FacilityCatalogReloader::last_error() const {
  if (!last_error_.empty()) {
    return last_error_;
  }
  return watcher_.last_error();
}

void FacilityCatalogReloader::TryLoad(bool count_as_reload) {
  if (path_.empty()) {
    return;
  }
  std::error_code ec;
  if (!std::filesystem::exists(path_, ec)) {
    last_error_ = "facility catalog file does not exist: " + path_.string();
    return;
  }

  const auto loaded = LoadFacilityCatalog(path_);
  if (!loaded.has_value()) {
    last_error_ = "failed to load/validate facility catalog from: " + path_.string();
    return;
  }

  catalog_ = *loaded;
  loaded_ = true;
  last_error_.clear();
  if (count_as_reload) {
    ++reload_count_;
  }
}

}  // namespace amr_dispatcher_core::facility
