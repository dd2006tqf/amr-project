#include "amr_dispatcher_core/config/config_loader.hpp"

#include <fstream>
#include <sstream>
#include <utility>
#include <vector>

namespace amr_dispatcher_core::config {

namespace {

YAML::Node Navigate(const YAML::Node& root, const std::string& key) {
  if (!root.IsDefined() || !root.IsMap()) return YAML::Node();
  std::stringstream ss(key);
  std::string segment;
  std::vector<std::string> segments;
  while (std::getline(ss, segment, '.')) {
    segments.push_back(segment);
  }
  YAML::Node cur = YAML::Clone(root);
  for (const auto& seg : segments) {
    if (!cur || !cur.IsMap()) {
      return YAML::Node();
    }
    YAML::Node next = cur[seg];
    if (!next || !next.IsDefined()) {
      return YAML::Node();
    }
    cur = next;
  }
  return cur;
}

bool ScalarTypeMatches(const YAML::Node& node, const std::string& type) {
  if (type == "string") return node.IsScalar();
  if (type == "int") return node.IsScalar();  // YAML 把 int 当 scalar；额外在 as<int> 时校验
  if (type == "double") return node.IsScalar();
  if (type == "bool") return node.IsScalar();
  if (type == "list") return node.IsSequence();
  if (type == "map") return node.IsMap();
  return true;
}

bool TryAsInt(const YAML::Node& node, int* out) {
  if (!node.IsScalar()) return false;
  try {
    *out = node.as<int>();
    return true;
  } catch (const YAML::Exception&) {
    return false;
  }
}

}  // namespace

ConfigLoader::ConfigLoader() = default;

void ConfigLoader::RegisterField(FieldSchema schema) {
  std::lock_guard<std::mutex> lock(mutex_);
  schemas_.push_back(std::move(schema));
}

YAML::Node ConfigLoader::LoadFromFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    throw ConfigError("cannot open config file: " + path);
  }
  std::stringstream buf;
  buf << in.rdbuf();
  return LoadFromString(buf.str());
}

YAML::Node ConfigLoader::LoadFromString(const std::string& yaml) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const YAML::Exception& e) {
    throw ConfigError(std::string("yaml parse error: ") + e.what());
  }
  std::vector<FieldSchema> schemas;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    schemas = schemas_;
  }
  // Validate 用 Clone 隔离副作用（yaml-cpp 在某些版本上赋值会污染源节点）。
  YAML::Node validate_root = YAML::Clone(root);
  ConfigValidator::Validate(validate_root, schemas);
  return root;
}

std::string ConfigLoader::GetString(const YAML::Node& root, const std::string& key,
                                    const std::string& fallback) {
  auto n = Navigate(root, key);
  if (!n || !n.IsScalar()) return fallback;
  return n.as<std::string>();
}

int ConfigLoader::GetInt(const YAML::Node& root, const std::string& key, int fallback) {
  auto n = Navigate(root, key);
  if (!n || !n.IsScalar()) return fallback;
  try {
    return n.as<int>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

double ConfigLoader::GetDouble(const YAML::Node& root, const std::string& key, double fallback) {
  auto n = Navigate(root, key);
  if (!n || !n.IsScalar()) return fallback;
  try {
    return n.as<double>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

bool ConfigLoader::GetBool(const YAML::Node& root, const std::string& key, bool fallback) {
  auto n = Navigate(root, key);
  if (!n || !n.IsScalar()) return fallback;
  const std::string s = n.as<std::string>();
  if (s == "true" || s == "True" || s == "TRUE" || s == "yes" || s == "on" || s == "1") return true;
  if (s == "false" || s == "False" || s == "FALSE" || s == "no" || s == "off" || s == "0") return false;
  return fallback;
}

void ConfigLoader::Subscribe(const std::string& key_prefix, ConfigObserver observer) {
  std::lock_guard<std::mutex> lock(mutex_);
  observers_.emplace_back(key_prefix, std::move(observer));
}

void ConfigLoader::NotifyChange(const std::string& key, const std::string& new_value) {
  std::vector<ConfigObserver> matching;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [prefix, obs] : observers_) {
      if (prefix.empty() || key.rfind(prefix, 0) == 0) {
        matching.push_back(obs);
      }
    }
  }
  for (const auto& obs : matching) {
    obs(key, new_value);
  }
}

void ConfigValidator::Validate(YAML::Node root,
                               const std::vector<ConfigLoader::FieldSchema>& schemas) {
  for (const auto& s : schemas) {
    auto node = Navigate(root, s.key);
    if (!node || node.IsNull()) {
      if (s.required) {
        throw ConfigError("missing required field: " + s.key);
      }
      continue;
    }
    if (s.type.empty()) continue;
    if (s.type == "int") {
      int dummy;
      if (!TryAsInt(node, &dummy)) {
        throw ConfigError("type mismatch for " + s.key + ": expected int");
      }
    } else if (!ScalarTypeMatches(node, s.type)) {
      throw ConfigError("type mismatch for " + s.key + ": expected " + s.type);
    }
  }
}

// =================== 热重载 ===================

HotReloadWatcher::HotReloadWatcher(std::string path, ReloadCallback on_reload)
    : path_(std::move(path)), on_reload_(std::move(on_reload)) {}

void HotReloadWatcher::Tick() {
  std::error_code ec;
  auto mtime = std::filesystem::last_write_time(path_, ec);
  if (ec) {
    last_error_ = ec.message();
    return;
  }
  if (!initialized_) {
    last_mtime_ = mtime;
    initialized_ = true;
    return;
  }
  if (mtime == last_mtime_) {
    return;
  }
  last_mtime_ = mtime;
  try {
    std::ifstream in(path_);
    if (!in) {
      last_error_ = "cannot open " + path_;
      return;
    }
    std::stringstream buf;
    buf << in.rdbuf();
    auto root = YAML::Load(buf.str());
    if (on_reload_) on_reload_(root);
    reloaded_ = true;
    last_error_.clear();
  } catch (const std::exception& e) {
    last_error_ = e.what();
  }
}

}  // namespace amr_dispatcher_core::config
