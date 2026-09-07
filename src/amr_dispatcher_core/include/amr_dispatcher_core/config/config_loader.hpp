#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace amr_dispatcher_core::config {

class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// 单次加载入口：读 YAML 文件 + 应用 schema 校验。线程安全。
class ConfigLoader {
 public:
  using Validator = std::function<void(const YAML::Node&)>;

  ConfigLoader();

  // 注册字段 schema：key 形如 "chassis.serial_device"，type 提示，required 表示必须存在。
  struct FieldSchema {
    std::string key;
    std::string type;     // "string" | "int" | "double" | "bool" | "list" | "map"
    bool required = false;
    std::string default_value;  // YAML 字面量；非空且 required=false 时缺失则补默认
  };

  void RegisterField(FieldSchema schema);

  // 从文件加载；通过 schema 校验；返回根节点供调用方取值。
  // 校验失败抛 ConfigError；file 不存在抛 ConfigError。
  YAML::Node LoadFromFile(const std::string& path);

  // 从字符串加载（同上）
  YAML::Node LoadFromString(const std::string& yaml);

  // 工具：取节点路径上的标量值
  static std::string GetString(const YAML::Node& root, const std::string& key,
                               const std::string& fallback = "");
  static int GetInt(const YAML::Node& root, const std::string& key, int fallback = 0);
  static double GetDouble(const YAML::Node& root, const std::string& key,
                          double fallback = 0.0);
  static bool GetBool(const YAML::Node& root, const std::string& key, bool fallback = false);

  // 观察者模式 (Observer Pattern)：允许业务组件订阅配置项在线更新事件
  using ConfigObserver = std::function<void(const std::string& key, const std::string& new_value)>;
  void Subscribe(const std::string& key_prefix, ConfigObserver observer);
  void NotifyChange(const std::string& key, const std::string& new_value);

 private:
  std::vector<FieldSchema> schemas_;
  std::vector<std::pair<std::string, ConfigObserver>> observers_;
  std::mutex mutex_;
};

// 热重载：周期性检查文件 mtime，发现变化时调用 on_reload(root)。
class HotReloadWatcher {
 public:
  using ReloadCallback = std::function<void(const YAML::Node&)>;

  HotReloadWatcher(std::string path, ReloadCallback on_reload);

  // 由调用方驱动（建议 1Hz）
  void Tick();

  bool reloaded() const { return reloaded_; }
  std::string last_error() const { return last_error_; }

 private:
  std::string path_;
  ReloadCallback on_reload_;
  std::filesystem::file_time_type last_mtime_{};
  bool initialized_ = false;
  bool reloaded_ = false;
  std::string last_error_;
};

// 通用 schema 校验器：required 字段缺失 / 类型不符抛 ConfigError。
class ConfigValidator {
 public:
  static void Validate(YAML::Node root, const std::vector<ConfigLoader::FieldSchema>& schemas);
};

}  // namespace amr_dispatcher_core::config
