#include "amr_dispatcher_core/config/config_loader.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <thread>

using namespace amr_dispatcher_core::config;
using namespace std::chrono_literals;

namespace {

std::string WriteTemp(const std::string& content) {
  std::string path = std::tmpnam(nullptr);
  std::ofstream out(path);
  out << content;
  return path;
}

}  // namespace

TEST(ConfigLoaderTest, LoadFromStringBasic) {
  ConfigLoader l;
  l.RegisterField({"chassis.serial_device", "string", true});
  l.RegisterField({"chassis.serial_baud", "int", false, "115200"});
  auto root = l.LoadFromString("chassis:\n  serial_device: /dev/ttyUSB0\n  serial_baud: 9600\n");
  EXPECT_EQ(ConfigLoader::GetString(root, "chassis.serial_device"), "/dev/ttyUSB0");
  EXPECT_EQ(ConfigLoader::GetInt(root, "chassis.serial_baud"), 9600);
}

TEST(ConfigLoaderTest, MissingRequiredThrows) {
  ConfigLoader l;
  l.RegisterField({"must.have", "string", true});
  EXPECT_THROW(l.LoadFromString("other: x\n"), ConfigError);
}

TEST(ConfigLoaderTest, OptionalMissingOk) {
  ConfigLoader l;
  l.RegisterField({"opt.x", "int", false, "42"});
  EXPECT_NO_THROW(l.LoadFromString("other: y\n"));
  EXPECT_EQ(ConfigLoader::GetInt(YAML::Node{}, "opt.x", 99), 99);  // 调用方 fallback
}

TEST(ConfigLoaderTest, TypeMismatchThrows) {
  ConfigLoader l;
  l.RegisterField({"x", "int", true});
  EXPECT_THROW(l.LoadFromString("x: hello\n"), ConfigError);
}

TEST(ConfigLoaderTest, GetStringReturnsFallbackForMissing) {
  EXPECT_EQ(ConfigLoader::GetString(YAML::Node{}, "absent", "fb"), "fb");
  EXPECT_EQ(ConfigLoader::GetInt(YAML::Node{}, "absent", 7), 7);
  EXPECT_DOUBLE_EQ(ConfigLoader::GetDouble(YAML::Node{}, "absent", 0.5), 0.5);
}

TEST(ConfigLoaderTest, GetBoolVariants) {
  auto root = YAML::Load("a: true\nb: false\nc: 1\n");
  EXPECT_TRUE(ConfigLoader::GetBool(root, "a", false));
  EXPECT_FALSE(ConfigLoader::GetBool(root, "b", true));
  EXPECT_TRUE(ConfigLoader::GetBool(root, "c", false));  // 1 → true
}

TEST(ConfigLoaderTest, LoadFromFileNotFoundThrows) {
  ConfigLoader l;
  EXPECT_THROW(l.LoadFromFile("/no/such/file"), ConfigError);
}

TEST(ConfigLoaderTest, LoadFromFileHappy) {
  auto p = WriteTemp("x: 1\n");
  ConfigLoader l;
  l.RegisterField({"x", "int", true});
  auto root = l.LoadFromFile(p);
  EXPECT_EQ(ConfigLoader::GetInt(root, "x"), 1);
  std::remove(p.c_str());
}

TEST(ConfigLoaderTest, HotReloadTriggersOnChange) {
  auto p = WriteTemp("x: 1\n");
  int call_count = 0;
  HotReloadWatcher w(p, [&](const YAML::Node&) { ++call_count; });
  w.Tick();   // 初始化
  EXPECT_EQ(call_count, 0);
  w.Tick();   // 无变化
  EXPECT_EQ(call_count, 0);

  // 改文件
  std::this_thread::sleep_for(20ms);
  std::ofstream out(p, std::ios::trunc);
  out << "x: 2\n";
  out.close();
  w.Tick();
  EXPECT_EQ(call_count, 1);
  EXPECT_TRUE(w.reloaded());
  std::remove(p.c_str());
}

TEST(ConfigLoaderTest, ObserverNotifiedOnRuntimeConfigChange) {
  ConfigLoader loader;
  std::string observed_key;
  std::string observed_val;
  int notify_count = 0;

  loader.Subscribe("dispatcher.comparator", [&](const std::string& key, const std::string& val) {
    observed_key = key;
    observed_val = val;
    notify_count++;
  });

  // 1. 触发订阅项变更
  loader.NotifyChange("dispatcher.comparator", "EarliestDeadlineFirst");
  EXPECT_EQ(notify_count, 1);
  EXPECT_EQ(observed_key, "dispatcher.comparator");
  EXPECT_EQ(observed_val, "EarliestDeadlineFirst");

  // 2. 触发不相关项变更 -> 不被该观察者捕获
  loader.NotifyChange("chassis.baud", "115200");
  EXPECT_EQ(notify_count, 1);
}
