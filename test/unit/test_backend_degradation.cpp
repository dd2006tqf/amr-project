#include "amr_dispatcher_core/chassis/backend_degradation.hpp"

#include <gtest/gtest.h>

#include <thread>

using namespace amr_dispatcher_core::chassis;
using namespace std::chrono_literals;

namespace {

class StubBackend : public ChassisBackend {
 public:
  explicit StubBackend(std::string name) : name_(std::move(name)) {}
  std::string Name() const override { return name_; }
  bool Open(std::string* /*err*/) override { opened_ = true; return true; }
  void Close() override { opened_ = false; }
  bool IsOpen() const override { return opened_; }
  bool WriteCommand(const ChassisCommand&, std::string*) override { return true; }
  std::optional<std::string> Read(std::string*) override { return std::nullopt; }
  bool opened_ = false;
 private:
  std::string name_;
};

}  // namespace

TEST(BackendDegradationTest, DefaultsToFirstBackend) {
  std::vector<BackendDescriptor> descs{
      {"first", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("first"); }},
      {"second", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("second"); }},
  };
  BackendDegradationController ctrl(std::move(descs), {});
  ASSERT_NE(ctrl.current(), nullptr);
  EXPECT_EQ(ctrl.current()->Name(), "first");
  EXPECT_EQ(ctrl.current_index(), 0u);
}

TEST(BackendDegradationTest, HighLossRateTriggersDemotion) {
  std::vector<BackendDescriptor> descs{
      {"first", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("first"); }},
      {"second", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("second"); }},
  };
  BackendDegradationController ctrl(
      std::move(descs), {},
      {.warmup_samples = 4, .demote_loss_rate = 0.3, .cooldown = 0ms});
  // warmup 中不触发
  ctrl.RecordLoss();
  std::string reason;
  EXPECT_FALSE(ctrl.EvaluateAndMaybeSwitch(&reason));
  // 喂入足够样本：3 成功 + 5 丢包 → loss_rate = 5/8 = 0.625 > 0.3
  for (int i = 0; i < 3; ++i) ctrl.RecordSuccess(1.0);
  for (int i = 0; i < 5; ++i) ctrl.RecordLoss();
  EXPECT_TRUE(ctrl.EvaluateAndMaybeSwitch(&reason));
  EXPECT_EQ(ctrl.current_index(), 1u);
  EXPECT_EQ(ctrl.current()->Name(), "second");
}

TEST(BackendDegradationTest, CooldownPreventsRapidFlip) {
  std::vector<BackendDescriptor> descs{
      {"first", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("first"); }},
      {"second", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("second"); }},
      {"third", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("third"); }},
  };
  BackendDegradationController ctrl(
      std::move(descs), {},
      {.warmup_samples = 2, .demote_loss_rate = 0.3, .cooldown = 1000ms});
  for (int i = 0; i < 3; ++i) ctrl.RecordSuccess(1.0);
  for (int i = 0; i < 5; ++i) ctrl.RecordLoss();
  EXPECT_TRUE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  // 立刻再触发：cooldown 阻止
  for (int i = 0; i < 5; ++i) ctrl.RecordLoss();
  EXPECT_FALSE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  EXPECT_EQ(ctrl.current_index(), 1u);
}

TEST(BackendDegradationTest, SwitchToKnownIndexClosesOld) {
  std::vector<BackendDescriptor> descs{
      {"a", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("a"); }},
      {"b", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("b"); }},
  };
  BackendDegradationController ctrl(std::move(descs), {});
  auto* old = static_cast<StubBackend*>(ctrl.current().get());
  EXPECT_TRUE(old->opened_);
  std::string r;
  EXPECT_TRUE(ctrl.SwitchTo(1, &r));
  EXPECT_FALSE(old->opened_);  // 已关闭
  EXPECT_TRUE(r.find("b") != std::string::npos);
  EXPECT_EQ(ctrl.current()->Name(), "b");
}

TEST(BackendDegradationTest, SwitchToInvalidReturnsFalse) {
  std::vector<BackendDescriptor> descs{
      {"a", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("a"); }},
  };
  BackendDegradationController ctrl(std::move(descs), {});
  std::string r;
  EXPECT_FALSE(ctrl.SwitchTo(99, &r));
}

// 链路恢复（连续成功达到 success_threshold 触发恢复信号）后自动回切首选后端。
TEST(BackendDegradationTest, FailbackToPreferredAfterRecoverySignal) {
  std::vector<BackendDescriptor> descs{
      {"first", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("first"); }},
      {"second", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("second"); }},
  };
  BackendDegradationController ctrl(
      std::move(descs), {},
      {.warmup_samples = 4, .demote_loss_rate = 0.3, .cooldown = 0ms});
  // 降级到 second：3 成功 + 5 丢包 -> loss_rate = 0.625 > 0.3
  for (int i = 0; i < 3; ++i) ctrl.RecordSuccess(1.0);
  for (int i = 0; i < 5; ++i) ctrl.RecordLoss();
  ASSERT_TRUE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  ASSERT_EQ(ctrl.current_index(), 1u);
  // 链路恢复：切档后滑窗重置，连续成功达到 success_threshold（默认 50）发出恢复信号
  for (int i = 0; i < 50; ++i) ctrl.RecordSuccess(1.0);
  // 自动回切首选后端
  EXPECT_TRUE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  EXPECT_EQ(ctrl.current_index(), 0u);
  EXPECT_EQ(ctrl.current()->Name(), "first");
}

// 冷却窗内禁止回切：降档后冷却期内即使出现恢复信号也不许切换；窗口结束后信号仍在，回切成功。
TEST(BackendDegradationTest, FailbackBlockedDuringCooldown) {
  std::vector<BackendDescriptor> descs{
      {"first", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("first"); }},
      {"second", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("second"); }},
  };
  BackendDegradationController ctrl(
      std::move(descs), {},
      {.warmup_samples = 4, .demote_loss_rate = 0.3, .cooldown = 100ms});
  for (int i = 0; i < 3; ++i) ctrl.RecordSuccess(1.0);
  for (int i = 0; i < 5; ++i) ctrl.RecordLoss();
  ASSERT_TRUE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  ASSERT_EQ(ctrl.current_index(), 1u);
  // 冷却窗内链路恢复：恢复信号已就绪，但不允许回切
  for (int i = 0; i < 50; ++i) ctrl.RecordSuccess(1.0);
  EXPECT_FALSE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  EXPECT_EQ(ctrl.current_index(), 1u);
  EXPECT_EQ(ctrl.current()->Name(), "second");
  // 窗口结束后：恢复信号被保留（未在冷却期被白白消费），回切随即成功
  std::this_thread::sleep_for(150ms);
  EXPECT_TRUE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  EXPECT_EQ(ctrl.current_index(), 0u);
  EXPECT_EQ(ctrl.current()->Name(), "first");
}

// Open 可按需失败的后端桩：回切目标设备故障时用于验证降级保护。
class FlakyBackend : public ChassisBackend {
 public:
  FlakyBackend(std::string name, std::shared_ptr<bool> open_ok)
      : name_(std::move(name)), open_ok_(std::move(open_ok)) {}
  std::string Name() const override { return name_; }
  bool Open(std::string* /*err*/) override {
    if (open_ok_ && !*open_ok_) return false;
    opened_ = true;
    return true;
  }
  void Close() override { opened_ = false; }
  bool IsOpen() const override { return opened_; }
  bool WriteCommand(const ChassisCommand&, std::string*) override { return true; }
  std::optional<std::string> Read(std::string*) override { return std::nullopt; }

 private:
  std::string name_;
  std::shared_ptr<bool> open_ok_;
  bool opened_ = false;
};

// 回切目标（首选后端）打不开时：必须留在当前可用后端，不许把连接搞成空档。
TEST(BackendDegradationTest, FailbackOpenFailureKeepsCurrentBackend) {
  auto first_open_ok = std::make_shared<bool>(true);
  std::vector<BackendDescriptor> descs{
      {"first",
       [first_open_ok](const ChassisBackendConfig&) {
         return std::make_unique<FlakyBackend>("first", first_open_ok);
       }},
      {"second", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("second"); }},
  };
  BackendDegradationController ctrl(
      std::move(descs), {},
      {.warmup_samples = 4, .demote_loss_rate = 0.3, .cooldown = 0ms});
  ASSERT_NE(ctrl.current(), nullptr);
  // 降级到 second
  for (int i = 0; i < 3; ++i) ctrl.RecordSuccess(1.0);
  for (int i = 0; i < 5; ++i) ctrl.RecordLoss();
  ASSERT_TRUE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  ASSERT_EQ(ctrl.current_index(), 1u);
  // 首选设备故障（打不开），链路同时恢复
  *first_open_ok = false;
  for (int i = 0; i < 50; ++i) ctrl.RecordSuccess(1.0);
  // 回切失败 -> 留在 second，连接不得断成空档
  EXPECT_FALSE(ctrl.EvaluateAndMaybeSwitch(nullptr));
  ASSERT_NE(ctrl.current(), nullptr);
  EXPECT_EQ(ctrl.current_index(), 1u);
  EXPECT_EQ(ctrl.current()->Name(), "second");
  EXPECT_TRUE(ctrl.current()->IsOpen());
}

// 连接丢失（首选后端开机不在位、Open 失败）后，按指数退避扫描重连到可用后端。
TEST(BackendDegradationTest, ReconnectsByScanningBackendsWhenConnectionLost) {
  auto first_open_ok = std::make_shared<bool>(false);  // 首选设备不在位
  std::vector<BackendDescriptor> descs{
      {"first",
       [first_open_ok](const ChassisBackendConfig&) {
         return std::make_unique<FlakyBackend>("first", first_open_ok);
       }},
      {"second", [](const ChassisBackendConfig&) { return std::make_unique<StubBackend>("second"); }},
  };
  BackendDegradationController ctrl(std::move(descs), {});
  // 开机即连接丢失：构造时 SwitchTo(0) 失败
  ASSERT_EQ(ctrl.current(), nullptr);

  // 驱动评估循环：默认 initial_backoff=200ms，等退避窗口过去后应完成重连
  bool connected = false;
  for (int i = 0; i < 40 && !connected; ++i) {
    connected = ctrl.EvaluateAndMaybeSwitch(nullptr);
    if (!connected) std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(connected);
  ASSERT_NE(ctrl.current(), nullptr);
  EXPECT_EQ(ctrl.current()->Name(), "second");
  EXPECT_EQ(ctrl.current_index(), 1u);
  EXPECT_TRUE(ctrl.current()->IsOpen());
}
