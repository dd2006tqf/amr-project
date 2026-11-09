#include "amr_dispatcher_core/chassis/backend_degradation.hpp"

#include <gtest/gtest.h>

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
