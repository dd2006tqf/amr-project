#pragma once

#include <memory>
#include <string>
#include <vector>

#include "amr_dispatcher_core/dispatcher/mission_state.hpp"

namespace amr_dispatcher_core::dispatcher {

// 可插拔队列排序策略（策略模式）。
// 内置实现见 mission_comparator.cpp；新增调度策略只需：
//   1) 继承 MissionComparator；
//   2) 在 CreateMissionComparator 工厂注册表加一行。
class MissionComparator {
 public:
  virtual ~MissionComparator() = default;

  virtual std::string name() const = 0;

  // 严格弱序：a 比 b 更应先出队时返回 true。
  // 实现必须保证自反/反对称/传递，否则 std::sort 行为未定义。
  // sequence 字段是各策略最终打破平局的兜底键。
  virtual bool Before(const Mission& a, const Mission& b) const = 0;
};

// priority 降序 → sequence 升序。默认策略，先到先服务同优先级。
class PriorityFifoComparator final : public MissionComparator {
 public:
  std::string name() const override { return "priority_fifo"; }
  bool Before(const Mission& a, const Mission& b) const override;
};

// 最早截止期优先（EDF）。deadline_unix_ms <= 0 视为无限远，排最后。
class DeadlineComparator final : public MissionComparator {
 public:
  std::string name() const override { return "earliest_deadline"; }
  bool Before(const Mission& a, const Mission& b) const override;
};

// 最短预估耗时优先（SJF），适合提升平均周转。
class ShortestJobComparator final : public MissionComparator {
 public:
  std::string name() const override { return "shortest_job"; }
  bool Before(const Mission& a, const Mission& b) const override;
};

// 加权得分 = w_p·priority + w_d·deadline_score + w_s·speed_score。
// 三个权重从配置注入，得分相同退回 FIFO。演示可扩展性卖点。
class WeightedScoreComparator final : public MissionComparator {
 public:
  struct Weights {
    double priority = 1.0;
    double deadline = 0.5;    // 距截止期越近得分越高
    double expected_time = 0.2;  // 预估耗时越短得分越高
    std::int64_t horizon_ms = 10 * 60 * 1000;  // 截止期得分归一化窗口
  };

  explicit WeightedScoreComparator(Weights weights);
  WeightedScoreComparator() : WeightedScoreComparator(Weights{}) {}

  std::string name() const override { return "weighted_score"; }
  bool Before(const Mission& a, const Mission& b) const override;

  // 暴露给测试：同序保证 score(a)==score(b) ⇔ !Before(a,b) && !Before(b,a)。
  double Score(const Mission& m, std::int64_t now_unix_ms) const;

 private:
  Weights weights_;
};

// 工厂：未知名称返回 nullptr。name 表：
//   "priority_fifo" | "earliest_deadline" | "shortest_job" | "weighted_score"
std::unique_ptr<MissionComparator> CreateMissionComparator(const std::string& name);

std::vector<std::string> RegisteredComparatorNames();

}  // namespace amr_dispatcher_core::dispatcher
