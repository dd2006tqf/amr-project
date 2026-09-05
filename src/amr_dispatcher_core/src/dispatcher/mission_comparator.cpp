#include "amr_dispatcher_core/dispatcher/mission_comparator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <string_view>

namespace amr_dispatcher_core::dispatcher {
namespace {

constexpr std::int64_t kNoDeadline = std::numeric_limits<std::int64_t>::max();

// 无截止期 → 视为无限远，EDF 下垫底。
std::int64_t EffectiveDeadline(const Mission& m) {
  return m.deadline_unix_ms > 0 ? m.deadline_unix_ms : kNoDeadline;
}

}  // namespace

bool PriorityFifoComparator::Before(const Mission& a, const Mission& b) const {
  if (a.priority != b.priority) {
    return a.priority > b.priority;
  }
  return a.sequence < b.sequence;
}

bool DeadlineComparator::Before(const Mission& a, const Mission& b) const {
  const std::int64_t da = EffectiveDeadline(a);
  const std::int64_t db = EffectiveDeadline(b);
  if (da != db) {
    return da < db;
  }
  return a.sequence < b.sequence;
}

bool ShortestJobComparator::Before(const Mission& a, const Mission& b) const {
  if (a.expected_seconds != b.expected_seconds) {
    return a.expected_seconds < b.expected_seconds;
  }
  return a.sequence < b.sequence;
}

WeightedScoreComparator::WeightedScoreComparator(Weights weights) : weights_(weights) {
  weights_.horizon_ms = std::max<std::int64_t>(1, weights_.horizon_ms);
}

double WeightedScoreComparator::Score(const Mission& m, const std::int64_t now_unix_ms) const {
  double deadline_score = 0.0;
  if (m.deadline_unix_ms > 0) {
    const double remaining = static_cast<double>(m.deadline_unix_ms - now_unix_ms);
    // 已过期 → 1.0；剩余 >= horizon → 0.0；线性内插。
    const double ratio = std::clamp(remaining / static_cast<double>(weights_.horizon_ms), 0.0, 1.0);
    deadline_score = 1.0 - ratio;
  }
  const double expected = std::max(0, m.expected_seconds);
  const double speed_score = 1.0 / (1.0 + expected);
  return weights_.priority * static_cast<double>(m.priority) +
         weights_.deadline * deadline_score + weights_.expected_time * speed_score;
}

bool WeightedScoreComparator::Before(const Mission& a, const Mission& b) const {
  // 注意：Before 无时钟，deadline 得分差异退化为「是否有截止期」。
  // 精确到期评估由调度器在 pop 时调用 Score(m, now) 完成；这里只做保守序。
  const bool a_has = a.deadline_unix_ms > 0;
  const bool b_has = b.deadline_unix_ms > 0;
  if (a_has != b_has) {
    return a_has;  // 有截止期的优先
  }
  if (a_has && b_has && a.deadline_unix_ms != b.deadline_unix_ms) {
    return a.deadline_unix_ms < b.deadline_unix_ms;
  }
  if (a.priority != b.priority) {
    return a.priority > b.priority;
  }
  return a.sequence < b.sequence;
}

namespace {

using ComparatorFactory = std::function<std::unique_ptr<MissionComparator>()>;

const std::array<std::pair<std::string_view, ComparatorFactory>, 4>& FactoryRegistry() {
  static const std::array<std::pair<std::string_view, ComparatorFactory>, 4> registry{{
      {"priority_fifo", [] { return std::make_unique<PriorityFifoComparator>(); }},
      {"earliest_deadline", [] { return std::make_unique<DeadlineComparator>(); }},
      {"shortest_job", [] { return std::make_unique<ShortestJobComparator>(); }},
      {"weighted_score", [] { return std::make_unique<WeightedScoreComparator>(); }},
  }};
  return registry;
}

}  // namespace

std::unique_ptr<MissionComparator> CreateMissionComparator(const std::string& name) {
  for (const auto& [registered_name, factory] : FactoryRegistry()) {
    if (registered_name == name) {
      return factory();
    }
  }
  return nullptr;
}

std::vector<std::string> RegisteredComparatorNames() {
  std::vector<std::string> names;
  names.reserve(FactoryRegistry().size());
  for (const auto& [name, factory] : FactoryRegistry()) {
    names.emplace_back(name);
  }
  return names;
}

}  // namespace amr_dispatcher_core::dispatcher
