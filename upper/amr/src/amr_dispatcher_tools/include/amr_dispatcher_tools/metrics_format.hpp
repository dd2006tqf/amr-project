// Prometheus 文本格式渲染（纯函数）。
//
// 为什么单列一个头文件：`/metrics` 的输出是**对外契约**，一旦被 Grafana /
// 抓取规则依赖，改字段名就是破坏性变更。字段名、转义、HELP/TYPE 行、以及
// "标签值必须转义"这条最容易踩的规则，都应该能在宿主 ctest 里断言，
// 而不是靠 `curl | grep` 肉眼看。
//
// 约定（与 Prometheus exposition format 一致）：
//   * 每个指标先写 `# HELP` / `# TYPE`，再写样本；
//   * 标签值里的 \ " \n 必须转义（否则抓取端解析失败）；
//   * 浮点统一用 `%g` 风格的最短表示（无穷/NaN 直接输出字面量，不用 1e+308 之类）；
//   * 同一指标的所有样本标签集必须一致（调用方负责，本层不做推断）。
#pragma once

#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace amr_dispatcher_tools {

enum class MetricType {
  kCounter,  // 单调递增
  kGauge,    // 可增可减
};

inline const char* MetricTypeName(MetricType type) {
  return type == MetricType::kCounter ? "counter" : "gauge";
}

// 标签键值转义：反斜杠、双引号、换行。
inline std::string EscapeMetricLabel(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

// 浮点渲染：整数值不带小数点（避免 `1` 与 `1.0` 两种写法混用导致 diff 抖动）。
inline std::string FormatMetricValue(double value) {
  // 不是有限值：按 exposition 格式写 ASCII 字面量（抓取端认这个）
  if (std::isnan(value)) {
    return "NaN";
  }
  if (std::isinf(value)) {
    return value > 0 ? "+Inf" : "-Inf";
  }
  // 整数且落在 long long 可表示范围内 -> 只输出整数部分。
  // 越界时不做转换（转换越界是未定义行为），直接走 %.10g。
  constexpr double kSafeIntegerLimit = 9.0e18;
  if (std::fabs(value) < kSafeIntegerLimit) {
    const auto as_integer = static_cast<long long>(value);
    if (static_cast<double>(as_integer) == value) {
      return std::to_string(as_integer);
    }
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.10g", value);
  return buffer;
}

// HELP 文本转义：反斜杠与换行（HELP 是行内文本，换行会毁掉整份 exposition）。
// 双引号在 HELP 里不需要转义。
inline std::string EscapeMetricHelp(const std::string& text) {
  std::string out;
  out.reserve(text.size());
  for (const char c : text) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

// 一条样本：标签 + 值。指标名由所属 MetricFamily 提供——
// 双份 name 容易在渲染时写出"族名 A、样本名 B"的非法 exposition，
// 索性只留一处。
struct MetricSample {
  std::vector<std::pair<std::string, std::string>> labels;
  double value = 0.0;
};

// 一组指标（同 name 的样本共享 HELP/TYPE）。
struct MetricFamily {
  std::string name;
  std::string help;
  MetricType type = MetricType::kGauge;
  std::vector<MetricSample> samples;
};

inline std::string RenderMetricSample(const std::string& name, const MetricSample& sample) {
  std::string out = name;
  if (!sample.labels.empty()) {
    out += "{";
    for (std::size_t i = 0; i < sample.labels.size(); ++i) {
      if (i) {
        out += ",";
      }
      out += sample.labels[i].first;
      out += "=\"";
      out += EscapeMetricLabel(sample.labels[i].second);
      out += "\"";
    }
    out += "}";
  }
  out += " ";
  out += FormatMetricValue(sample.value);
  return out;
}

// 渲染整份 exposition（含尾部换行；空族不输出）。
inline std::string RenderMetricFamilies(const std::vector<MetricFamily>& families) {
  std::string out;
  for (const auto& family : families) {
    if (family.samples.empty()) {
      continue;  // 没有样本就不声明 HELP/TYPE（否则抓取端会保留一条空族）
    }
    out += "# HELP " + family.name + " " + EscapeMetricHelp(family.help) + "\n";
    out += "# TYPE " + family.name + " " + MetricTypeName(family.type) + "\n";
    for (const auto& sample : family.samples) {
      out += RenderMetricSample(family.name, sample) + "\n";
    }
  }
  return out;
}

}  // namespace amr_dispatcher_tools
