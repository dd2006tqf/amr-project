#include "amr_dispatcher_tools/metrics_format.hpp"

#include <gtest/gtest.h>

#include <limits>

using amr_dispatcher_tools::EscapeMetricHelp;
using amr_dispatcher_tools::EscapeMetricLabel;
using amr_dispatcher_tools::FormatMetricValue;
using amr_dispatcher_tools::MetricFamily;
using amr_dispatcher_tools::MetricSample;
using amr_dispatcher_tools::MetricType;
using amr_dispatcher_tools::RenderMetricFamilies;
using amr_dispatcher_tools::RenderMetricSample;

TEST(MetricsFormatTest, IntegerValuesHaveNoDecimalPoint) {
  EXPECT_EQ(FormatMetricValue(0.0), "0");
  EXPECT_EQ(FormatMetricValue(1.0), "1");
  EXPECT_EQ(FormatMetricValue(-42.0), "-42");
  EXPECT_EQ(FormatMetricValue(1e6), "1000000");
}

TEST(MetricsFormatTest, FractionalValuesUseShortestForm) {
  EXPECT_EQ(FormatMetricValue(0.5), "0.5");
  EXPECT_EQ(FormatMetricValue(12.25), "12.25");
  // 不出现 1e+06 这类科学计数法（抓取端虽认，但 diff 与人工核对都难读）
  EXPECT_EQ(FormatMetricValue(1234567.5), "1234567.5");
}

TEST(MetricsFormatTest, NonFiniteValuesUseAsciiLiterals) {
  EXPECT_EQ(FormatMetricValue(std::numeric_limits<double>::quiet_NaN()), "NaN");
  EXPECT_EQ(FormatMetricValue(std::numeric_limits<double>::infinity()), "+Inf");
  EXPECT_EQ(FormatMetricValue(-std::numeric_limits<double>::infinity()), "-Inf");
}

// 极大值不得触发 long long 越界转换（UB）。
TEST(MetricsFormatTest, HugeValuesDoNotOverflowIntegerCast) {
  const std::string rendered = FormatMetricValue(1e300);
  EXPECT_FALSE(rendered.empty());
  EXPECT_NE(rendered.front(), '-');
  const std::string negative = FormatMetricValue(-1e300);
  EXPECT_EQ(negative.front(), '-');
}

TEST(MetricsFormatTest, LabelEscaping) {
  EXPECT_EQ(EscapeMetricLabel("plain"), "plain");
  EXPECT_EQ(EscapeMetricLabel("a\\b"), "a\\\\b");
  EXPECT_EQ(EscapeMetricLabel("say \"hi\""), "say \\\"hi\\\"");
  EXPECT_EQ(EscapeMetricLabel("line1\nline2"), "line1\\nline2");
}

// HELP 是行内文本：换行会截断 HELP 并让下一行变成非法样本。
TEST(MetricsFormatTest, HelpEscaping) {
  EXPECT_EQ(EscapeMetricHelp("plain help"), "plain help");
  EXPECT_EQ(EscapeMetricHelp("line1\nline2"), "line1\\nline2");
  EXPECT_EQ(EscapeMetricHelp("back\\slash"), "back\\\\slash");
  // 双引号在 HELP 里合法，不转义
  EXPECT_EQ(EscapeMetricHelp("say \"hi\""), "say \"hi\"");
}

TEST(MetricsFormatTest, SampleWithoutLabels) {
  MetricSample sample;
  sample.value = 3.0;
  EXPECT_EQ(RenderMetricSample("amr_queue_size", sample), "amr_queue_size 3");
}

// vehicle_id 来自配置，可能含引号/反斜杠：不转义会让整份 exposition 解析失败。
TEST(MetricsFormatTest, SampleWithLabelsEscapesValues) {
  MetricSample sample;
  sample.labels = {{"vehicle_id", "amr_\"01\""}};
  sample.value = 1.0;
  EXPECT_EQ(RenderMetricSample("amr_vehicle_online", sample),
            "amr_vehicle_online{vehicle_id=\"amr_\\\"01\\\"\"} 1");
}

TEST(MetricsFormatTest, FamilyRendersHelpAndType) {
  MetricFamily family;
  family.name = "amr_active_missions";
  family.help = "Number of missions currently executing";
  family.type = MetricType::kGauge;
  family.samples.push_back(MetricSample{{}, 2.0});
  const std::string rendered = RenderMetricFamilies({family});
  EXPECT_EQ(rendered,
            "# HELP amr_active_missions Number of missions currently executing\n"
            "# TYPE amr_active_missions gauge\n"
            "amr_active_missions 2\n");
}

TEST(MetricsFormatTest, CounterTypeRendersAsCounter) {
  MetricFamily family;
  family.name = "amr_dispatch_total";
  family.help = "Dispatched missions";
  family.type = MetricType::kCounter;
  family.samples.push_back(MetricSample{{}, 7.0});
  const std::string rendered = RenderMetricFamilies({family});
  EXPECT_NE(rendered.find("# TYPE amr_dispatch_total counter\n"), std::string::npos);
}

TEST(MetricsFormatTest, EmptyFamilyIsOmitted) {
  MetricFamily family;
  family.name = "amr_vehicle_online";
  family.help = "Per-vehicle online flag";
  // 没有样本（例如未配置受管车）：不输出 HELP/TYPE，避免空族
  EXPECT_TRUE(RenderMetricFamilies({family}).empty());
}

TEST(MetricsFormatTest, EmptyInputRendersEmpty) {
  EXPECT_TRUE(RenderMetricFamilies({}).empty());
}

TEST(MetricsFormatTest, MultipleFamiliesKeepOrder) {
  MetricFamily a;
  a.name = "amr_a";
  a.help = "A";
  a.samples.push_back(MetricSample{{}, 1.0});
  MetricFamily b;
  b.name = "amr_b";
  b.help = "B";
  b.samples.push_back(MetricSample{{}, 2.0});
  const std::string rendered = RenderMetricFamilies({a, b});
  EXPECT_LT(rendered.find("# HELP amr_a"), rendered.find("# HELP amr_b"));
}
