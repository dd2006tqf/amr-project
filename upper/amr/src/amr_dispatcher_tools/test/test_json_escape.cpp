#include "amr_dispatcher_tools/json_escape.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace amr_dispatcher_tools;

TEST(JsonEscapeTest, PlainTextUnchanged) {
  EXPECT_EQ(JsonEscape("mission_1"), "mission_1");
  EXPECT_EQ(JsonEscape(""), "");
}

TEST(JsonEscapeTest, QuotesAndBackslashesEscaped) {
  EXPECT_EQ(JsonEscape("a\"b"), "a\\\"b");
  EXPECT_EQ(JsonEscape("a\\b"), "a\\\\b");
}

TEST(JsonEscapeTest, ControlCharactersEscaped) {
  EXPECT_EQ(JsonEscape("a\nb"), "a\\nb");
  EXPECT_EQ(JsonEscape("a\rb"), "a\\rb");
  EXPECT_EQ(JsonEscape("a\tb"), "a\\tb");
  EXPECT_EQ(JsonEscape(std::string("a\x01" "b")), "a\\u0001b");
}

TEST(JsonEscapeTest, Utf8BytesPreserved) {
  // 中文按 UTF-8 原样保留（JSON 允许非 ASCII 字节直出）
  const std::string zh = "车辆离线";
  EXPECT_EQ(JsonEscape(zh), zh);
}

TEST(JsonEscapeTest, EscapedOutputStaysSingleLine) {
  // 审计原因里带换行时，响应体不能因此断成多行
  const std::string escaped = JsonEscape("原因：\n车辆报废\r\n现场已确认");
  EXPECT_EQ(escaped.find('\n'), std::string::npos);
  EXPECT_EQ(escaped.find('\r'), std::string::npos);
}

TEST(JsonEscapeTest, StringPairProducesValidFragment) {
  EXPECT_EQ(JsonStringPair("message", "ok"), "\"message\":\"ok\"");
  EXPECT_EQ(JsonStringPair("reason", "say \"hi\""), "\"reason\":\"say \\\"hi\\\"\"");
}
