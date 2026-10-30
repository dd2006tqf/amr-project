#include "amr_dispatcher_tools/http_limits.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace amr_dispatcher_tools;

// ---------------------------------------------------------------------------
// Content-Length 解析：旧实现（std::stoi）在这里会抛 std::invalid_argument
// 并让整个网关进程 terminate。以下每条都是实测打死过网关的输入。
// ---------------------------------------------------------------------------

TEST(HttpLimitsTest, ParseContentLength_ValidValueIsAccepted) {
  std::size_t out = 0;
  EXPECT_EQ(ParseContentLength("128", &out), ContentLengthStatus::kOk);
  EXPECT_EQ(out, 128u);
}

TEST(HttpLimitsTest, ParseContentLength_TrimsWhitespace) {
  std::size_t out = 0;
  EXPECT_EQ(ParseContentLength(" 42 \r\n", &out), ContentLengthStatus::kOk);
  EXPECT_EQ(out, 42u);
}

TEST(HttpLimitsTest, ParseContentLength_ZeroIsValid) {
  std::size_t out = 99;
  EXPECT_EQ(ParseContentLength("0", &out), ContentLengthStatus::kOk);
  EXPECT_EQ(out, 0u);
}

TEST(HttpLimitsTest, ParseContentLength_NonNumericIsInvalid) {
  std::size_t out = 0;
  EXPECT_EQ(ParseContentLength("abc", &out), ContentLengthStatus::kInvalid);
  EXPECT_EQ(ParseContentLength("12a", &out), ContentLengthStatus::kInvalid);
  EXPECT_EQ(ParseContentLength("1 2", &out), ContentLengthStatus::kInvalid);
  EXPECT_EQ(ParseContentLength("0x10", &out), ContentLengthStatus::kInvalid);
  EXPECT_EQ(ParseContentLength("1.5", &out), ContentLengthStatus::kInvalid);
}

TEST(HttpLimitsTest, ParseContentLength_NegativeIsInvalid) {
  std::size_t out = 0;
  EXPECT_EQ(ParseContentLength("-1", &out), ContentLengthStatus::kInvalid);
  EXPECT_EQ(out, 0u);
}

TEST(HttpLimitsTest, ParseContentLength_OverflowIsInvalid) {
  std::size_t out = 0;
  // 超出 uint64 的单个数（20 位全是 9）在累加时就会判溢出
  EXPECT_EQ(ParseContentLength("999999999999999999999999999999999999999999", &out),
            ContentLengthStatus::kInvalid);
}

TEST(HttpLimitsTest, ParseContentLength_AboveLimitIsTooLarge) {
  std::size_t out = 0;
  const std::string too_big = std::to_string(kMaxRequestBytes + 1);
  EXPECT_EQ(ParseContentLength(too_big, &out), ContentLengthStatus::kTooLarge);
}

TEST(HttpLimitsTest, ParseContentLength_ExactlyLimitIsOk) {
  std::size_t out = 0;
  EXPECT_EQ(ParseContentLength(std::to_string(kMaxRequestBytes), &out),
            ContentLengthStatus::kOk);
  EXPECT_EQ(out, kMaxRequestBytes);
}

TEST(HttpLimitsTest, ParseContentLength_MissingWhenEmpty) {
  std::size_t out = 7;
  EXPECT_EQ(ParseContentLength("", &out), ContentLengthStatus::kMissing);
  EXPECT_EQ(ParseContentLength("   ", &out), ContentLengthStatus::kMissing);
}

// ---------------------------------------------------------------------------
// 路径与路由
// ---------------------------------------------------------------------------

TEST(HttpLimitsTest, RequestPathOnly_StripsQuery) {
  EXPECT_EQ(RequestPathOnly("/api/v1/orders?id=m1&from=A"), "/api/v1/orders");
}

TEST(HttpLimitsTest, RequestPathOnly_NoQueryUnchanged) {
  EXPECT_EQ(RequestPathOnly("/api/v1/state"), "/api/v1/state");
}

TEST(HttpLimitsTest, RequestPathOnly_StripsFragment) {
  EXPECT_EQ(RequestPathOnly("/api/v1/state#top"), "/api/v1/state");
}

TEST(HttpLimitsTest, RequestPathOnly_HandlesAbsoluteForm) {
  EXPECT_EQ(RequestPathOnly("http://127.0.0.1:8080/api/v1/state?x=1"), "/api/v1/state");
  EXPECT_EQ(RequestPathOnly("http://127.0.0.1:8080"), "/");
}

TEST(HttpLimitsTest, RequestQueryOnly_ReturnsQueryWithoutQuestionMark) {
  EXPECT_EQ(RequestQueryOnly("/api/v1/orders?id=m1&prio=5"), "id=m1&prio=5");
  EXPECT_EQ(RequestQueryOnly("/api/v1/state"), "");
}

TEST(HttpLimitsTest, PathMatches_RequiresExactMatch) {
  EXPECT_TRUE(PathMatches("/api/v1/orders", "/api/v1/orders"));
  // 旧实现用 find() 子串匹配，下面三条都会被误判为订单路由
  EXPECT_FALSE(PathMatches("/api/v1/ordersXYZ", "/api/v1/orders"));
  EXPECT_FALSE(PathMatches("/api/v1/orders/extra", "/api/v1/orders"));
  EXPECT_FALSE(PathMatches("/x/api/v1/orders", "/api/v1/orders"));
}

TEST(HttpLimitsTest, PathMatchesAny_MatchesEither) {
  EXPECT_TRUE(PathMatchesAny("/a", "/a", "/b"));
  EXPECT_TRUE(PathMatchesAny("/b", "/a", "/b"));
  EXPECT_FALSE(PathMatchesAny("/c", "/a", "/b"));
}

// ---------------------------------------------------------------------------
// 头部解析
// ---------------------------------------------------------------------------

TEST(HttpLimitsTest, FindHeaderValue_IsCaseInsensitive) {
  const std::string head =
      "GET / HTTP/1.1\r\nHost: x\r\ncontent-length: 12\r\n\r\n";
  std::string value;
  EXPECT_TRUE(FindHeaderValue(head, "Content-Length", &value));
  EXPECT_EQ(value, "12");
}

TEST(HttpLimitsTest, FindHeaderValue_MissingReturnsFalse) {
  const std::string head = "GET / HTTP/1.1\r\nHost: x\r\n\r\n";
  std::string value = "stale";
  EXPECT_FALSE(FindHeaderValue(head, "Content-Length", &value));
  EXPECT_TRUE(value.empty());
}

TEST(HttpLimitsTest, FindHeaderValue_DoesNotMatchLongerFieldName) {
  const std::string head = "GET / HTTP/1.1\r\nContent-Length-Extra: 9\r\n\r\n";
  std::string value;
  EXPECT_FALSE(FindHeaderValue(head, "Content-Length", &value));
}

TEST(HttpLimitsTest, HasChunkedTransferEncoding_DetectsChunked) {
  const std::string head = "POST / HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n";
  EXPECT_TRUE(HasChunkedTransferEncoding(head));
  const std::string plain = "POST / HTTP/1.1\r\nContent-Length: 3\r\n\r\n";
  EXPECT_FALSE(HasChunkedTransferEncoding(plain));
}

TEST(HttpLimitsTest, Limits_AreSane) {
  EXPECT_GT(kMaxRequestBytes, 64U * 1024U);   // 地图上传要放得下
  EXPECT_LT(kMaxHeaderBytes, kMaxRequestBytes);
  EXPECT_GT(kSocketTimeoutSec, 0);
  // accept 线程的读头预算必须远小于连接超时：它是单点，为慢客户端等满
  // kSocketTimeoutSec 会让所有新连接（包括探针）一起排队。
  // 反过来也不能太短：本机环回一次 read 通常在 1ms 量级。
  EXPECT_GT(kAcceptReadBudgetMs, 0);
  EXPECT_LT(kAcceptReadBudgetMs, kSocketTimeoutSec * 1000 / 10);
}

// ---------------------------------------------------------------------------
// prio 等整型查询参数：旧实现 std::stoi 对 "notanumber" 直接 terminate
// ---------------------------------------------------------------------------

TEST(HttpLimitsTest, ParseInt32Strict_ValidValues) {
  std::int32_t out = 0;
  EXPECT_EQ(ParseInt32Strict("5", &out), IntParseStatus::kOk);
  EXPECT_EQ(out, 5);
  EXPECT_EQ(ParseInt32Strict("-7", &out), IntParseStatus::kOk);
  EXPECT_EQ(out, -7);
  EXPECT_EQ(ParseInt32Strict("+9", &out), IntParseStatus::kOk);
  EXPECT_EQ(out, 9);
  EXPECT_EQ(ParseInt32Strict(" 12 ", &out), IntParseStatus::kOk);
  EXPECT_EQ(out, 12);
}

TEST(HttpLimitsTest, ParseInt32Strict_NonNumericIsInvalid) {
  std::int32_t out = 0;
  EXPECT_EQ(ParseInt32Strict("notanumber", &out), IntParseStatus::kInvalid);
  EXPECT_EQ(ParseInt32Strict("5x", &out), IntParseStatus::kInvalid);  // 不许静默截断
  EXPECT_EQ(ParseInt32Strict("", &out), IntParseStatus::kInvalid);
  EXPECT_EQ(ParseInt32Strict("-", &out), IntParseStatus::kInvalid);
  EXPECT_EQ(ParseInt32Strict("1.5", &out), IntParseStatus::kInvalid);
}

TEST(HttpLimitsTest, ParseInt32Strict_OutOfRangeDetected) {
  std::int32_t out = 0;
  EXPECT_EQ(ParseInt32Strict("2147483648", &out), IntParseStatus::kOutOfRange);
  EXPECT_EQ(ParseInt32Strict("-2147483649", &out), IntParseStatus::kOutOfRange);
  EXPECT_EQ(ParseInt32Strict("2147483647", &out), IntParseStatus::kOk);
  EXPECT_EQ(ParseInt32Strict("-2147483648", &out), IntParseStatus::kOk);
}
