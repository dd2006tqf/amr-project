#include "amr_dispatcher_tools/bearer_auth.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace amr_dispatcher_tools;

TEST(BearerAuthTest, EmptyExpectedTokenDisablesAuth) {
  // 默认关闭：现网与三套 e2e 都不带 Authorization 头
  EXPECT_TRUE(IsAuthorized("", ""));
  EXPECT_TRUE(IsAuthorized("Bearer whatever", ""));
}

TEST(BearerAuthTest, CorrectTokenIsAccepted) {
  EXPECT_TRUE(IsAuthorized("Bearer s3cret", "s3cret"));
}

TEST(BearerAuthTest, SchemeIsCaseInsensitive) {
  EXPECT_TRUE(IsAuthorized("bearer s3cret", "s3cret"));
  EXPECT_TRUE(IsAuthorized("BEARER s3cret", "s3cret"));
}

TEST(BearerAuthTest, SurroundingWhitespaceTolerated) {
  EXPECT_TRUE(IsAuthorized("  Bearer   s3cret  ", "s3cret"));
}

TEST(BearerAuthTest, MissingHeaderIsRejected) {
  EXPECT_FALSE(IsAuthorized("", "s3cret"));
}

TEST(BearerAuthTest, WrongTokenIsRejected) {
  EXPECT_FALSE(IsAuthorized("Bearer wrong", "s3cret"));
  EXPECT_FALSE(IsAuthorized("Bearer s3cre", "s3cret"));
  EXPECT_FALSE(IsAuthorized("Bearer s3cretx", "s3cret"));
}

TEST(BearerAuthTest, WrongSchemeIsRejected) {
  EXPECT_FALSE(IsAuthorized("Basic s3cret", "s3cret"));
  EXPECT_FALSE(IsAuthorized("s3cret", "s3cret"));
}

TEST(BearerAuthTest, SchemePrefixWithoutSeparatorIsRejected) {
  EXPECT_FALSE(IsAuthorized("Bearers3cret", "s3cret"));
}

TEST(BearerAuthTest, EmptyBearerValueIsRejected) {
  EXPECT_FALSE(IsAuthorized("Bearer", "s3cret"));
  EXPECT_FALSE(IsAuthorized("Bearer   ", "s3cret"));
}

TEST(BearerAuthTest, ExtractBearerToken_ReturnsValue) {
  std::string token;
  EXPECT_TRUE(ExtractBearerToken("Bearer abc123", &token));
  EXPECT_EQ(token, "abc123");
}

TEST(BearerAuthTest, ConstantTimeEquals_HandlesPrefixesAndEmpty) {
  EXPECT_TRUE(ConstantTimeEquals("abc", "abc"));
  EXPECT_FALSE(ConstantTimeEquals("abc", "abd"));
  EXPECT_FALSE(ConstantTimeEquals("abc", "ab"));
  EXPECT_TRUE(ConstantTimeEquals("", ""));
}

TEST(BearerAuthTest, StaticAssetsAreExemptOnlyForRootPages) {
  EXPECT_TRUE(IsAuthExemptPath("/"));
  EXPECT_TRUE(IsAuthExemptPath("/index.html"));
  EXPECT_FALSE(IsAuthExemptPath("/api/v1/state"));
  EXPECT_FALSE(IsAuthExemptPath("/api/operator/estop"));
}

// 探针必须免鉴权：容器 HEALTHCHECK / 编排平台探针无法携带 token。
// 若这里变成需要 token，探针会恒 401 —— 把"进程死了"伪装成"探针永远不健康"。
TEST(BearerAuthTest, ProbesAreExemptButTopologyOpsAreNot) {
  EXPECT_TRUE(IsAuthExemptPath("/healthz"));
  EXPECT_TRUE(IsAuthExemptPath("/readyz"));
  // 拓扑运维端点会改调度行为，必须鉴权（与强清/恢复同档）
  EXPECT_FALSE(IsAuthExemptPath("/api/v1/topology/overlay"));
  EXPECT_FALSE(IsAuthExemptPath("/api/v1/topology/block_edge"));
  EXPECT_FALSE(IsAuthExemptPath("/api/v1/topology/one_way_edge"));
  EXPECT_FALSE(IsAuthExemptPath("/api/v1/topology/hot_update"));
  EXPECT_FALSE(IsAuthExemptPath("/api/v1/topology/preview"));
  // 子串相邻路径也绝不能顺带豁免
  EXPECT_FALSE(IsAuthExemptPath("/healthz/x"));
  EXPECT_FALSE(IsAuthExemptPath("/readyz?x=1"));
}
