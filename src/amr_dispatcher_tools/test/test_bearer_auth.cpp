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
