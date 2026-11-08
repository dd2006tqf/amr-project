#include "amr_dispatcher_tools/idempotency_store.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace amr_dispatcher_tools;

namespace {
using Reserve = IdempotencyStore::ReserveResult;

IdempotencyStore::Options MakeOptions(std::size_t capacity, std::int64_t ttl_sec) {
  IdempotencyStore::Options o;
  o.capacity = capacity;
  o.ttl_sec = ttl_sec;
  return o;
}
}  // namespace

TEST(IdempotencyStoreTest, ReserveThenDuplicate) {
  IdempotencyStore store;
  EXPECT_EQ(store.Reserve("k1", 100), Reserve::kInserted);
  // 还没 commit 也算占用：并发重放的同 key 不能同时打到后端
  EXPECT_EQ(store.Reserve("k1", 100), Reserve::kDuplicate);
}

TEST(IdempotencyStoreTest, CommitMakesKeyDurable) {
  IdempotencyStore store;
  ASSERT_EQ(store.Reserve("k1", 100), Reserve::kInserted);
  store.Commit("k1", 100);
  EXPECT_TRUE(store.IsCommitted("k1"));
  EXPECT_EQ(store.Reserve("k1", 101), Reserve::kDuplicate);
}

// 核心修复点：后端 503/504 后 key 必须能再次使用，否则一次失败会被永久
// 记成"处理过"，客户端重试永远得到 duplicate。
TEST(IdempotencyStoreTest, AbortAllowsRetryWithSameKey) {
  IdempotencyStore store;
  ASSERT_EQ(store.Reserve("k1", 100), Reserve::kInserted);
  store.Abort("k1");
  EXPECT_EQ(store.size(), 0u);
  EXPECT_EQ(store.Reserve("k1", 101), Reserve::kInserted);
}

TEST(IdempotencyStoreTest, AbortOnCommittedKeyAlsoAllowsRetry) {
  IdempotencyStore store;
  ASSERT_EQ(store.Reserve("k1", 100), Reserve::kInserted);
  store.Commit("k1", 100);
  store.Abort("k1");
  EXPECT_EQ(store.Reserve("k1", 101), Reserve::kInserted);
}

TEST(IdempotencyStoreTest, AbortUnknownKeyIsNoOp) {
  IdempotencyStore store;
  store.Abort("nope");
  EXPECT_EQ(store.size(), 0u);
}

TEST(IdempotencyStoreTest, EmptyAndControlKeysAreInvalid) {
  IdempotencyStore store;
  EXPECT_EQ(store.Reserve("", 100), Reserve::kInvalidKey);
  EXPECT_EQ(store.Reserve("a\tb", 100), Reserve::kInvalidKey);
  EXPECT_EQ(store.Reserve("a\nb", 100), Reserve::kInvalidKey);
  EXPECT_EQ(store.Reserve(std::string(257, 'x'), 100), Reserve::kInvalidKey);
}

TEST(IdempotencyStoreTest, FullStoreEvictsLeastRecentlyUsedCommitted) {
  IdempotencyStore store(MakeOptions(2, 3600));
  ASSERT_EQ(store.Reserve("k1", 100), Reserve::kInserted);
  store.Commit("k1", 100);
  ASSERT_EQ(store.Reserve("k2", 100), Reserve::kInserted);
  store.Commit("k2", 100);
  // 触发 k2 被再次访问 -> k1 变成最久未使用
  EXPECT_EQ(store.Reserve("k2", 100), Reserve::kDuplicate);

  ASSERT_EQ(store.Reserve("k3", 100), Reserve::kInserted);
  EXPECT_EQ(store.size(), 2u);
  EXPECT_FALSE(store.Contains("k1"));
  EXPECT_TRUE(store.Contains("k2"));
  EXPECT_TRUE(store.Contains("k3"));
}

TEST(IdempotencyStoreTest, FullStoreRefusesToEvictPendingKeys) {
  IdempotencyStore store(MakeOptions(1, 3600));
  ASSERT_EQ(store.Reserve("pending", 100), Reserve::kInserted);
  // 唯一一条还在处理中：宁可拒绝新 key，也不能把它踢掉
  EXPECT_EQ(store.Reserve("other", 100), Reserve::kFull);
  EXPECT_TRUE(store.Contains("pending"));
}

TEST(IdempotencyStoreTest, ExpiredCommittedKeyCanBeReservedAgain) {
  IdempotencyStore store(MakeOptions(16, 10));
  ASSERT_EQ(store.Reserve("k1", 100), Reserve::kInserted);
  store.Commit("k1", 100);
  EXPECT_EQ(store.Reserve("k1", 105), Reserve::kDuplicate);  // 未过期
  // expires_at = 110，到点即可重新占用
  EXPECT_EQ(store.Reserve("k1", 110), Reserve::kInserted);
}

TEST(IdempotencyStoreTest, PruneExpiredDropsOnlyExpiredEntries) {
  IdempotencyStore store(MakeOptions(16, 10));
  ASSERT_EQ(store.Reserve("old", 100), Reserve::kInserted);
  store.Commit("old", 100);
  ASSERT_EQ(store.Reserve("fresh", 105), Reserve::kInserted);
  store.Commit("fresh", 105);
  store.PruneExpired(111);
  EXPECT_FALSE(store.Contains("old"));
  EXPECT_TRUE(store.Contains("fresh"));
}

TEST(IdempotencyStoreTest, SerializeOnlyWritesCommittedKeys) {
  IdempotencyStore store;
  ASSERT_EQ(store.Reserve("done", 100), Reserve::kInserted);
  store.Commit("done", 100);
  ASSERT_EQ(store.Reserve("inflight", 100), Reserve::kInserted);
  const std::string text = store.Serialize();
  EXPECT_NE(text.find("done"), std::string::npos);
  EXPECT_EQ(text.find("inflight"), std::string::npos);
  EXPECT_EQ(text.rfind("v1\n", 0), 0u);
}

TEST(IdempotencyStoreTest, SnapshotRoundTrip) {
  IdempotencyStore store(MakeOptions(16, 3600));
  ASSERT_EQ(store.Reserve("k1", 100), Reserve::kInserted);
  store.Commit("k1", 100);
  ASSERT_EQ(store.Reserve("k2", 100), Reserve::kInserted);
  store.Commit("k2", 100);
  const std::string text = store.Serialize();

  IdempotencyStore restored(MakeOptions(16, 3600));
  EXPECT_EQ(restored.Restore(text, 200), 2u);
  EXPECT_TRUE(restored.IsCommitted("k1"));
  EXPECT_TRUE(restored.IsCommitted("k2"));
  EXPECT_EQ(restored.Reserve("k1", 200), Reserve::kDuplicate);
}

TEST(IdempotencyStoreTest, RestoreDropsExpiredEntries) {
  IdempotencyStore store(MakeOptions(16, 10));
  ASSERT_EQ(store.Reserve("k1", 100), Reserve::kInserted);
  store.Commit("k1", 100);
  const std::string text = store.Serialize();

  IdempotencyStore restored(MakeOptions(16, 10));
  EXPECT_EQ(restored.Restore(text, 500), 0u);
}

TEST(IdempotencyStoreTest, RestoreToleratesCorruptLines) {
  IdempotencyStore store;
  // 无版本头、坏行、非数字过期时间、超长数字都必须跳过而不是崩溃
  const std::string text =
      "good\t9999999999\n"
      "broken-line-without-tab\n"
      "bad-expiry\tabc\n"
      "overflow\t999999999999999999999\n"
      "\n";
  EXPECT_EQ(store.Restore(text, 0), 1u);
  EXPECT_TRUE(store.Contains("good"));
}

TEST(IdempotencyStoreTest, RestoreRespectsCapacity) {
  IdempotencyStore store(MakeOptions(2, 3600));
  const std::string text = "v1\nk1\t9999999999\nk2\t9999999999\nk3\t9999999999\n";
  EXPECT_EQ(store.Restore(text, 0), 2u);
}
