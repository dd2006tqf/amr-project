// 叠加层文件读写的验收。
//
// 这些用例守的是"进程随时可能死"这条现实：
//   * 写入必须先落 tmp、回读校验、再 rename —— 半截文件绝不能被当成有效叠加层；
//   * 旧文件要留 .bak，现场才能追溯"昨天封了哪条路"；
//   * 基础地图变更导致的失效是**改名归档**，不是删除用户数据。
#include "amr_dispatcher_core/catalog/topology_overlay.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace amr_dispatcher_core::catalog {
namespace {

namespace fs = std::filesystem;

class OverlayFileTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("amr_overlay_test_" + std::to_string(::getpid()) + "_" +
                                        std::to_string(counter_++));
    std::error_code ec;
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_);
    path_ = dir_ / "topology_overlay.yaml";
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }

  static TopologyOverlay MakeOverlay(std::uint64_t revision) {
    TopologyOverlay overlay;
    overlay.base_map_hash = "fnv1a64_deadbeef";
    overlay.revision = revision;
    overlay.updated_at = "2026-10-29T08:00:00Z";
    OverlayEdge edge;
    edge.from = "a";
    edge.to = "b";
    edge.reason = "water main";
    edge.operator_id = "op_1";
    edge.created_at = "2026-10-29T08:00:00Z";
    overlay.blocked_edges.push_back(edge);
    return overlay;
  }

  fs::path dir_;
  fs::path path_;
  static int counter_;
};

int OverlayFileTest::counter_ = 0;

TEST_F(OverlayFileTest, PersistWritesFileThatLoadsBack) {
  const auto overlay = MakeOverlay(1);
  std::string message;
  ASSERT_TRUE(PersistTopologyOverlay(path_, overlay, &message)) << message;
  ASSERT_TRUE(fs::exists(path_));

  const auto loaded = LoadTopologyOverlay(path_, &message);
  ASSERT_TRUE(loaded.has_value()) << message;
  EXPECT_EQ(loaded->base_map_hash, overlay.base_map_hash);
  EXPECT_EQ(loaded->revision, 1u);
  ASSERT_EQ(loaded->blocked_edges.size(), 1u);
  EXPECT_EQ(loaded->blocked_edges.front().reason, "water main");
}

// 已有文件时被替换，旧内容留成 .bak —— 现场靠它回溯"昨天封了哪条路"
TEST_F(OverlayFileTest, PersistRotatesPreviousFileToBak) {
  std::string message;
  ASSERT_TRUE(PersistTopologyOverlay(path_, MakeOverlay(1), &message)) << message;
  ASSERT_TRUE(PersistTopologyOverlay(path_, MakeOverlay(2), &message)) << message;

  fs::path backup = path_;
  backup += ".bak";
  ASSERT_TRUE(fs::exists(backup));

  const auto current = LoadTopologyOverlay(path_, &message);
  const auto previous = LoadTopologyOverlay(backup, &message);
  ASSERT_TRUE(current.has_value()) << message;
  ASSERT_TRUE(previous.has_value()) << message;
  EXPECT_EQ(current->revision, 2u);
  EXPECT_EQ(previous->revision, 1u) << "backup must hold the previous revision, not the new one";
}

// 写入过程中不能留下 .tmp 残骸：否则下次启动会看到一堆垃圾文件
TEST_F(OverlayFileTest, PersistLeavesNoTempFileBehind) {
  std::string message;
  ASSERT_TRUE(PersistTopologyOverlay(path_, MakeOverlay(1), &message)) << message;
  fs::path tmp = path_;
  tmp += ".tmp";
  EXPECT_FALSE(fs::exists(tmp));
}

// 目标路径的父目录不存在时应自动创建（生产里 config/generated 可能还没生成）
TEST_F(OverlayFileTest, PersistCreatesParentDirectory) {
  const fs::path nested = dir_ / "config" / "generated" / "topology_overlay.yaml";
  std::string message;
  ASSERT_TRUE(PersistTopologyOverlay(nested, MakeOverlay(1), &message)) << message;
  EXPECT_TRUE(fs::exists(nested));
}

// 写入失败必须不破坏原文件 —— 这是"原子"的最低要求。
// 用"父路径是普通文件"制造 create_directories 失败。
TEST_F(OverlayFileTest, PersistFailureLeavesOriginalUntouched) {
  std::string message;
  ASSERT_TRUE(PersistTopologyOverlay(path_, MakeOverlay(1), &message)) << message;
  const std::string before = [this] {
    std::ifstream in(path_);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  }();

  // 父目录换成一个普通文件：再往它下面建目录必然失败
  const fs::path blocker = dir_ / "blocker";
  {
    std::ofstream out(blocker);
    out << "not a directory\n";
  }
  const fs::path impossible = blocker / "topology_overlay.yaml";
  EXPECT_FALSE(PersistTopologyOverlay(impossible, MakeOverlay(2), &message));
  EXPECT_FALSE(message.empty());

  // 原文件逐字节未变
  const std::string after = [this] {
    std::ifstream in(path_);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  }();
  EXPECT_EQ(before, after);
}

TEST_F(OverlayFileTest, LoadReportsMissingFile) {
  std::string message;
  EXPECT_FALSE(LoadTopologyOverlay(dir_ / "nope.yaml", &message).has_value());
  EXPECT_TRUE(message.find("cannot open") != std::string::npos) << message;
}

// 手改坏了文件时，加载必须失败（而不是当成空叠加层静默通过）——
// 静默通过会让"我以为封了路、其实没封"成为现实事故。
TEST_F(OverlayFileTest, LoadRejectsCorruptFile) {
  {
    std::ofstream out(path_);
    out << "base_map_hash: \"h\"\nblocked_edges:\n  - from: \"a\"\n    garbage_line_here\n";
  }
  std::string message;
  EXPECT_FALSE(LoadTopologyOverlay(path_, &message).has_value());
  EXPECT_FALSE(message.empty());
}

// 基础地图重新出图 -> 叠加层失效：改名归档，不删除（可能要追溯）
TEST_F(OverlayFileTest, InvalidateRelocatesWithSuffix) {
  std::string message;
  ASSERT_TRUE(PersistTopologyOverlay(path_, MakeOverlay(3), &message)) << message;

  std::string relocated;
  ASSERT_TRUE(InvalidateOverlayFile(path_, "stale", &relocated, &message)) << message;

  EXPECT_FALSE(fs::exists(path_)) << "the live overlay path must be gone after invalidation";
  ASSERT_TRUE(fs::exists(relocated));
  EXPECT_NE(relocated.find(".stale."), std::string::npos) << relocated;

  // 归档件仍是可读的完整叠加层（现场可以据此复现昨天的施工区）
  const auto archived = LoadTopologyOverlay(relocated, &message);
  ASSERT_TRUE(archived.has_value()) << message;
  EXPECT_EQ(archived->revision, 3u);
}

TEST_F(OverlayFileTest, InvalidateOnMissingFileReportsError) {
  std::string relocated;
  std::string message;
  EXPECT_FALSE(InvalidateOverlayFile(dir_ / "absent.yaml", "stale", &relocated, &message));
  EXPECT_TRUE(relocated.empty());
  EXPECT_FALSE(message.empty());
}

// 往返：写 -> 读 -> 再写，两次字节必须一致（否则"拒绝时文件未改写"的断言无法成立）
TEST_F(OverlayFileTest, PersistRoundTripsByteIdentically) {
  std::string message;
  ASSERT_TRUE(PersistTopologyOverlay(path_, MakeOverlay(1), &message)) << message;
  const auto loaded = LoadTopologyOverlay(path_, &message);
  ASSERT_TRUE(loaded.has_value()) << message;

  const fs::path second = dir_ / "second.yaml";
  ASSERT_TRUE(PersistTopologyOverlay(second, *loaded, &message)) << message;

  const auto read_all = [](const fs::path& p) {
    std::ifstream in(p);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  };
  EXPECT_EQ(read_all(path_), read_all(second));
}

// 空叠加层（清空全部临时改道）也要能正确往返
TEST_F(OverlayFileTest, PersistsEmptyOverlaySoItCanBeCleared) {
  TopologyOverlay empty;
  empty.base_map_hash = "fnv1a64_deadbeef";
  empty.revision = 5;
  empty.updated_at = "2026-10-29T09:00:00Z";

  std::string message;
  ASSERT_TRUE(PersistTopologyOverlay(path_, empty, &message)) << message;
  const auto loaded = LoadTopologyOverlay(path_, &message);
  ASSERT_TRUE(loaded.has_value()) << message;
  EXPECT_TRUE(loaded->Empty());
  EXPECT_EQ(loaded->revision, 5u);
}

}  // namespace
}  // namespace amr_dispatcher_core::catalog
