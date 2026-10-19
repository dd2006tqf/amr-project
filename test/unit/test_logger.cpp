#include "amr_dispatcher_core/logging/async_logger.hpp"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <regex>
#include <thread>
#include <vector>

using namespace amr_dispatcher_core::logging;

namespace {

std::string ReadAll(const std::string& path) {
  std::ifstream in(path);
  std::stringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

std::size_t CountLines(const std::string& path) {
  std::ifstream in(path);
  std::size_t lines = 0;
  std::string line;
  while (std::getline(in, line)) {
    ++lines;
  }
  return lines;
}

}  // namespace

TEST(SyncLoggerTest, WritesFormattedRecords) {
  std::string path = std::tmpnam(nullptr);
  {
    SyncLogger logger(path);
    logger.Log(LogLevel::kInfo, "core", "hello");
    logger.Log(LogLevel::kError, "core", "boom");
    logger.Flush();
  }
  const auto content = ReadAll(path);
  EXPECT_NE(content.find("INFO core: hello"), std::string::npos);
  EXPECT_NE(content.find("ERROR core: boom"), std::string::npos);
  std::remove(path.c_str());
}

TEST(SyncLoggerTest, BelowMinLevelDropped) {
  std::string path = std::tmpnam(nullptr);
  SyncLogger logger(path, LogLevel::kWarn);
  logger.Log(LogLevel::kInfo, "x", "should_drop");
  logger.Log(LogLevel::kWarn, "x", "kept");
  logger.Flush();
  const auto content = ReadAll(path);
  EXPECT_EQ(content.find("should_drop"), std::string::npos);
  EXPECT_NE(content.find("kept"), std::string::npos);
  std::remove(path.c_str());
}

TEST(AsyncLoggerTest, FlushesOnDestroy) {
  std::string path = std::tmpnam(nullptr);
  {
    AsyncLogger logger({.path = path, .queue_capacity = 1024,
                        .flush_interval = std::chrono::milliseconds(20)});
    for (int i = 0; i < 100; ++i) {
      logger.Log(LogLevel::kInfo, "async", "msg" + std::to_string(i));
    }
  }  // 析构触发 flush + join
  const auto content = ReadAll(path);
  EXPECT_NE(content.find("msg0"), std::string::npos);
  EXPECT_NE(content.find("msg99"), std::string::npos);
  std::remove(path.c_str());
}

TEST(AsyncLoggerTest, BelowMinLevelDropped) {
  std::string path = std::tmpnam(nullptr);
  AsyncLogger logger({.path = path, .min_level = LogLevel::kWarn,
                      .queue_capacity = 64, .flush_interval = std::chrono::milliseconds(20)});
  logger.Log(LogLevel::kInfo, "x", "drop");
  logger.Log(LogLevel::kError, "x", "kept");
  logger.Flush();
  const auto content = ReadAll(path);
  EXPECT_EQ(content.find("drop"), std::string::npos);
  EXPECT_NE(content.find("kept"), std::string::npos);
  logger.Stop();
  std::remove(path.c_str());
}

TEST(LoggerFormatTest, LevelToStringAllCases) {
  EXPECT_EQ(LevelToString(LogLevel::kDebug), "DEBUG");
  EXPECT_EQ(LevelToString(LogLevel::kInfo), "INFO");
  EXPECT_EQ(LevelToString(LogLevel::kWarn), "WARN");
  EXPECT_EQ(LevelToString(LogLevel::kError), "ERROR");
}

TEST(LoggerFormatTest, FormatLogRecordStructure) {
  LogRecord r;
  r.level = LogLevel::kWarn;
  r.component = "x";
  r.message = "m";
  const auto line = FormatLogRecord(r);
  std::regex re(R"(^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z WARN x: m$)");
  EXPECT_TRUE(std::regex_match(line, re));
}

// A4(a)：4 生产者 × 10k 条并发 Log + 周期 Flush，无竞争语义下零丢弃、条数精确
TEST(AsyncLoggerTest, ConcurrentProducersLoseNothingWhenCapacitySufficient) {
  std::string path = std::tmpnam(nullptr);
  constexpr int kProducers = 4;
  constexpr int kPerProducer = 10000;
  {
    // 容量 >= 总条数：永不触发溢出，dropped_count 必须为 0
    AsyncLogger logger({.path = path, .queue_capacity = 65536,
                        .flush_interval = std::chrono::milliseconds(10)});
    std::vector<std::thread> producers;
    producers.reserve(kProducers);
    for (int t = 0; t < kProducers; ++t) {
      producers.emplace_back([&logger, t] {
        for (int i = 0; i < kPerProducer; ++i) {
          logger.Log(LogLevel::kInfo, "stress",
                     "t" + std::to_string(t) + "_m" + std::to_string(i));
        }
      });
    }
    // 主线程周期 Flush，与写线程/生产者并发
    for (int round = 0; round < 10; ++round) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      logger.Flush();
    }
    for (auto& p : producers) {
      p.join();
    }
    EXPECT_EQ(logger.dropped_count(), 0u);
  }  // 析构 flush + join
  // 全部落盘：每条记录恰好一行
  EXPECT_EQ(CountLines(path), static_cast<std::size_t>(kProducers * kPerProducer));
  const auto content = ReadAll(path);
  EXPECT_NE(content.find("t0_m0"), std::string::npos);
  EXPECT_NE(content.find("t3_m9999"), std::string::npos);
  std::remove(path.c_str());
}

// A4(b)：队列溢出丢弃必须精确计数（capacity=0：每条都丢，无"静默丢失"）
TEST(AsyncLoggerTest, DroppedCountIsExactUnderOverflow) {
  std::string path = std::tmpnam(nullptr);
  constexpr int kProducers = 4;
  constexpr int kPerProducer = 10000;
  {
    AsyncLogger logger({.path = path, .queue_capacity = 0,
                        .flush_interval = std::chrono::milliseconds(10)});
    std::vector<std::thread> producers;
    producers.reserve(kProducers);
    for (int t = 0; t < kProducers; ++t) {
      producers.emplace_back([&logger, t] {
        for (int i = 0; i < kPerProducer; ++i) {
          logger.Log(LogLevel::kInfo, "stress",
                     "t" + std::to_string(t) + "_m" + std::to_string(i));
        }
      });
    }
    for (auto& p : producers) {
      p.join();
    }
    EXPECT_EQ(logger.dropped_count(), static_cast<std::uint64_t>(kProducers * kPerProducer));
  }
  EXPECT_EQ(CountLines(path), 0u);  // 全丢：文件为空
  std::remove(path.c_str());
}

// A4(c)：Flush() 返回后，调用前所有已 Log 记录必须已写入文件（含已出队未写盘的）
TEST(AsyncLoggerTest, FlushGuaranteesRecordsOnDisk) {
  std::string path = std::tmpnam(nullptr);
  constexpr int kRecords = 1000;
  {
    AsyncLogger logger({.path = path, .queue_capacity = 65536,
                        .flush_interval = std::chrono::milliseconds(10)});
    for (int i = 0; i < kRecords; ++i) {
      logger.Log(LogLevel::kInfo, "flush", "pre_flush_" + std::to_string(i));
    }
    logger.Flush();
    // 不 Stop、不析构：Flush 返回即须可见
    EXPECT_EQ(CountLines(path), static_cast<std::size_t>(kRecords));
    const auto content = ReadAll(path);
    EXPECT_NE(content.find("pre_flush_0"), std::string::npos);
    EXPECT_NE(content.find("pre_flush_999"), std::string::npos);
  }
  std::remove(path.c_str());
}
