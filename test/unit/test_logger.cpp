#include "amr_dispatcher_core/logging/async_logger.hpp"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <regex>

using namespace amr_dispatcher_core::logging;

namespace {

std::string ReadAll(const std::string& path) {
  std::ifstream in(path);
  std::stringstream buf;
  buf << in.rdbuf();
  return buf.str();
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
