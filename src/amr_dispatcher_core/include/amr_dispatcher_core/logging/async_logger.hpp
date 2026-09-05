#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <fstream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace amr_dispatcher_core::logging {

enum class LogLevel { kDebug, kInfo, kWarn, kError };

struct LogRecord {
  std::chrono::system_clock::time_point stamp = std::chrono::system_clock::now();
  LogLevel level = LogLevel::kInfo;
  std::string component;
  std::string message;
};

// 同步日志器（baseline）：每条 Log() 直接写文件。无队列、无线程。
class SyncLogger {
 public:
  explicit SyncLogger(std::string path, LogLevel min_level = LogLevel::kInfo);
  ~SyncLogger();

  SyncLogger(const SyncLogger&) = delete;
  SyncLogger& operator=(const SyncLogger&) = delete;

  void Log(LogLevel level, const std::string& component, const std::string& message);
  void Flush();

 private:
  std::string path_;
  LogLevel min_level_;
  std::ofstream stream_;
};

// 异步日志器：单写线程 + 队列。Log() 仅入队；析构/Stop() 时 flush 并 join。
class AsyncLogger {
 public:
  struct Config {
    std::string path;
    LogLevel min_level = LogLevel::kInfo;
    std::size_t queue_capacity = 4096;
    std::chrono::milliseconds flush_interval{std::chrono::milliseconds(100)};
  };

  explicit AsyncLogger(Config config);
  ~AsyncLogger();

  AsyncLogger(const AsyncLogger&) = delete;
  AsyncLogger& operator=(const AsyncLogger&) = delete;

  void Log(LogLevel level, const std::string& component, const std::string& message);
  void Flush();

  void Stop();

 private:
  void WriterLoop();

  Config config_;
  std::ofstream stream_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<LogRecord> queue_;
  bool stop_ = false;
  std::thread writer_;
};

// 格式化辅助
std::string FormatLogRecord(const LogRecord& record);
std::string LevelToString(LogLevel level);

}  // namespace amr_dispatcher_core::logging
