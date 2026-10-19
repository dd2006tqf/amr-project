#include "amr_dispatcher_core/logging/async_logger.hpp"

#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

namespace amr_dispatcher_core::logging {

std::string LevelToString(const LogLevel level) {
  switch (level) {
    case LogLevel::kDebug: return "DEBUG";
    case LogLevel::kInfo: return "INFO";
    case LogLevel::kWarn: return "WARN";
    case LogLevel::kError: return "ERROR";
  }
  return "INFO";
}

std::string FormatLogRecord(const LogRecord& record) {
  std::time_t t = std::chrono::system_clock::to_time_t(record.stamp);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  std::ostringstream out;
  out << buf << " " << LevelToString(record.level) << " "
      << record.component << ": " << record.message;
  return out.str();
}

// =================== SyncLogger ===================

SyncLogger::SyncLogger(std::string path, LogLevel min_level)
    : path_(std::move(path)), min_level_(min_level),
      stream_(path_, std::ios::app) {}

SyncLogger::~SyncLogger() {
  if (stream_.is_open()) {
    stream_.flush();
  }
}

void SyncLogger::Log(const LogLevel level, const std::string& component,
                     const std::string& message) {
  if (static_cast<int>(level) < static_cast<int>(min_level_)) {
    return;
  }
  LogRecord r;
  r.level = level;
  r.component = component;
  r.message = message;
  if (stream_.is_open()) {
    stream_ << FormatLogRecord(r) << "\n";
  }
}

void SyncLogger::Flush() {
  if (stream_.is_open()) {
    stream_.flush();
  }
}

// =================== AsyncLogger ===================

AsyncLogger::AsyncLogger(Config config) : config_(std::move(config)) {
  stream_.open(config_.path, std::ios::app);
  writer_ = std::thread([this] { WriterLoop(); });
}

AsyncLogger::~AsyncLogger() { Stop(); }

void AsyncLogger::Log(const LogLevel level, const std::string& component,
                      const std::string& message) {
  if (static_cast<int>(level) < static_cast<int>(config_.min_level)) {
    return;
  }
  LogRecord r;
  r.level = level;
  r.component = component;
  r.message = message;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= config_.queue_capacity) {
      // 队列满：丢弃（不阻塞调用方），原子计数供监控告警
      dropped_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    queue_.push(std::move(r));
  }
  queue_cv_.notify_one();
}

void AsyncLogger::Flush() {
  // 等待队列清空，但有上限防止队列长持续写入卡死。
  // 写线程 pop+write 全程持锁，故空队列 ⇒ 没有"已出队未写盘"的记录；
  // 返回前在锁内 flush stream_，保证返回时数据已在文件上。
  std::unique_lock<std::mutex> lock(queue_mutex_);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  queue_cv_.wait_until(lock, deadline,
                       [this] { return queue_.empty() || stop_; });
  if (stream_.is_open()) {
    stream_.flush();
  }
}

void AsyncLogger::Stop() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  queue_cv_.notify_all();
  if (writer_.joinable()) {
    writer_.join();
  }
  if (stream_.is_open()) {
    stream_.flush();
  }
}

void AsyncLogger::WriterLoop() {
  while (true) {
    bool should_exit = false;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(lock, config_.flush_interval,
                         [this] { return !queue_.empty() || stop_; });
      // 出队与写盘同持 queue_mutex_：消掉"已 pop 但未写 stream_"的竞态窗口，
      // 使 Flush 只依赖 queue_.empty() 即可断言已全部写入 stream_。
      while (!queue_.empty()) {
        const LogRecord record = std::move(queue_.front());
        queue_.pop();
        if (stream_.is_open()) {
          stream_ << FormatLogRecord(record) << "\n";
        }
      }
      should_exit = stop_;
    }
    // 队列已排空：唤醒等待落盘的 Flush，避免其空等到超时
    queue_cv_.notify_all();
    if (should_exit) {
      return;
    }
  }
}

}  // namespace amr_dispatcher_core::logging
