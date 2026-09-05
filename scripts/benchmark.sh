#!/usr/bin/env bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR="$DIR/.."

echo "=========================================================="
echo "         AMR Dispatcher Performance Benchmark             "
echo "=========================================================="

BUILD_DIR="$ROOT_DIR/build/benchmark"
mkdir -p "$BUILD_DIR"

cat << 'EOF' > "$BUILD_DIR/bench_core.cpp"
#include <chrono>
#include <iostream>
#include <vector>
#include "amr_dispatcher_core/dispatcher/mission_queue.hpp"
#include "amr_dispatcher_core/logging/async_logger.hpp"

using namespace std::chrono_literals;

void BenchmarkMissionQueue(std::size_t n_items) {
  std::cout << "[Benchmark] Testing MissionQueue (" << n_items << " missions Push & Pop)..." << std::endl;
  amr_dispatcher_core::dispatcher::MissionQueue::Config cfg;
  cfg.max_size = n_items + 10;
  cfg.comparator = "priority_fifo";
  amr_dispatcher_core::dispatcher::MissionQueue queue(cfg);

  auto t0 = std::chrono::high_resolution_clock::now();
  for (std::size_t i = 0; i < n_items; ++i) {
    amr_dispatcher_core::dispatcher::Mission m;
    m.id = "m_" + std::to_string(i);
    m.priority = static_cast<int>(i % 100);
    m.sequence = i;
    queue.Push(m);
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  double push_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  auto t2 = std::chrono::high_resolution_clock::now();
  for (std::size_t i = 0; i < n_items; ++i) {
    queue.PopNext();
  }
  auto t3 = std::chrono::high_resolution_clock::now();
  double pop_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

  std::cout << "  - Push throughput: " << (n_items / (push_ms / 1000.0)) << " ops/sec (total: " << push_ms << " ms)" << std::endl;
  std::cout << "  - Pop throughput:  " << (n_items / (pop_ms / 1000.0)) << " ops/sec (total: " << pop_ms << " ms)" << std::endl;
}

void BenchmarkAsyncLogger(std::size_t n_logs) {
  std::cout << "[Benchmark] Testing AsyncLogger (" << n_logs << " records enqueue latency)..." << std::endl;
  std::string tmp_log = "/tmp/bench_async.log";
  {
    amr_dispatcher_core::logging::AsyncLogger logger(
        {.path = tmp_log, .min_level = amr_dispatcher_core::logging::LogLevel::kInfo,
         .queue_capacity = n_logs + 100, .flush_interval = 50ms});

    auto t0 = std::chrono::high_resolution_clock::now();
    for (std::size_t i = 0; i < n_logs; ++i) {
      logger.Log(amr_dispatcher_core::logging::LogLevel::kInfo, "bench", "high_freq_log_message");
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double log_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "  - Enqueue throughput: " << (n_logs / (log_ms / 1000.0)) << " ops/sec (total: " << log_ms << " ms)" << std::endl;
  }
  std::remove(tmp_log.c_str());
}

int main() {
  BenchmarkMissionQueue(10000);
  BenchmarkAsyncLogger(50000);
  std::cout << "[Benchmark] All benchmarks completed successfully!" << std::endl;
  return 0;
}
EOF

c++ -O3 -std=c++20 "$BUILD_DIR/bench_core.cpp" -o "$BUILD_DIR/bench_core" \
    -I"$ROOT_DIR/src/amr_dispatcher_core/include" \
    -L"$ROOT_DIR/build/core" -lamr_dispatcher_core -lyaml-cpp -lpthread

"$BUILD_DIR/bench_core"
rm -rf "$BUILD_DIR"
