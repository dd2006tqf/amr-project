// REST 幂等键存储（纯 C++，无 IO、无时钟读取——时间由调用方注入）。
//
// 旧实现是一个裸 unordered_set：请求**还没发给后端服务**就先插入 key。
// 后端 503 / 504 时 key 已经被吃掉，客户端拿同一个 idempotency_key 重试
// 永远得到 "idempotent_duplicate"，等于把一次失败永久变成"成功过"。
//
// 现在的语义是两阶段：
//   Reserve(key) -> kInserted   （占用，标记为 pending）
//   Commit(key)                  （后端确实受理了，才算真正消费掉）
//   Abort(key)                   （后端失败/超时，撤掉占用，允许客户端重试）
// 重复请求（pending 或 committed）都返回 kDuplicate。
#pragma once

#include <cstddef>
#include <cstdint>
#include <list>
#include <string>
#include <unordered_map>
#include <vector>

namespace amr_dispatcher_tools {

class IdempotencyStore {
 public:
  enum class ReserveResult {
    kInserted,   // 新 key，已占用（pending）
    kDuplicate,  // 已存在（pending 或 committed）
    kFull,       // 容量已满且无可淘汰项（全部 pending）
    kInvalidKey, // 空 key 或含控制字符
  };

  struct Options {
    std::size_t capacity = 4096;
    std::int64_t ttl_sec = 3600;
  };

  IdempotencyStore() = default;
  explicit IdempotencyStore(Options options) : options_(options) {}

  // 键必须是单行、非空：落盘格式是 TSV，含 \t / \r / \n 会破坏格式。
  static bool IsValidKey(const std::string& key) {
    if (key.empty() || key.size() > 256) {
      return false;
    }
    for (const char c : key) {
      if (c == '\t' || c == '\r' || c == '\n' || static_cast<unsigned char>(c) < 0x20) {
        return false;
      }
    }
    return true;
  }

  // 占用一个 key。now_epoch_sec 由调用方注入（便于测试，不读系统时钟）。
  ReserveResult Reserve(const std::string& key, std::int64_t now_epoch_sec) {
    if (!IsValidKey(key)) {
      return ReserveResult::kInvalidKey;
    }
    // 先做一次惰性过期清理：容量压力与重复判定都以此为准
    DropEntryIfExpired(key, now_epoch_sec);
    auto it = entries_.find(key);
    if (it != entries_.end()) {
      // 重放也算一次访问：否则并发重放会把仍在用的键排到最前被淘汰
      Touch(key, it->second);
      return ReserveResult::kDuplicate;
    }
    if (entries_.size() >= options_.capacity) {
      PruneExpired(now_epoch_sec);
    }
    while (entries_.size() >= options_.capacity) {
      if (!EvictLeastRecentlyUsedCommitted()) {
        // 全部是 pending：宁可选拒绝，也不驱逐别人正在处理的键
        return ReserveResult::kFull;
      }
    }
    Entry entry;
    entry.committed = false;
    entry.expires_at = now_epoch_sec + options_.ttl_sec;
    entry.lru_it = lru_.insert(lru_.end(), key);
    entries_.emplace(key, std::move(entry));
    return ReserveResult::kInserted;
  }

  // 后端已受理：标记 committed 并刷新 TTL。key 不存在（例如已被 Abort）时无操作。
  void Commit(const std::string& key, std::int64_t now_epoch_sec) {
    auto it = entries_.find(key);
    if (it == entries_.end()) {
      return;
    }
    it->second.committed = true;
    it->second.expires_at = now_epoch_sec + options_.ttl_sec;
    Touch(key, it->second);
  }

  // 后端失败/超时：撤销占用，客户端可以用同一个 key 重试。
  void Abort(const std::string& key) {
    auto it = entries_.find(key);
    if (it == entries_.end()) {
      return;
    }
    lru_.erase(it->second.lru_it);
    entries_.erase(it);
  }

  // 清理已过期条目（pending 与 committed 一视同仁：卡死的工作线程不能永久占位）。
  void PruneExpired(std::int64_t now_epoch_sec) {
    for (auto it = entries_.begin(); it != entries_.end();) {
      if (it->second.expires_at <= now_epoch_sec) {
        lru_.erase(it->second.lru_it);
        it = entries_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::size_t size() const { return entries_.size(); }

  bool Contains(const std::string& key) const { return entries_.find(key) != entries_.end(); }

  // 判断某 key 是否已被确认消费（committed）。pending 返回 false。
  bool IsCommitted(const std::string& key) const {
    auto it = entries_.find(key);
    return it != entries_.end() && it->second.committed;
  }

  // 落盘（TSV，单行一条，按 LRU 顺序）。只写 committed 的键——pending 是
  // 进程内瞬态，重启后本就不该再挡住客户端重试。
  std::string Serialize() const {
    std::string out = "v1\n";
    for (const auto& key : lru_) {
      auto it = entries_.find(key);
      if (it == entries_.end() || !it->second.committed) {
        continue;
      }
      out += key;
      out += '\t';
      out += std::to_string(it->second.expires_at);
      out += '\n';
    }
    return out;
  }

  // 载入落盘内容；已过期条目丢弃。返回成功载入的条数。
  std::size_t Restore(const std::string& text, std::int64_t now_epoch_sec) {
    entries_.clear();
    lru_.clear();
    std::size_t loaded = 0;
    std::size_t pos = 0;
    bool first = true;
    while (pos < text.size()) {
      std::size_t eol = text.find('\n', pos);
      if (eol == std::string::npos) {
        eol = text.size();
      }
      const std::string line = text.substr(pos, eol - pos);
      pos = eol + 1;
      if (first) {
        first = false;
        if (line == "v1") {
          continue;  // 版本头
        }
      }
      if (line.empty()) {
        continue;
      }
      const std::size_t tab = line.find('\t');
      if (tab == std::string::npos) {
        continue;  // 破损行：跳过而不是整体拒载
      }
      const std::string key = line.substr(0, tab);
      if (!IsValidKey(key)) {
        continue;
      }
      const std::string expiry_str = line.substr(tab + 1);
      // 位数上限：超过 18 位必然溢出 int64，std::stoll 会抛 out_of_range
      if (expiry_str.empty() || expiry_str.size() > 18) {
        continue;
      }
      for (const char c : expiry_str) {
        if (c < '0' || c > '9') {
          return loaded;  // 破损行：停止载入，保留已载入部分
        }
      }
      const std::int64_t expires_at = std::stoll(expiry_str);
      if (expires_at <= now_epoch_sec) {
        continue;  // 已过期
      }
      if (entries_.size() >= options_.capacity) {
        break;
      }
      Entry entry;
      entry.committed = true;
      entry.expires_at = expires_at;
      entry.lru_it = lru_.insert(lru_.end(), key);
      entries_.emplace(key, std::move(entry));
      ++loaded;
    }
    return loaded;
  }

  const Options& options() const { return options_; }

 private:
  struct Entry {
    bool committed = false;
    std::int64_t expires_at = 0;
    std::list<std::string>::iterator lru_it;
  };

  void Touch(const std::string& key, Entry& entry) {
    lru_.erase(entry.lru_it);
    entry.lru_it = lru_.insert(lru_.end(), key);
  }

  void DropEntryIfExpired(const std::string& key, std::int64_t now_epoch_sec) {
    auto it = entries_.find(key);
    if (it == entries_.end() || it->second.expires_at > now_epoch_sec) {
      return;
    }
    lru_.erase(it->second.lru_it);
    entries_.erase(it);
  }

  // 淘汰最久未使用的 committed 条目；只剩 pending 时返回 false。
  bool EvictLeastRecentlyUsedCommitted() {
    for (auto lru_it = lru_.begin(); lru_it != lru_.end(); ++lru_it) {
      auto it = entries_.find(*lru_it);
      if (it == entries_.end() || !it->second.committed) {
        continue;
      }
      entries_.erase(it);
      lru_.erase(lru_it);
      return true;
    }
    return false;
  }

  Options options_{};
  std::unordered_map<std::string, Entry> entries_;
  std::list<std::string> lru_;
};

}  // namespace amr_dispatcher_tools
