#include "amr_dispatcher_core/dispatcher/audit_log.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace amr_dispatcher_core::dispatcher {
namespace {

// 与 tools 侧同名函数语义一致：控制字符一律转义，非法 JSON 字符不裸出。
std::string Escape(const std::string& input) {
  std::string out;
  out.reserve(input.size() + 8);
  for (const char raw : input) {
    const unsigned char c = static_cast<unsigned char>(raw);
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[7];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        } else {
          out.push_back(raw);
        }
        break;
    }
  }
  return out;
}

void AppendPair(std::ostringstream& os, const char* key, const std::string& value, bool comma) {
  if (comma) os << ",";
  os << "\"" << key << "\":\"" << Escape(value) << "\"";
}

}  // namespace

std::string SerializeAuditRecordJsonl(const AuditRecord& record) {
  std::ostringstream os;
  os << "{";
  AppendPair(os, "stamp", record.stamp_iso8601, false);
  AppendPair(os, "operator_id", record.operator_id, true);
  AppendPair(os, "action", record.action, true);
  AppendPair(os, "vehicle_id", record.vehicle_id, true);
  AppendPair(os, "mission_id", record.mission_id, true);
  AppendPair(os, "reason", record.reason, true);
  os << ",\"released_resources\":[";
  for (std::size_t i = 0; i < record.released_resources.size(); ++i) {
    if (i) os << ",";
    os << "\"" << Escape(record.released_resources[i]) << "\"";
  }
  os << "]}";
  return os.str();
}

bool AppendAuditRecord(const std::filesystem::path& path, const AuditRecord& record,
                       std::string* message) {
  if (message != nullptr) message->clear();
  if (path.empty()) {
    if (message != nullptr) *message = "audit log path is empty";
    return false;
  }
  std::error_code ec;
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
      if (message != nullptr) *message = "cannot create directory: " + ec.message();
      return false;
    }
  }
  const std::string line = SerializeAuditRecordJsonl(record) + "\n";
  // O_APPEND 单次 write：多进程/多线程追加不会互相覆盖（原子性由内核按写粒度保证）
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
  if (fd < 0) {
    if (message != nullptr) {
      *message = std::string("cannot open audit log: ") + std::strerror(errno);
    }
    return false;
  }
  std::size_t written = 0;
  while (written < line.size()) {
    const ssize_t n = ::write(fd, line.data() + written, line.size() - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (message != nullptr) {
      *message = std::string("write failed: ") + std::strerror(errno);
    }
    ::close(fd);
    return false;
  }
  ::close(fd);
  return true;
}

}  // namespace amr_dispatcher_core::dispatcher
