// Webhook 发送端：URL 解析 + 请求组装 + 一次阻塞 POST（纯 POSIX socket）。
//
// 为什么不用 libcurl：网关其余部分全部是裸 socket，为了一个"POST 一段 JSON"
// 引入新依赖（并在服务器上承受又一次 apt 安装）不划算。这里只实现必要子集，
// 代价是**不支持 https**——启动时会明确告警并禁用，而不是静默降级成明文。
//
// 关键约束：`PostWebhook` 是**阻塞**的，只允许在专用的发送线程里调用。
// 任何 HTTP 请求处理线程都不该调它（见 rest_gateway.cpp 的 AlertSenderLoop）。
#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

namespace amr_dispatcher_tools {

struct ParsedUrl {
  std::string host;
  int port = 80;
  std::string path = "/";  // 含 query
};

// 只接受 `http://host[:port][/path][?query]`。
// 显式拒绝：空 host、非 http scheme（含 https）、端口越界、带 userinfo（`user@host`）。
// 允许 query：很多 webhook（企业微信/钉钉）把 token 放在 query 里，拒绝会让功能不可用。
inline std::optional<ParsedUrl> ParseHttpUrl(const std::string& url, std::string* error) {
  const std::string scheme = "http://";
  if (url.size() <= scheme.size() || url.compare(0, scheme.size(), scheme) != 0) {
    if (error != nullptr) {
      *error = url.rfind("https://", 0) == 0
                   ? "https is not supported (no TLS in this build); use an http relay"
                   : "url must start with http://";
    }
    return std::nullopt;
  }
  const std::string rest = url.substr(scheme.size());

  ParsedUrl parsed;
  std::string host_port;
  const auto path_pos = rest.find_first_of("/?");
  if (path_pos == std::string::npos) {
    host_port = rest;
    parsed.path = "/";
  } else {
    host_port = rest.substr(0, path_pos);
    parsed.path = rest.substr(path_pos);
    if (parsed.path.front() == '?') {
      parsed.path = "/" + parsed.path;  // 裸 query 也要有 path 部分
    }
  }

  if (host_port.empty()) {
    if (error != nullptr) *error = "url has no host";
    return std::nullopt;
  }
  if (host_port.find('@') != std::string::npos) {
    if (error != nullptr) *error = "url must not contain userinfo (user@host)";
    return std::nullopt;
  }

  const auto colon = host_port.rfind(':');
  if (colon != std::string::npos) {
    parsed.host = host_port.substr(0, colon);
    const std::string port_text = host_port.substr(colon + 1);
    if (port_text.empty()) {
      if (error != nullptr) *error = "url has empty port";
      return std::nullopt;
    }
    int port = 0;
    for (const char c : port_text) {
      if (c < '0' || c > '9') {
        if (error != nullptr) *error = "url port is not numeric";
        return std::nullopt;
      }
      port = port * 10 + (c - '0');
      if (port > 65535) {
        if (error != nullptr) *error = "url port out of range";
        return std::nullopt;
      }
    }
    if (port <= 0) {
      if (error != nullptr) *error = "url port must be 1..65535";
      return std::nullopt;
    }
    parsed.port = port;
  } else {
    parsed.host = host_port;
  }

  if (parsed.host.empty()) {
    if (error != nullptr) *error = "url has no host";
    return std::nullopt;
  }
  if (error != nullptr) error->clear();
  return parsed;
}

// HTTP/1.1 POST 报文（纯字符串，便于单测断言）。
inline std::string BuildWebhookPost(const ParsedUrl& url, const std::string& body) {
  std::string out = "POST " + url.path + " HTTP/1.1\r\n";
  out += "Host: " + url.host;
  if (url.port != 80) {
    out += ":" + std::to_string(url.port);
  }
  out += "\r\n";
  out += "Content-Type: application/json\r\n";
  out += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  // 一连接一请求，显式要求对端关闭；避免某些服务端在 keep-alive 下不 flush。
  out += "Connection: close\r\n";
  out += "User-Agent: amr-dispatcher-alert/1\r\n";
  out += "\r\n";
  out += body;
  return out;
}

enum class WebhookResult {
  kOk,
  kInvalidUrl,
  kResolveFailed,
  kConnectFailed,
  kSendFailed,
  kTimeout,
  kBadStatus,
  kResponseTooLarge,
};

inline const char* WebhookResultName(WebhookResult result) {
  switch (result) {
    case WebhookResult::kOk:
      return "ok";
    case WebhookResult::kInvalidUrl:
      return "invalid_url";
    case WebhookResult::kResolveFailed:
      return "resolve_failed";
    case WebhookResult::kConnectFailed:
      return "connect_failed";
    case WebhookResult::kSendFailed:
      return "send_failed";
    case WebhookResult::kTimeout:
      return "timeout";
    case WebhookResult::kBadStatus:
      return "bad_status";
    case WebhookResult::kResponseTooLarge:
      return "response_too_large";
  }
  return "unknown";
}

struct WebhookOutcome {
  WebhookResult result = WebhookResult::kInvalidUrl;
  int status_code = 0;
  std::string detail;
};

namespace detail {

inline bool SendAll(int fd, const std::string& data) {
  std::size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    return false;
  }
  return true;
}

// 只读状态行的三位码；响应体一律丢弃（不解析、不落盘、不回显）。
inline int ParseStatusCode(const std::string& head) {
  const auto sp = head.find(' ');
  if (sp == std::string::npos || head.size() < sp + 4) {
    return 0;
  }
  if (!std::isdigit(static_cast<unsigned char>(head[sp + 1])) ||
      !std::isdigit(static_cast<unsigned char>(head[sp + 2])) ||
      !std::isdigit(static_cast<unsigned char>(head[sp + 3]))) {
    return 0;
  }
  return (head[sp + 1] - '0') * 100 + (head[sp + 2] - '0') * 10 + (head[sp + 3] - '0');
}

}  // namespace detail

// 同步 POST 一次。只允许在专用发送线程调用（会阻塞到超时）。
inline WebhookOutcome PostWebhook(const ParsedUrl& url, const std::string& body,
                                  int timeout_ms) {
  WebhookOutcome outcome;

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* addresses = nullptr;
  const std::string port_text = std::to_string(url.port);
  const int gai = ::getaddrinfo(url.host.c_str(), port_text.c_str(), &hints, &addresses);
  if (gai != 0 || addresses == nullptr) {
    outcome.result = WebhookResult::kResolveFailed;
    outcome.detail = ::gai_strerror(gai);
    return outcome;
  }

  int fd = -1;
  for (addrinfo* candidate = addresses; candidate != nullptr; candidate = candidate->ai_next) {
    fd = ::socket(candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol);
    if (fd < 0) {
      continue;
    }
    // 连接本身也要有超时：不可达的 webhook 地址不能把发送线程吊死。
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if (::connect(fd, candidate->ai_addr, candidate->ai_addrlen) == 0) {
      break;
    }
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(addresses);

  if (fd < 0) {
    outcome.result = WebhookResult::kConnectFailed;
    outcome.detail = std::strerror(errno);
    return outcome;
  }

  const std::string request = BuildWebhookPost(url, body);
  if (!detail::SendAll(fd, request)) {
    outcome.result = WebhookResult::kSendFailed;
    outcome.detail = std::strerror(errno);
    ::close(fd);
    return outcome;
  }

  // 只读到头部结束（或上限），不解析 body。
  std::string head;
  head.reserve(512);
  char buffer[1024];
  constexpr std::size_t kMaxResponseBytes = 4096;
  while (head.find("\r\n\r\n") == std::string::npos) {
    const ssize_t n = ::recv(fd, buffer, sizeof(buffer), 0);
    if (n == 0) {
      break;
    }
    if (n < 0) {
      if (errno == EINTR) continue;
      outcome.result = (errno == EAGAIN || errno == EWOULDBLOCK) ? WebhookResult::kTimeout
                                                                 : WebhookResult::kBadStatus;
      outcome.detail = std::strerror(errno);
      ::close(fd);
      return outcome;
    }
    head.append(buffer, static_cast<std::size_t>(n));
    if (head.size() > kMaxResponseBytes) {
      outcome.result = WebhookResult::kResponseTooLarge;
      ::close(fd);
      return outcome;
    }
  }
  ::close(fd);

  outcome.status_code = detail::ParseStatusCode(head);
  if (outcome.status_code >= 200 && outcome.status_code < 300) {
    outcome.result = WebhookResult::kOk;
  } else {
    outcome.result = WebhookResult::kBadStatus;
    outcome.detail = head.empty() ? "empty response" : "non-2xx status";
  }
  return outcome;
}

}  // namespace amr_dispatcher_tools
