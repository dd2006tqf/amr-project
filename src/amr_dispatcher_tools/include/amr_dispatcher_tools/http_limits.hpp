// HTTP 请求解析的安全边界（纯函数，无 IO、无异常、无时钟）。
//
// 背景：旧网关用 std::stoi 直接解析 Content-Length 与 prio，任何一个畸形值
// 都会抛 std::invalid_argument 并且没有被捕获 —— 整个网关进程 terminate。
// 另外路由用 path.find(...) 做子串匹配，`/api/v1/ordersXYZ` 也会命中订单路由。
// 这里把"解析"与"路由匹配"抽成可单测的纯函数，网关只负责 IO 与副作用。
#pragma once

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <string>

namespace amr_dispatcher_tools {

// 请求体上限（1 MiB）：地图上传是本网关最大的合法请求，远小于此值。
inline constexpr std::size_t kMaxRequestBytes = 1024U * 1024U;
// 请求头段上限（8 KiB）：正常请求头只有几百字节。
inline constexpr std::size_t kMaxHeaderBytes = 8U * 1024U;
// 单连接收发超时（秒）：静默连接不能永久占住工作线程。
inline constexpr int kSocketTimeoutSec = 5;
// accept 线程在"读请求头"上允许占用的预算（毫秒）。
//
// 探针快车道必须在 accept 线程里读头才能绕开工作线程池，但 accept 线程本身
// 是单点：一旦它为一个慢客户端等满 kSocketTimeoutSec，所有新连接（包括
// 探针自己）都得排队。所以这里只给 50ms —— 足够本机环回一次 read，
// 又远小于任何客户端的容忍度。超预算的连接立刻入队，由 worker 去等。
inline constexpr int kAcceptReadBudgetMs = 50;

// Content-Length 的解析结果：
//   kMissing  —— 没有该头部（GET 等无体请求，合法）
//   kOk       —— 合法且在上限内
//   kInvalid  —— 非数字 / 负数 / 溢出（调用方应回 400）
//   kTooLarge —— 合法但超过 kMaxRequestBytes（调用方应回 413）
enum class ContentLengthStatus { kMissing, kOk, kInvalid, kTooLarge };

// 去掉首尾空白（空格、制表符、CR、LF）。
inline std::string TrimWhitespace(const std::string& s) {
  std::size_t begin = 0;
  while (begin < s.size() &&
         (s[begin] == ' ' || s[begin] == '\t' || s[begin] == '\r' || s[begin] == '\n')) {
    ++begin;
  }
  std::size_t end = s.size();
  while (end > begin &&
         (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r' || s[end - 1] == '\n')) {
    --end;
  }
  return s.substr(begin, end - begin);
}

// 解析 Content-Length 的值（不含头部名）。只接受纯十进制数字。
inline ContentLengthStatus ParseContentLength(const std::string& value, std::size_t* out) {
  if (out != nullptr) {
    *out = 0;
  }
  const std::string trimmed = TrimWhitespace(value);
  if (trimmed.empty()) {
    return ContentLengthStatus::kMissing;
  }
  for (const char c : trimmed) {
    if (c < '0' || c > '9') {
      return ContentLengthStatus::kInvalid;  // 负数（'-'）、小数、字母一律非法
    }
  }
  // 用 unsigned long long 累计并逐位判溢出：超大数值必须是明确的 kInvalid，
  // 绝不能被截断成一个"看起来合法"的小长度。
  unsigned long long acc = 0;
  for (const char c : trimmed) {
    const unsigned long long digit = static_cast<unsigned long long>(c - '0');
    if (acc > (18446744073709551615ULL - digit) / 10ULL) {
      return ContentLengthStatus::kInvalid;  // 数值溢出
    }
    acc = acc * 10ULL + digit;
  }
  if (acc > static_cast<unsigned long long>(kMaxRequestBytes)) {
    if (out != nullptr) {
      *out = 0;  // 不返回被截断的巨值，调用方只按状态码处置
    }
    return ContentLengthStatus::kTooLarge;
  }
  if (out != nullptr) {
    *out = static_cast<std::size_t>(acc);
  }
  return ContentLengthStatus::kOk;
}

// 取请求 URI 的路径部分：剥离 query（`?` 之后）与 fragment（`#` 之后）。
// 兼容代理常见 absolute-form：`http://host:port/path?q` -> `/path`。
inline std::string RequestPathOnly(const std::string& target) {
  std::string path = target;
  const std::size_t scheme = path.find("://");
  if (scheme != std::string::npos) {
    const std::size_t path_begin = path.find('/', scheme + 3);
    path = (path_begin == std::string::npos) ? "/" : path.substr(path_begin);
  }
  const std::size_t query = path.find('?');
  if (query != std::string::npos) {
    path = path.substr(0, query);
  }
  const std::size_t fragment = path.find('#');
  if (fragment != std::string::npos) {
    path = path.substr(0, fragment);
  }
  return path;
}

// 取请求 URI 的 query 部分（不含 `?`；无 query 返回空串）。
inline std::string RequestQueryOnly(const std::string& target) {
  const std::size_t query = target.find('?');
  if (query == std::string::npos) {
    return "";
  }
  std::string rest = target.substr(query + 1);
  const std::size_t fragment = rest.find('#');
  if (fragment != std::string::npos) {
    rest = rest.substr(0, fragment);
  }
  return rest;
}

// 精确路径匹配：不再用子串包含，杜绝 `/api/v1/ordersXYZ` 命中订单路由。
inline bool PathMatches(const std::string& path, const std::string& expected) {
  return path == expected;
}

inline bool PathMatchesAny(const std::string& path, const std::string& a, const std::string& b) {
  return PathMatches(path, a) || PathMatches(path, b);
}

// 在头部块里大小写不敏感地查一个字段，返回去掉首尾空白后的值。
// head_block 是请求行之后、空行之前的全部内容（可含请求行，按行扫描即可）。
inline bool FindHeaderValue(const std::string& head_block, const std::string& name,
                            std::string* out) {
  if (out != nullptr) {
    out->clear();
  }
  std::size_t line_begin = 0;
  while (line_begin < head_block.size()) {
    std::size_t line_end = head_block.find("\r\n", line_begin);
    if (line_end == std::string::npos) {
      line_end = head_block.size();
    }
    const std::string line = head_block.substr(line_begin, line_end - line_begin);
    const std::size_t colon = line.find(':');
    if (colon != std::string::npos) {
      std::string field = TrimWhitespace(line.substr(0, colon));
      if (field.size() == name.size()) {
        bool same = true;
        for (std::size_t i = 0; i < field.size(); ++i) {
          const char a = static_cast<char>(std::tolower(static_cast<unsigned char>(field[i])));
          const char b = static_cast<char>(std::tolower(static_cast<unsigned char>(name[i])));
          if (a != b) {
            same = false;
            break;
          }
        }
        if (same) {
          if (out != nullptr) {
            *out = TrimWhitespace(line.substr(colon + 1));
          }
          return true;
        }
      }
    }
    line_begin = line_end + 2;  // 跳过 "\r\n"
  }
  return false;
}

// 严格整数解析（用于 prio 这类查询参数）。
// 旧实现 `std::stoi` 遇到 "abc" 直接抛异常并打死网关；且 "5x" 会被静默截成 5。
enum class IntParseStatus { kOk, kInvalid, kOutOfRange };

inline IntParseStatus ParseInt32Strict(const std::string& text, std::int32_t* out) {
  if (out != nullptr) {
    *out = 0;
  }
  const std::string trimmed = TrimWhitespace(text);
  if (trimmed.empty()) {
    return IntParseStatus::kInvalid;
  }
  std::size_t i = 0;
  bool negative = false;
  if (trimmed[0] == '+' || trimmed[0] == '-') {
    negative = (trimmed[0] == '-');
    ++i;
    if (i >= trimmed.size()) {
      return IntParseStatus::kInvalid;
    }
  }
  long long acc = 0;
  for (; i < trimmed.size(); ++i) {
    const char c = trimmed[i];
    if (c < '0' || c > '9') {
      return IntParseStatus::kInvalid;
    }
    acc = acc * 10 + (c - '0');
    if (acc > 2147483648LL) {
      return IntParseStatus::kOutOfRange;
    }
  }
  if (negative) {
    acc = -acc;
  }
  if (acc < -2147483648LL || acc > 2147483647LL) {
    return IntParseStatus::kOutOfRange;
  }
  if (out != nullptr) {
    *out = static_cast<std::int32_t>(acc);
  }
  return IntParseStatus::kOk;
}

// 判断请求是否声明了分块传输（本网关不支持，调用方应回 400 而不是猜长度）。
inline bool HasChunkedTransferEncoding(const std::string& head_block) {
  std::string value;
  if (!FindHeaderValue(head_block, "Transfer-Encoding", &value)) {
    return false;
  }
  for (char& c : value) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return value.find("chunked") != std::string::npos;
}

}  // namespace amr_dispatcher_tools
