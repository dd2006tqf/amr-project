// 静态 Bearer Token 鉴权（纯函数，无 IO）。
//
// 默认关闭：expected_token 为空时 IsAuthorized 恒真——现网与所有 e2e 的行为
// 逐字不变。配置了 token 才要求 `Authorization: Bearer <token>`。
//
// 比较用常量时间，避免按字符提前返回泄露 token 前缀。
#pragma once

#include <cstddef>
#include <string>

#include "amr_dispatcher_tools/http_limits.hpp"

namespace amr_dispatcher_tools {

// 大小写不敏感地判断 s 是否以 prefix 开头。
inline bool StartsWithIgnoreCase(const std::string& s, const std::string& prefix) {
  if (s.size() < prefix.size()) {
    return false;
  }
  for (std::size_t i = 0; i < prefix.size(); ++i) {
    const char a = static_cast<char>(std::tolower(static_cast<unsigned char>(s[i])));
    const char b = static_cast<char>(std::tolower(static_cast<unsigned char>(prefix[i])));
    if (a != b) {
      return false;
    }
  }
  return true;
}

// 常量时间字符串比较：长度不同直接返回 false（长度本身不是秘密），
// 同长时始终遍历全部字符，用位或累计差异。
inline bool ConstantTimeEquals(const std::string& lhs, const std::string& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  unsigned char diff = 0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    diff = static_cast<unsigned char>(diff |
                                      static_cast<unsigned char>(lhs[i] ^ rhs[i]));
  }
  return diff == 0;
}

// 从 `Authorization: Bearer xxx` 里取出 token；非 Bearer 或空值返回 false。
inline bool ExtractBearerToken(const std::string& authorization_header, std::string* token) {
  if (token != nullptr) {
    token->clear();
  }
  const std::string trimmed = TrimWhitespace(authorization_header);
  const std::string scheme = "Bearer";
  if (!StartsWithIgnoreCase(trimmed, scheme)) {
    return false;
  }
  if (trimmed.size() > scheme.size()) {
    const char sep = trimmed[scheme.size()];
    if (sep != ' ' && sep != '\t') {
      return false;  // "Bearerx" 不是合法 scheme
    }
  }
  const std::string value = TrimWhitespace(trimmed.substr(scheme.size()));
  if (value.empty()) {
    return false;
  }
  if (token != nullptr) {
    *token = value;
  }
  return true;
}

// 是否需要鉴权 + 是否通过。
//   expected_token 为空  -> 鉴权关闭，恒真（默认关闭，兼容红线）
//   否则比较 Bearer token；缺失/格式错误/不匹配 -> false
inline bool IsAuthorized(const std::string& authorization_header,
                         const std::string& expected_token) {
  if (expected_token.empty()) {
    return true;
  }
  std::string presented;
  if (!ExtractBearerToken(authorization_header, &presented)) {
    return false;
  }
  return ConstantTimeEquals(presented, expected_token);
}

// 免鉴权的只读静态资源（操作台页面本身）：不需要 token 才能打开页面，
// 页面里的每个 API 调用仍各自带 token。空 = 没有端点免鉴权。
inline bool IsAuthExemptPath(const std::string& path) {
  return path == "/" || path == "/index.html";
}

}  // namespace amr_dispatcher_tools
